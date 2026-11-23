// ============================================================
// gpu_main.cpp  --  MLP 784-128-64-10 (MNIST) entrenada en GPU con OpenCL
//
// Funcionalidad minima segun la Propuesta P2 (Algoritmo 2):
//   * 1 kernel, 1 solo work-group (global = local = P hilos).
//   * Cada lanzamiento procesa G imagenes en orden (SGD por imagen).
//   * Las 7 fases de cada imagen se separan con barrier().
//   * Mismos pesos iniciales que main.cpp (Eigen Random + DL_SEED).
//   * Al final: checksum, MSE/accuracy por epoca y, opcionalmente,
//     diferencia numerica contra los parametros del baseline (DL_REF).
//
// Compilar (Linux):
//   g++ -O2 -std=c++17 -I/usr/include/eigen3 -I. \
//       gpu_main.cpp lector.cpp -o mlp_gpu -lOpenCL
//
// Ejecutar (desde la misma carpeta que main, para que ../data/mnist exista):
//   ./mlp_gpu [epocas]
//
// Variables de entorno (las mismas del baseline + algunas propias):
//   DL_SAMPLES  cantidad de imagenes (def. 1000)
//   DL_SEED     semilla de Eigen::Random (def. 1)
//   DL_QUIET=1  silencia la salida por epoca
//   DL_PARAMS   archivo binario donde guardar W1 b1 W2 b2 W3 b3 (double)
//   DL_REF      archivo DL_PARAMS del baseline para calcular diferencia
//   DL_GROUP    imagenes por lanzamiento de kernel (def. 100)
//   DL_LOCAL    hilos del work-group P (def. 128)
//   DL_FP32=1   fuerza float (por defecto usa double si la GPU lo soporta)
// ============================================================

#define CL_TARGET_OPENCL_VERSION 300
#include <CL/cl.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "runtime.h"
#include "mnist.h"

// (solo para poder compilar; el valor no sera comparable con el del baseline).
#if __has_include("checksum.h")
#include "checksum.h"
#else
static const uint64_t CHECKSUM_INICIAL = 1469598103934665603ULL;
template <class M>
uint64_t hash_params(const M& m, uint64_t h) {
    const unsigned char* p = reinterpret_cast<const unsigned char*>(m.data());
    for (size_t i = 0; i < m.size() * sizeof(double); ++i) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}
#endif

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;

// Verificacion de llamadas OpenCL
#define CK(call)                                                              \
    do {                                                                      \
        cl_int e_ = (call);                                                   \
        if (e_ != CL_SUCCESS)                                                 \
            throw std::runtime_error(std::string(#call) +                     \
                                     " fallo, codigo OpenCL " +               \
                                     std::to_string(e_));                     \
    } while (0)

// ============================================================
// KERNEL OpenCL (un work-group; cada hilo procesa filas/columnas)
// Pesos en la GPU: ROW-MAJOR  W[i*cols + j]  (fila i = neurona i)
// REAL se define al compilar: double o float
// ============================================================
static const char* KERNEL_SRC = R"CLC(
#ifdef USE_FP64
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#endif

inline REAL sigm(REAL z) { return (REAL)1 / ((REAL)1 + exp(-z)); }

#define FENCE (CLK_GLOBAL_MEM_FENCE | CLK_LOCAL_MEM_FENCE)

__kernel void train(
    __global REAL* W1, __global REAL* b1,
    __global REAL* W2, __global REAL* b2,
    __global REAL* W3, __global REAL* b3,
    __global const REAL* X, __global const REAL* Y,
    __global REAL* a1, __global REAL* a2, __global REAL* a3,
    __global REAL* dZ1, __global REAL* dZ2, __global REAL* dZ3,
    __global REAL* metrics,
    const int epoch, const int start, const int count, const REAL lr)
{
    const int t = get_local_id(0);     // id del hilo dentro del work-group
    const int P = get_local_size(0);   // numero de hilos

    for (int s = start; s < start + count; s++) {      // imagenes en orden (SGD)
        __global const REAL* x = X + (size_t)s * 784;
        __global const REAL* y = Y + (size_t)s * 10;

        // ---- Fase 1: forward capa 1 (784 -> 128), hilo = fila ----
        for (int i = t; i < 128; i += P) {
            REAL z = b1[i];
            for (int j = 0; j < 784; j++) z += W1[i * 784 + j] * x[j];
            a1[i] = sigm(z);
        }
        barrier(FENCE);

        // ---- Fase 2: forward capa 2 (128 -> 64) ----
        for (int i = t; i < 64; i += P) {
            REAL z = b2[i];
            for (int j = 0; j < 128; j++) z += W2[i * 128 + j] * a1[j];
            a2[i] = sigm(z);
        }
        barrier(FENCE);

        // ---- Fase 3: forward capa 3 (64 -> 10) ----
        for (int i = t; i < 10; i += P) {
            REAL z = b3[i];
            for (int j = 0; j < 64; j++) z += W3[i * 64 + j] * a2[j];
            a3[i] = sigm(z);
        }
        barrier(FENCE);

        // ---- Fase 4: salida. dZ3, b3 (y metricas en el hilo 0) ----
        for (int i = t; i < 10; i += P) {
            REAL dA = (REAL)0.2 * (a3[i] - y[i]);
            dZ3[i] = dA * a3[i] * ((REAL)1 - a3[i]);
            b3[i] -= lr * dZ3[i];
        }
        if (t == 0) {                       // MSE y acierto (10 valores: barato)
            REAL m = 0; int pred = 0, real = 0;
            for (int i = 0; i < 10; i++) {
                REAL d = a3[i] - y[i];
                m += d * d;
                if (a3[i] > a3[pred]) pred = i;
                if (y[i]  > y[real])  real = i;
            }
            metrics[2 * epoch]     += m / (REAL)10;
            metrics[2 * epoch + 1] += (pred == real) ? (REAL)1 : (REAL)0;
        }
        barrier(FENCE);

        // ---- Fase 5: backprop capa 2, hilo = columna j de W3 ----
        // dA se calcula con la columna ANTES de actualizarla
        for (int j = t; j < 64; j += P) {
            REAL dA = 0;
            for (int i = 0; i < 10; i++) dA += W3[i * 64 + j] * dZ3[i];
            dZ2[j] = dA * a2[j] * ((REAL)1 - a2[j]);
            for (int i = 0; i < 10; i++) W3[i * 64 + j] -= lr * dZ3[i] * a2[j];
            b2[j] -= lr * dZ2[j];
        }
        barrier(FENCE);

        // ---- Fase 6: backprop capa 1, hilo = columna k de W2 ----
        for (int k = t; k < 128; k += P) {
            REAL dA = 0;
            for (int i = 0; i < 64; i++) dA += W2[i * 128 + k] * dZ2[i];
            dZ1[k] = dA * a1[k] * ((REAL)1 - a1[k]);
            for (int i = 0; i < 64; i++) W2[i * 128 + k] -= lr * dZ2[i] * a1[k];
            b1[k] -= lr * dZ1[k];
        }
        barrier(FENCE);

        // ---- Fase 7: actualizar W1, hilo = fila i ----
        for (int i = t; i < 128; i += P) {
            REAL g = lr * dZ1[i];
            for (int j = 0; j < 784; j++) W1[i * 784 + j] -= g * x[j];
        }
        barrier(FENCE);   // antes de la siguiente imagen
    }
}
)CLC";

// ============================================================
// Utilidades de conversion Eigen (col-major, double) <-> GPU (row-major)
// ============================================================
static std::vector<char> pack(const MatrixXd& m, bool fp64) {
    const size_t es = fp64 ? 8 : 4;
    std::vector<char> out(m.size() * es);
    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++) {
            const size_t k = (size_t)i * m.cols() + j;
            if (fp64) { double v = m(i, j); std::memcpy(&out[k * es], &v, 8); }
            else      { float  v = (float)m(i, j); std::memcpy(&out[k * es], &v, 4); }
        }
    return out;
}

static void unpack(const std::vector<char>& raw, MatrixXd& m, bool fp64) {
    const size_t es = fp64 ? 8 : 4;
    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++) {
            const size_t k = (size_t)i * m.cols() + j;
            if (fp64) { double v; std::memcpy(&v, &raw[k * es], 8); m(i, j) = v; }
            else      { float  v; std::memcpy(&v, &raw[k * es], 4); m(i, j) = v; }
        }
}

// ============================================================
int run(int argc, char** argv) {
    dl::initialize();

    const int num_muestras = dl::env_int("DL_SAMPLES", 1000);
    const int epochs = (argc > 1) ? dl::integer(argv[1]) : 1000;
    int G = dl::env_int("DL_GROUP", 100);
    int P = dl::env_int("DL_LOCAL", 128);
    if (G > num_muestras) G = num_muestras;
    const double learning_rate = 0.1;

    // ---------- Pesos iniciales: MISMO orden de Random que main.cpp ----------
    MatrixXd W1 = MatrixXd::Random(128, 784) * 0.1;
    MatrixXd b1 = MatrixXd::Zero(128, 1);
    MatrixXd W2 = MatrixXd::Random(64, 128) * 0.1;
    MatrixXd b2 = MatrixXd::Zero(64, 1);
    MatrixXd W3 = MatrixXd::Random(10, 64) * 0.1;
    MatrixXd b3 = MatrixXd::Zero(10, 1);

    // ---------- Datos MNIST (igual que el baseline) ----------
    std::ifstream image_file("../data/mnist/train-images-idx3-ubyte", std::ios::binary);
    std::ifstream label_file("../data/mnist/train-labels-idx1-ubyte", std::ios::binary);
    if (!image_file || !label_file) {
        std::cerr << "No se pudieron abrir los archivos MNIST\n";
        return 1;
    }
    MatrixXd Xm(num_muestras, 784), Ym(num_muestras, 10);
    for (int i = 0; i < num_muestras; i++) {
        Sample s = load_sample(image_file, label_file, i);
        Xm.row(i) = s.x.transpose();
        Ym.row(i) = s.y.transpose();
    }

    // ---------- OpenCL: plataforma, dispositivo GPU, contexto, cola ----------
    cl_device_id dev = nullptr;
    {
        cl_uint np = 0;
        CK(clGetPlatformIDs(0, nullptr, &np));
        std::vector<cl_platform_id> ps(np);
        CK(clGetPlatformIDs(np, ps.data(), nullptr));
        for (auto p : ps) {
            cl_uint nd = 0;
            if (clGetDeviceIDs(p, CL_DEVICE_TYPE_GPU, 1, &dev, &nd) == CL_SUCCESS && nd > 0) break;
            dev = nullptr;
        }
    }
    if (!dev) throw std::runtime_error("No se encontro ninguna GPU OpenCL (instalar intel-opencl-icd / compute-runtime)");

    char name[256] = {0}, exts[8192] = {0};
    CK(clGetDeviceInfo(dev, CL_DEVICE_NAME, sizeof(name), name, nullptr));
    CK(clGetDeviceInfo(dev, CL_DEVICE_EXTENSIONS, sizeof(exts), exts, nullptr));
    const bool fp64 = std::strstr(exts, "cl_khr_fp64") && !std::getenv("DL_FP32");
    const size_t es = fp64 ? 8 : 4;
    std::cerr << "GPU: " << name << " | precision: " << (fp64 ? "double (fp64)" : "float (fp32)") << "\n";
    if (!fp64)
        std::cerr << "AVISO: en fp32 el resultado NO sera bit a bit igual al baseline (double); "
                     "comparar por diferencia numerica, MSE y accuracy.\n";

    cl_int err;
    cl_context ctx = clCreateContext(nullptr, 1, &dev, nullptr, nullptr, &err); CK(err);
    cl_command_queue q = clCreateCommandQueueWithProperties(ctx, dev, nullptr, &err); CK(err);

    // ---------- Compilar el kernel ----------
    cl_program prog = clCreateProgramWithSource(ctx, 1, &KERNEL_SRC, nullptr, &err); CK(err);
    std::string opts = fp64 ? "-DREAL=double -DUSE_FP64" : "-DREAL=float";
    if (clBuildProgram(prog, 1, &dev, opts.c_str(), nullptr, nullptr) != CL_SUCCESS) {
        size_t n = 0;
        clGetProgramBuildInfo(prog, dev, CL_PROGRAM_BUILD_LOG, 0, nullptr, &n);
        std::string log(n, '\0');
        clGetProgramBuildInfo(prog, dev, CL_PROGRAM_BUILD_LOG, n, &log[0], nullptr);
        throw std::runtime_error("Error compilando kernel:\n" + log);
    }
    cl_kernel kern = clCreateKernel(prog, "train", &err); CK(err);

    // Ajustar P al maximo permitido por el dispositivo/kernel
    size_t maxwg = 0;
    CK(clGetKernelWorkGroupInfo(kern, dev, CL_KERNEL_WORK_GROUP_SIZE, sizeof(maxwg), &maxwg, nullptr));
    if ((size_t)P > maxwg) P = (int)maxwg;
    std::cerr << "Work-group: P=" << P << " hilos (max " << maxwg << "), G=" << G
              << " imagenes por kernel\n";

    // ---------- Buffers ----------
    auto mk = [&](size_t bytes, const void* host) {
        cl_mem m = clCreateBuffer(ctx, CL_MEM_READ_WRITE | (host ? CL_MEM_COPY_HOST_PTR : 0),
                                  bytes, const_cast<void*>(host), &err);
        CK(err);
        return m;
    };
    auto pW1 = pack(W1, fp64), pb1 = pack(b1, fp64), pW2 = pack(W2, fp64),
         pb2 = pack(b2, fp64), pW3 = pack(W3, fp64), pb3 = pack(b3, fp64);
    // X e Y tambien en row-major (una imagen por fila)
    auto pX = pack(Xm, fp64), pY = pack(Ym, fp64);
    std::vector<char> zeros(std::max<size_t>(128 * es, (size_t)epochs * 2 * es), 0);

    cl_mem dW1 = mk(pW1.size(), pW1.data()), db1 = mk(pb1.size(), pb1.data());
    cl_mem dW2 = mk(pW2.size(), pW2.data()), db2 = mk(pb2.size(), pb2.data());
    cl_mem dW3 = mk(pW3.size(), pW3.data()), db3 = mk(pb3.size(), pb3.data());
    cl_mem dX = mk(pX.size(), pX.data()), dY = mk(pY.size(), pY.data());
    cl_mem da1 = mk(128 * es, zeros.data()), da2 = mk(64 * es, zeros.data()), da3 = mk(10 * es, zeros.data());
    cl_mem dz1 = mk(128 * es, zeros.data()), dz2 = mk(64 * es, zeros.data()), dz3 = mk(10 * es, zeros.data());
    cl_mem dM = mk((size_t)epochs * 2 * es, zeros.data());

    // Argumentos estaticos (los de epoch/start/count cambian en cada lanzamiento)
    cl_mem bufs[15] = {dW1, db1, dW2, db2, dW3, db3, dX, dY, da1, da2, da3, dz1, dz2, dz3, dM};
    for (int i = 0; i < 15; i++) CK(clSetKernelArg(kern, i, sizeof(cl_mem), &bufs[i]));
    const float  lr_f = (float)learning_rate;
    const double lr_d = learning_rate;
    CK(clSetKernelArg(kern, 18, es, fp64 ? (const void*)&lr_d : (const void*)&lr_f));

    // ---------- Entrenamiento (zona cronometrada) ----------
    auto t0 = std::chrono::steady_clock::now();
    const size_t gsz = (size_t)P, lsz = (size_t)P;   // global = local -> 1 work-group
    for (int e = 0; e < epochs; e++) {
        CK(clSetKernelArg(kern, 15, sizeof(int), &e));
        for (int start = 0; start < num_muestras; start += G) {
            int count = std::min(G, num_muestras - start);
            CK(clSetKernelArg(kern, 16, sizeof(int), &start));
            CK(clSetKernelArg(kern, 17, sizeof(int), &count));
            auto l0 = std::chrono::steady_clock::now();
            CK(clEnqueueNDRangeKernel(q, kern, 1, nullptr, &gsz, &lsz, 0, nullptr, nullptr));
            if (std::getenv("DL_SYNC")) {
                CK(clFinish(q));
                if (std::getenv("DL_TRACE"))
                    std::cerr << "ep " << e << " start " << start << " us "
                            << std::chrono::duration_cast<std::chrono::microseconds>(
                                    std::chrono::steady_clock::now() - l0).count() << "\n";
            }
        }
    }
    CK(clFinish(q));
    auto t1 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);

    // ---------- Leer resultados ----------
    auto rd = [&](cl_mem m, size_t bytes) {
        std::vector<char> v(bytes);
        CK(clEnqueueReadBuffer(q, m, CL_TRUE, 0, bytes, v.data(), 0, nullptr, nullptr));
        return v;
    };
    unpack(rd(dW1, pW1.size()), W1, fp64); unpack(rd(db1, pb1.size()), b1, fp64);
    unpack(rd(dW2, pW2.size()), W2, fp64); unpack(rd(db2, pb2.size()), b2, fp64);
    unpack(rd(dW3, pW3.size()), W3, fp64); unpack(rd(db3, pb3.size()), b3, fp64);
    auto mraw = rd(dM, (size_t)epochs * 2 * es);

    // MSE / accuracy por epoca (acumulados en la GPU)
    for (int e = 0; e < epochs; e++) {
        double mse, hit;
        if (fp64) { std::memcpy(&mse, &mraw[(2 * e) * 8], 8); std::memcpy(&hit, &mraw[(2 * e + 1) * 8], 8); }
        else { float a, b; std::memcpy(&a, &mraw[(2 * e) * 4], 4); std::memcpy(&b, &mraw[(2 * e + 1) * 4], 4); mse = a; hit = b; }
        cout << "MSE promedio: " << mse / num_muestras
             << " | Accuracy: " << hit / num_muestras * 100.0 << "%\n";
    }

    // ---------- Checksum (mismo orden/tipos que el baseline) ----------
    VectorXd vb1 = b1.col(0), vb2 = b2.col(0), vb3 = b3.col(0);
    dl::finish(W1, vb1, W2, vb2, W3, vb3);     // escribe DL_PARAMS si esta definida
    uint64_t h = CHECKSUM_INICIAL;
    h = hash_params(W1, h); h = hash_params(vb1, h);
    h = hash_params(W2, h); h = hash_params(vb2, h);
    h = hash_params(W3, h); h = hash_params(vb3, h);
    cout << "\nChecksum parametros: " << std::hex << h << std::dec << "\n";

    // ---------- Diferencia numerica contra el baseline (opcional) ----------
    if (const char* ref = std::getenv("DL_REF")) {
        const size_t total = 128 * 784 + 128 + 64 * 128 + 64 + 10 * 64 + 10;
        std::vector<double> r(total);
        std::ifstream f(ref, std::ios::binary);
        f.read(reinterpret_cast<char*>(r.data()), total * sizeof(double));
        if (!f) {
            std::cerr << "No se pudo leer DL_REF=" << ref << "\n";
        } else {
            const double* mine[6] = {W1.data(), vb1.data(), W2.data(), vb2.data(), W3.data(), vb3.data()};
            const size_t len[6] = {128 * 784, 128, 64 * 128, 64, 10 * 64, 10};
            const char* nm[6] = {"W1", "b1", "W2", "b2", "W3", "b3"};
            size_t off = 0;
            cout << "\nDiferencia numerica vs baseline (max |GPU - CPU|):\n";
            for (int k = 0; k < 6; k++) {
                double md = 0;
                for (size_t i = 0; i < len[k]; i++) md = std::max(md, std::abs(mine[k][i] - r[off + i]));
                cout << "  " << nm[k] << ": " << md << "\n";
                off += len[k];
            }
        }
    }

    cout << "\n=== GPU OpenCL (" << name << ") ===\n";
    cout << "\nTiempo total: " << elapsed.count() << " us\n";

    for (cl_mem m : bufs) clReleaseMemObject(m);
    clReleaseKernel(kern); clReleaseProgram(prog);
    clReleaseCommandQueue(q); clReleaseContext(ctx);
    return 0;
}

int main(int argc, char** argv) {
    try { return run(argc, argv); }
    catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}