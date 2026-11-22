// ============================================================
// emulador.cpp
//
// Emulador de la MLP 784-128-64-10 (SGD por imagen, sigmoide, MSE)
// en tres modos aritmeticos, partiendo de pesos_iniciales.bin:
//
//   double : MISMAS expresiones Eigen que main.cpp. Con los mismos
//            pesos iniciales debe dar el MISMO checksum que main
//            (prueba de fondo del emulador).
//   float  : igual que double pero en float32 (referencia para la GPU,
//            que no tiene fp64; separa el efecto de la precision del
//            efecto del paralelismo).
//   fixed  : aritmetica entera en punto fijo, la que implementara el
//            acelerador en la FPGA. Es el "golden model" del RTL.
//
// El algoritmo NO cambia: SGD por imagen en orden, todos los gradientes
// con los pesos previos a la actualizacion de esa imagen, y luego se
// actualizan W3, W2, W1 (igual que main.cpp).
//
// Uso:
//   ./emulador <double|float|fixed> <epocas> [opciones]
//   ./emulador --comparar A.bin B.bin     (diferencia numerica, B = ref.)
//   ./emulador --test-lut                 (error de la sigmoide por LUT)
//
// Opciones:
//   --n=100                 numero de imagenes de entrenamiento
//   --pesos=pesos_iniciales.bin
//   --datos=../data/mnist
//   --volcar=salida.bin     pesos finales como double (mismo formato
//                           que pesos_iniciales.bin) para --comparar
//   --cada=1                imprimir MSE/accuracy cada N epocas
//   --fw=20 | --fw1= --fw2= --fw3=   bits fraccionarios de W y b
//                           (solo fixed; W ocupa 24 bits en total)
//   --redondeo=cercano|estocastico|piso   redondeo de la actualizacion
//                           de pesos (solo fixed)
//   --semilla=1             semilla base de los LFSR (estocastico)
//
// ---------------- FORMATOS DE PUNTO FIJO (propuesta) -----------
// Qi.f = i bits enteros (con signo si corresponde), f fraccionarios.
//   x, a1, a2, a3   Q0.16 sin signo          (16 bits)
//   W, b            24 bits con signo, FW fraccionarios (def. 20)
//   acumulador      F = FW+16, saturado a 48 bits
//   z               Q4.12 con signo (16 bits), saturado a +-8
//   dZ, dA          Q2.22 con signo (24 bits)
//   dZ' = lr*dZ     F = 30, 27 bits con signo
//   lr = 6554/65536,  2/10 = 13107/65536
//   sigmoide: LUT de 1025 puntos sobre [-8,8], paso 1/64, interpolacion
//             lineal con redondeo; derivada a*(1-a) sin segunda LUT.
// Redondeo: todas las operaciones usan "al mas cercano"
//   (v + 2^(s-1)) >> s con desplazamiento aritmetico, salvo la
//   actualizacion de pesos, que usa el modo elegido.
//
// ---------------- HASH DE ENTEROS (para RTL vs emulador) -------
// Se aplica FNV-1a (constantes de checksum.h) sobre los bytes de cada
// valor como int32 little-endian, en este orden: W1,b1,W2,b2,W3,b3;
// cada W en orden FILA (W[j][k], j=fila, k=columna), b en orden j.
//
// ---------------- REDONDEO ESTOCASTICO -------------------------
// Un LFSR de Galois de 32 bits (mascara 0x80200003) POR FILA (por
// "lane"). Se avanza una vez por cada peso actualizado, en orden de
// columna, y una vez mas para el sesgo; los bits bajos del estado
// forman R. Semilla de la fila: semilla ^ (0x9E3779B1 * (capa*1024+fila+1)).
// Asi cada lane de la FPGA puede reproducir su secuencia en paralelo.
// ============================================================

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "checksum.h"
#include "mnist.h"

using std::cout;
using Eigen::Dynamic;
using Eigen::MatrixXd;

// ------------------------------------------------------------
// Dimensiones de la red
// ------------------------------------------------------------
constexpr int N_IN  = 784;
constexpr int N_H1  = 128;
constexpr int N_H2  = 64;
constexpr int N_OUT = 10;

constexpr size_t TOTAL_PARAMS =
    (size_t)N_H1 * N_IN + N_H1 + (size_t)N_H2 * N_H1 + N_H2 +
    (size_t)N_OUT * N_H2 + N_OUT;   // 109386

// ------------------------------------------------------------
// Opciones de linea de comandos
// ------------------------------------------------------------
enum class Redondeo { Cercano, Estocastico, Piso };

struct Opciones {
    std::string modo = "double";
    int epocas = 2;
    int n = 100;
    std::string pesos = "pesos_iniciales.bin";
    std::string datos = "../data/mnist";
    std::string volcar;
    std::string vectores;   // prefijo de los .hex dorados para el RTL
    int cada = 1;
    int fw[3] = {20, 20, 20};
    Redondeo red = Redondeo::Cercano;
    uint32_t semilla = 1;
    bool test_lut = false;
    bool test_bw = false;   // autoprueba sintetica de MATVEC_T + DERIV
    int caso = 0;           // caso de datos de la autoprueba (0..3)
    std::string cmp_a, cmp_b;
};

static void uso() {
    std::cerr <<
        "Uso: ./emulador <double|float|fixed> <epocas> [opciones]\n"
        "     ./emulador --comparar A.bin B.bin\n"
        "     ./emulador --test-lut\n"
        "     ./emulador --test-backward --caso=0..3 --semilla=N --vectores=PREF\n"
        "Opciones: --n= --pesos= --datos= --volcar= --vectores= --cada= --fw= --fw1=\n"
        "          --fw2= --fw3= --redondeo=cercano|estocastico|piso\n"
        "          --semilla=\n";
}

static bool empieza_con(const std::string& s, const char* p) {
    return s.rfind(p, 0) == 0;
}

static bool parsear(int argc, char** argv, Opciones& op) {
    int pos = 0;
    for (int i = 1; i < argc; i++) {
        std::string s = argv[i];
        if (!empieza_con(s, "--")) {
            if (pos == 0) op.modo = s;
            else if (pos == 1) op.epocas = std::atoi(s.c_str());
            else return false;
            pos++;
            continue;
        }
        std::string clave = s.substr(2), val;
        size_t eq = clave.find('=');
        if (eq != std::string::npos) {
            val = clave.substr(eq + 1);
            clave = clave.substr(0, eq);
        }
        if (clave == "comparar") {
            if (i + 2 >= argc) return false;
            op.cmp_a = argv[++i];
            op.cmp_b = argv[++i];
        } else if (clave == "test-lut") op.test_lut = true;
        else if (clave == "test-backward") op.test_bw = true;
        else if (clave == "caso") op.caso = std::atoi(val.c_str());
        else if (clave == "n") op.n = std::atoi(val.c_str());
        else if (clave == "pesos") op.pesos = val;
        else if (clave == "datos") op.datos = val;
        else if (clave == "volcar") op.volcar = val;
        else if (clave == "vectores") op.vectores = val;
        else if (clave == "cada") op.cada = std::max(1, std::atoi(val.c_str()));
        else if (clave == "fw") op.fw[0] = op.fw[1] = op.fw[2] = std::atoi(val.c_str());
        else if (clave == "fw1") op.fw[0] = std::atoi(val.c_str());
        else if (clave == "fw2") op.fw[1] = std::atoi(val.c_str());
        else if (clave == "fw3") op.fw[2] = std::atoi(val.c_str());
        else if (clave == "semilla") op.semilla = (uint32_t)std::strtoul(val.c_str(), nullptr, 10);
        else if (clave == "redondeo") {
            if (val == "cercano") op.red = Redondeo::Cercano;
            else if (val == "estocastico") op.red = Redondeo::Estocastico;
            else if (val == "piso") op.red = Redondeo::Piso;
            else return false;
        } else return false;
    }
    return true;
}

// ------------------------------------------------------------
// Utilidades de hash (mismas constantes que checksum.h)
// ------------------------------------------------------------
static uint64_t fnv_bytes(const void* d, size_t nb, uint64_t h) {
    const unsigned char* p = static_cast<const unsigned char*>(d);
    for (size_t i = 0; i < nb; i++) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

// ------------------------------------------------------------
// Lectura de pesos_iniciales.bin y comparacion de archivos
// ------------------------------------------------------------
static bool leer_doubles(const std::string& ruta, std::vector<double>& v) {
    std::ifstream f(ruta, std::ios::binary | std::ios::ate);
    if (!f) return false;
    if (f.tellg() != (std::streamoff)(TOTAL_PARAMS * sizeof(double))) return false;
    f.seekg(0, std::ios::beg);
    v.resize(TOTAL_PARAMS);
    f.read(reinterpret_cast<char*>(v.data()),
           (std::streamsize)(TOTAL_PARAMS * sizeof(double)));
    return static_cast<bool>(f);
}

struct PesosD {
    MatrixXd W1, b1, W2, b2, W3, b3;
};

static bool cargar_pesos(const std::string& ruta, PesosD& p) {
    std::vector<double> v;
    if (!leer_doubles(ruta, v)) return false;
    const double* q = v.data();
    auto tomar = [&q](int filas, int cols) {
        MatrixXd m = Eigen::Map<const MatrixXd>(q, filas, cols);
        q += (size_t)filas * cols;
        return m;
    };
    p.W1 = tomar(N_H1, N_IN);  p.b1 = tomar(N_H1, 1);
    p.W2 = tomar(N_H2, N_H1);  p.b2 = tomar(N_H2, 1);
    p.W3 = tomar(N_OUT, N_H2); p.b3 = tomar(N_OUT, 1);
    return true;
}

// Diferencia numerica entre dos archivos de pesos (B es la referencia)
static int comparar(const std::string& ra, const std::string& rb) {
    std::vector<double> a, b;
    if (!leer_doubles(ra, a) || !leer_doubles(rb, b)) {
        std::cerr << "No se pudieron leer los archivos de pesos\n";
        return 1;
    }
    const char* nombre[6] = {"W1", "b1", "W2", "b2", "W3", "b3"};
    const size_t tam[6] = {(size_t)N_H1 * N_IN, N_H1, (size_t)N_H2 * N_H1,
                           N_H2, (size_t)N_OUT * N_H2, N_OUT};
    std::printf("Comparacion A=%s  vs  B(ref)=%s\n", ra.c_str(), rb.c_str());
    std::printf("%-6s %-12s %-12s %-12s\n", "Grupo", "max|A-B|", "RMS(A-B)", "||A-B||/||B||");
    size_t off = 0;
    double g_max = 0, g_s2 = 0, g_b2 = 0;
    for (int g = 0; g < 6; g++) {
        double mx = 0, s2 = 0, b2 = 0;
        for (size_t i = 0; i < tam[g]; i++) {
            double d = a[off + i] - b[off + i];
            mx = std::max(mx, std::fabs(d));
            s2 += d * d;
            b2 += b[off + i] * b[off + i];
        }
        std::printf("%-6s %-12.3e %-12.3e %-12.3e\n", nombre[g], mx,
                    std::sqrt(s2 / (double)tam[g]), b2 > 0 ? std::sqrt(s2 / b2) : 0.0);
        g_max = std::max(g_max, mx); g_s2 += s2; g_b2 += b2;
        off += tam[g];
    }
    std::printf("%-6s %-12.3e %-12.3e %-12.3e\n", "TOTAL", g_max,
                std::sqrt(g_s2 / (double)TOTAL_PARAMS),
                g_b2 > 0 ? std::sqrt(g_s2 / g_b2) : 0.0);
    return 0;
}

// ------------------------------------------------------------
// Resultado comun a los tres modos
// ------------------------------------------------------------
struct Resultado {
    uint64_t checksum = 0;
    long long us = 0;
    std::vector<double> params;   // W1,b1,W2,b2,W3,b3 en column-major
};

// ============================================================
// MODOS double y float (mismas expresiones que main.cpp)
// ============================================================
template <typename T>
static Eigen::Matrix<T, Dynamic, 1> sigmoide(const Eigen::Matrix<T, Dynamic, 1>& z) {
    return T(1) / (T(1) + (-z.array()).exp());
}

template <typename T>
static void agregar(std::vector<double>& out, const Eigen::Matrix<T, Dynamic, Dynamic>& m) {
    for (Eigen::Index i = 0; i < m.size(); i++) out.push_back((double)m.data()[i]);
}

template <typename T>
static uint64_t hash_fp(const Eigen::Matrix<T, Dynamic, Dynamic>& m, uint64_t h) {
    if constexpr (std::is_same<T, double>::value) {
        return hash_params(m, h);                 // la de checksum.h
    } else {
        return fnv_bytes(m.data(), (size_t)m.size() * sizeof(T), h);
    }
}

template <typename T>
static Resultado entrenar_fp(const Opciones& op, const PesosD& p0,
                             const std::vector<Sample>& muestras) {
    using Vec = Eigen::Matrix<T, Dynamic, 1>;
    using Mat = Eigen::Matrix<T, Dynamic, Dynamic>;

    Mat W1 = p0.W1.cast<T>(), b1 = p0.b1.cast<T>();
    Mat W2 = p0.W2.cast<T>(), b2 = p0.b2.cast<T>();
    Mat W3 = p0.W3.cast<T>(), b3 = p0.b3.cast<T>();

    std::vector<Vec> X, Y;
    for (const Sample& s : muestras) {
        X.push_back(s.x.cast<T>());
        Y.push_back(s.y.cast<T>());
    }

    const T learning_rate = T(0.1);
    const int num_muestras = (int)muestras.size();

    auto start = std::chrono::steady_clock::now();

    for (int epoch = 0; epoch < op.epocas; epoch++) {
        double mse_total = 0.0;
        int aciertos = 0;

        for (int i = 0; i < num_muestras; i++) {
            const Vec& x = X[i];
            const Vec& y = Y[i];

            // ---- forward ----
            Vec z1 = W1 * x + b1;
            Vec a1 = sigmoide<T>(z1);
            Vec z2 = W2 * a1 + b2;
            Vec a2 = sigmoide<T>(z2);
            Vec z3 = W3 * a2 + b3;
            Vec a3 = sigmoide<T>(z3);

            // ---- perdida MSE ----
            T mse = (a3 - y).squaredNorm() / (T)a3.size();
            mse_total += (double)mse;

            // ---- backprop (todos los gradientes con pesos viejos) ----
            Vec dA3 = (T(2) / (T)a3.size()) * (a3 - y);
            Vec dZ3 = dA3.array() * a3.array() * (T(1) - a3.array());
            Mat dW3 = dZ3 * a2.transpose();
            Vec db3 = dZ3;

            Vec dA2 = W3.transpose() * dZ3;
            Vec dZ2 = dA2.array() * a2.array() * (T(1) - a2.array());
            Mat dW2 = dZ2 * a1.transpose();
            Vec db2 = dZ2;

            Vec dA1 = W2.transpose() * dZ2;
            Vec dZ1 = dA1.array() * a1.array() * (T(1) - a1.array());
            Mat dW1 = dZ1 * x.transpose();
            Vec db1 = dZ1;

            // ---- actualizacion ----
            W3 -= learning_rate * dW3;  b3 -= learning_rate * db3;
            W2 -= learning_rate * dW2;  b2 -= learning_rate * db2;
            W1 -= learning_rate * dW1;  b1 -= learning_rate * db1;

            // ---- acierto ----
            Eigen::Index pred, real;
            a3.maxCoeff(&pred);
            y.maxCoeff(&real);
            if (pred == real) aciertos++;
        }

        double mse_promedio = mse_total / num_muestras;
        double accuracy = (double)aciertos / num_muestras * 100.0;
        if ((epoch + 1) % op.cada == 0 || epoch == op.epocas - 1) {
            cout << "MSE promedio: " << mse_promedio
                 << " | Accuracy: " << accuracy << "%\n";
        }
    }

    auto end = std::chrono::steady_clock::now();

    Resultado r;
    r.us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    uint64_t h = CHECKSUM_INICIAL;
    h = hash_fp<T>(W1, h); h = hash_fp<T>(b1, h);
    h = hash_fp<T>(W2, h); h = hash_fp<T>(b2, h);
    h = hash_fp<T>(W3, h); h = hash_fp<T>(b3, h);
    r.checksum = h;
    agregar<T>(r.params, W1); agregar<T>(r.params, b1);
    agregar<T>(r.params, W2); agregar<T>(r.params, b2);
    agregar<T>(r.params, W3); agregar<T>(r.params, b3);
    return r;
}

// ============================================================
// MODO fixed (aritmetica entera, golden model del RTL)
// ============================================================
using i64 = int64_t;
using i32 = int32_t;

constexpr int LR_Q  = 6554;    // 0.1   en F16 (~0.100006)
constexpr int K02_Q = 13107;   // 2/10  en F16 (~0.199997)
constexpr int BITS_W = 24;     // ancho total de W y b

// Desplazamiento a la derecha con redondeo al mas cercano
// (suma de 2^(s-1) y desplazamiento aritmetico).
static inline i64 rsh(i64 v, int s) {
    return (v + ((i64)1 << (s - 1))) >> s;
}

// Contadores de diagnostico (para dimensionar el hardware)
struct Estad {
    uint64_t sat_w[3] = {0, 0, 0};
    uint64_t sat_acc = 0, sat_z = 0, sat_dz = 0, sat_dzp = 0;
    uint64_t upd_total[3] = {0, 0, 0};
    uint64_t upd_perdidas[3] = {0, 0, 0};   // p != 0 pero delta == 0
    i64 max_acc_fwd = 0, max_acc_bwd = 0;
};

// Satura v a "bits" bits con signo y cuenta si hubo saturacion
static inline i64 sat(i64 v, int bits, uint64_t& cnt) {
    const i64 hi = ((i64)1 << (bits - 1)) - 1;
    const i64 lo = -((i64)1 << (bits - 1));
    if (v > hi) { cnt++; return hi; }
    if (v < lo) { cnt++; return lo; }
    return v;
}

static int bits_con_signo(i64 m) {
    int b = 1;
    while (m > 0) { b++; m >>= 1; }
    return b;
}

// ---- LUT de la sigmoide: 1025 puntos en [-8,8], paso 1/64, Q0.16 ----
static i32 LUT[1025];

static void inicializar_lut() {
    for (int i = 0; i <= 1024; i++) {
        double v = 65536.0 / (1.0 + std::exp(-(-8.0 + i / 64.0)));
        i64 q = std::llround(v);
        LUT[i] = (i32)std::min<i64>(q, 65535);
    }
}

// z en Q4.12 con signo (16 bits) -> a en Q0.16
static inline i32 lut_sigmoide(i32 z) {
    const unsigned u = (unsigned)(z + 32768);   // 0..65535
    const unsigned i = u >> 6;                  // 10 bits altos: indice
    const unsigned f = u & 63;                  // 6 bits bajos: interpola
    const i32 l0 = LUT[i], l1 = LUT[i + 1];
    return l0 + (((l1 - l0) * (i32)f + 32) >> 6);
}

static int test_lut() {
    inicializar_lut();
    double max_err = 0;
    for (int z = -32768; z <= 32767; z++) {
        double exacto = 1.0 / (1.0 + std::exp(-z / 4096.0));
        double got = lut_sigmoide(z) / 65536.0;
        max_err = std::max(max_err, std::fabs(got - exacto));
    }
    std::printf("LUT sigmoide: error maximo = %.3e (%.2f LSB de Q0.16; 1 LSB = 1.526e-05)\n",
                max_err, max_err * 65536.0);
    return 0;
}

// ---- Capa en punto fijo ----
struct CapaQ {
    int n_out = 0, n_in = 0, fw = 20;
    std::vector<i32> W;   // n_out x n_in, orden FILA
    std::vector<i32> b;   // n_out
};

// ---- Vectores dorados para el testbench del RTL ($readmemh) ----
// Escribe v[0..n) como hex en complemento a 2 de "bits" bits, un valor por
// linea (formato de $readmemh de Verilog).
static bool volcar_hex(const std::string& ruta, const i32* v, size_t n, int bits) {
    std::FILE* f = std::fopen(ruta.c_str(), "w");
    if (!f) { std::cerr << "No se pudo escribir " << ruta << "\n"; return false; }
    const uint32_t mask = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    const int dig = (bits + 3) / 4;
    for (size_t i = 0; i < n; i++)
        std::fprintf(f, "%0*x\n", dig, (unsigned)((uint32_t)v[i] & mask));
    std::fclose(f);
    return true;
}

static CapaQ cuantizar(const MatrixXd& W, const MatrixXd& b, int fw, uint64_t& sat_cnt,
                       double& max_err) {
    CapaQ c;
    c.n_out = (int)W.rows(); c.n_in = (int)W.cols(); c.fw = fw;
    c.W.resize((size_t)c.n_out * c.n_in);
    c.b.resize(c.n_out);
    const double esc = std::ldexp(1.0, fw);
    for (int j = 0; j < c.n_out; j++) {
        for (int k = 0; k < c.n_in; k++) {
            i64 q = sat(std::llround(W(j, k) * esc), BITS_W, sat_cnt);
            c.W[(size_t)j * c.n_in + k] = (i32)q;
            max_err = std::max(max_err, std::fabs((double)q / esc - W(j, k)));
        }
        i64 q = sat(std::llround(b(j, 0) * esc), BITS_W, sat_cnt);
        c.b[j] = (i32)q;
        max_err = std::max(max_err, std::fabs((double)q / esc - b(j, 0)));
    }
    return c;
}

// Pesos y sesgos de las 3 capas (W en orden FILA, igual que el RTL) con
// sufijo "pre" (antes de actualizar) o "post" (despues).
static void volcar_capas(const std::string& pref, const char* suf, const CapaQ* c) {
    for (int l = 0; l < 3; l++) {
        const std::string n = std::to_string(l + 1);
        volcar_hex(pref + "_W" + n + "_" + suf + ".hex", c[l].W.data(), c[l].W.size(), BITS_W);
        volcar_hex(pref + "_b" + n + "_" + suf + ".hex", c[l].b.data(), c[l].b.size(), BITS_W);
    }
}

// MATVEC + SIGMOID: a_out = sigmoide(W*a_in + b)
static void matvec_sigmoide(const CapaQ& c, const i32* a_in, i32* a_out, Estad& st,
                            i32* z_out = nullptr) {
    for (int j = 0; j < c.n_out; j++) {
        i64 acc = (i64)c.b[j] << 16;                   // F = fw+16
        const i32* fila = &c.W[(size_t)j * c.n_in];
        for (int k = 0; k < c.n_in; k++) acc += (i64)fila[k] * a_in[k];
        st.max_acc_fwd = std::max<i64>(st.max_acc_fwd, acc < 0 ? -acc : acc);
        acc = sat(acc, 48, st.sat_acc);
        i64 z = sat(rsh(acc, c.fw + 4), 16, st.sat_z); // a Q4.12
        if (z_out) z_out[j] = (i32)z;                  // solo para volcar vectores
        a_out[j] = lut_sigmoide((i32)z);
    }
}

// MATVEC_T + DERIV: dz_prev = (W^T * dz) .* a_prev(1-a_prev)
// da_out (opcional): dA[k] en Q2.22 tras rsh+sat24, ANTES de multiplicar por
// a(1-a). Solo se usa para volcar vectores dorados (aisla el arbol sumador
// y su redondeo de la etapa DERIV).
static void matvec_t_deriv(const CapaQ& c, const i32* dz, const i32* a_prev,
                           i32* dz_prev, Estad& st, i32* da_out = nullptr) {
    std::vector<i64> acc(c.n_in, 0);
    for (int j = 0; j < c.n_out; j++) {
        const i32* fila = &c.W[(size_t)j * c.n_in];
        for (int k = 0; k < c.n_in; k++) acc[k] += (i64)fila[k] * dz[j];   // F = fw+22
    }
    for (int k = 0; k < c.n_in; k++) {
        st.max_acc_bwd = std::max<i64>(st.max_acc_bwd, acc[k] < 0 ? -acc[k] : acc[k]);
        i64 da = sat(rsh(acc[k], c.fw), 24, st.sat_dz);                    // Q2.22
        if (da_out) da_out[k] = (i32)da;                                   // solo vectores
        i64 s = rsh((i64)a_prev[k] * (65536 - a_prev[k]), 16);             // a(1-a), F16
        dz_prev[k] = (i32)sat(rsh(da * s, 16), 24, st.sat_dz);             // Q2.22
    }
}

static inline uint32_t lfsr_paso(uint32_t s) {
    return (s >> 1) ^ (uint32_t)(-(int32_t)(s & 1u) & 0x80200003u);
}

// UPDATE: W -= lr*dz*a^T ; b -= lr*dz
static void actualizar(CapaQ& c, const i32* dz, const i32* a_in, Redondeo red,
                       std::vector<uint32_t>& lfsr, Estad& st, int idx) {
    const int sh  = 46 - c.fw;    // dZ'(F30) * a(F16) = F46 -> F fw
    const int shb = 30 - c.fw;    // dZ'(F30) -> F fw
    for (int j = 0; j < c.n_out; j++) {
        // dZ' = lr * dZ  (una vez por neurona), F = 30, 27 bits
        i64 dzp = sat(rsh((i64)dz[j] * LR_Q, 8), 27, st.sat_dzp);
        i32* fila = &c.W[(size_t)j * c.n_in];
        uint32_t& s = lfsr[j];
        for (int k = 0; k < c.n_in; k++) {
            i64 p = dzp * a_in[k];                                // F = 46
            i64 R = 0;
            if (red == Redondeo::Cercano) R = (i64)1 << (sh - 1);
            else if (red == Redondeo::Estocastico) {
                s = lfsr_paso(s);
                R = (i64)(s & (((uint32_t)1 << sh) - 1));
            }
            i64 delta = (p + R) >> sh;
            st.upd_total[idx]++;
            if (p != 0 && delta == 0) st.upd_perdidas[idx]++;
            fila[k] = (i32)sat((i64)fila[k] - delta, BITS_W, st.sat_w[idx]);
        }
        i64 Rb = 0;
        if (red == Redondeo::Cercano) Rb = (i64)1 << (shb - 1);
        else if (red == Redondeo::Estocastico) {
            s = lfsr_paso(s);
            Rb = (i64)(s & (((uint32_t)1 << shb) - 1));
        }
        i64 db = (dzp + Rb) >> shb;
        c.b[j] = (i32)sat((i64)c.b[j] - db, BITS_W, st.sat_w[idx]);
    }
}

struct MuestraQ {
    std::vector<i32> x;   // Q0.16
    int etiqueta = 0;
};

static Resultado entrenar_fixed(const Opciones& op, const PesosD& p0,
                                const std::vector<Sample>& muestras) {
    inicializar_lut();
    Estad st;
    uint64_t sat_ini = 0;
    double err_q = 0;

    CapaQ c[3];
    c[0] = cuantizar(p0.W1, p0.b1, op.fw[0], sat_ini, err_q);
    c[1] = cuantizar(p0.W2, p0.b2, op.fw[1], sat_ini, err_q);
    c[2] = cuantizar(p0.W3, p0.b3, op.fw[2], sat_ini, err_q);

    // LFSR por fila (solo se usan en modo estocastico)
    std::vector<uint32_t> lfsr[3];
    for (int l = 0; l < 3; l++) {
        lfsr[l].resize(c[l].n_out);
        for (int j = 0; j < c[l].n_out; j++) {
            uint32_t s = op.semilla ^ (0x9E3779B1u * (uint32_t)(l * 1024 + j + 1));
            lfsr[l][j] = s ? s : 1u;
        }
    }

    // Entradas: pixel uint8 -> Q0.16 = round(pix*65536/255), sat. 65535
    std::vector<MuestraQ> mq;
    for (const Sample& s : muestras) {
        MuestraQ m;
        m.x.resize(N_IN);
        for (int i = 0; i < N_IN; i++) {
            i64 pix = std::llround(s.x(i) * 255.0);
            i64 q = (pix * 131072 + 255) / 510;
            m.x[i] = (i32)std::min<i64>(q, 65535);
        }
        Eigen::Index idx;
        s.y.maxCoeff(&idx);
        m.etiqueta = (int)idx;
        mq.push_back(std::move(m));
    }

    std::printf("# fixed: fw=(%d,%d,%d) redondeo=%s; error de cuantizacion inicial max=%.3e; "
                "saturaciones iniciales=%llu\n",
                op.fw[0], op.fw[1], op.fw[2],
                op.red == Redondeo::Cercano ? "cercano" :
                op.red == Redondeo::Estocastico ? "estocastico" : "piso",
                err_q, (unsigned long long)sat_ini);

    const int num_muestras = (int)mq.size();
    std::vector<i32> a1(N_H1), a2(N_H2), a3(N_OUT);
    std::vector<i32> dz3(N_OUT), dz2(N_H2), dz1(N_H1);
    std::vector<i32> z1(N_H1), z2(N_H2), z3(N_OUT);   // Q4.12, solo para vectores
    std::vector<i32> da2(N_H2), da1(N_H1);            // Q2.22, solo para vectores
    const std::string& pv = op.vectores;
    if (!pv.empty()) {
        volcar_hex(pv + "_lut.hex", LUT, 1025, 16);
        std::printf("# vectores dorados (imagen 0, epoca 0) con prefijo '%s'\n", pv.c_str());
    }
    // Traza de DELTA_OUT: una linea por imagen y epoca (ver formato abajo).
    std::FILE* ftr = pv.empty() ? nullptr : std::fopen((pv + "_delta.hex").c_str(), "w");

    auto start = std::chrono::steady_clock::now();

    for (int epoch = 0; epoch < op.epocas; epoch++) {
        i64 e2_total = 0;     // suma de e^2, F32
        int aciertos = 0;

        for (int i = 0; i < num_muestras; i++) {
            const MuestraQ& m = mq[i];
            const bool cap = !pv.empty() && epoch == 0 && i == 0;   // 1a imagen
            if (cap) {
                volcar_capas(pv, "pre", c);
                volcar_hex(pv + "_x.hex", m.x.data(), N_IN, 16);
            }

            // ---- forward (LOADX ya hecho: m.x en Q0.16) ----
            matvec_sigmoide(c[0], m.x.data(), a1.data(), st, z1.data());
            matvec_sigmoide(c[1], a1.data(), a2.data(), st, z2.data());
            matvec_sigmoide(c[2], a2.data(), a3.data(), st, z3.data());
            if (cap) {
                volcar_hex(pv + "_z1.hex", z1.data(), N_H1, 16);  volcar_hex(pv + "_a1.hex", a1.data(), N_H1, 16);
                volcar_hex(pv + "_z2.hex", z2.data(), N_H2, 16);  volcar_hex(pv + "_a2.hex", a2.data(), N_H2, 16);
                volcar_hex(pv + "_z3.hex", z3.data(), N_OUT, 16); volcar_hex(pv + "_a3.hex", a3.data(), N_OUT, 16);
            }

            // ---- DELTA_OUT: dZ3 = (2/10)(a3-y) a3(1-a3), MSE y acierto ----
            const i64 e2_antes = e2_total;      // para aislar el e^2 de ESTA imagen
            int pred = 0;
            for (int k = 0; k < N_OUT; k++) {
                i64 y = (k == m.etiqueta) ? 65536 : 0;
                i64 e = (i64)a3[k] - y;                                   // F16
                e2_total += e * e;                                        // F32
                i64 s = rsh((i64)a3[k] * (65536 - a3[k]), 16);            // F16
                dz3[k] = (i32)sat(rsh(e * s * K02_Q, 26), 24, st.sat_dz); // Q2.22
                if (a3[k] > a3[pred]) pred = k;                           // 1er maximo
            }
            if (pred == m.etiqueta) aciertos++;
            // Formato de cada linea (hex, primer campo = bits mas altos, 112 digitos):
            //   etiqueta(2) pred(1) e2(9, F32) dz3[0..9](6 c/u, 24 bits) a3[0..9](4 c/u, Q0.16)
            if (ftr) {
                std::fprintf(ftr, "%02x%x%09llx", m.etiqueta, pred,
                             (unsigned long long)(e2_total - e2_antes));
                for (int k = 0; k < N_OUT; k++)
                    std::fprintf(ftr, "%06x", (unsigned)((uint32_t)dz3[k] & 0xFFFFFFu));
                for (int k = 0; k < N_OUT; k++)
                    std::fprintf(ftr, "%04x", (unsigned)a3[k]);
                std::fprintf(ftr, "\n");
            }
            if (cap) {
                volcar_hex(pv + "_dz3.hex", dz3.data(), N_OUT, 24);
                const i32 et = m.etiqueta;
                volcar_hex(pv + "_etiqueta.hex", &et, 1, 8);
            }

            // ---- backprop con los pesos VIEJOS (antes de actualizar) ----
            matvec_t_deriv(c[2], dz3.data(), a2.data(), dz2.data(), st, da2.data());
            matvec_t_deriv(c[1], dz2.data(), a1.data(), dz1.data(), st, da1.data());
            if (cap) {
                volcar_hex(pv + "_dA2.hex", da2.data(), N_H2, 24);   // v3: antes de a(1-a)
                volcar_hex(pv + "_dA1.hex", da1.data(), N_H1, 24);
                volcar_hex(pv + "_dz2.hex", dz2.data(), N_H2, 24);
                volcar_hex(pv + "_dz1.hex", dz1.data(), N_H1, 24);
            }

            // ---- actualizacion: W3, W2, W1 ----
            actualizar(c[2], dz3.data(), a2.data(), op.red, lfsr[2], st, 2);
            actualizar(c[1], dz2.data(), a1.data(), op.red, lfsr[1], st, 1);
            actualizar(c[0], dz1.data(), m.x.data(), op.red, lfsr[0], st, 0);
            if (cap) volcar_capas(pv, "post", c);
        }

        double mse_promedio = (double)e2_total / 4294967296.0 / N_OUT / num_muestras;
        double accuracy = (double)aciertos / num_muestras * 100.0;
        if ((epoch + 1) % op.cada == 0 || epoch == op.epocas - 1) {
            cout << "MSE promedio: " << mse_promedio
                 << " | Accuracy: " << accuracy << "%\n";
        }
    }

    auto end = std::chrono::steady_clock::now();

    Resultado r;
    r.us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    if (ftr) std::fclose(ftr);

    // Hash de enteros (formato canonico del encabezado)
    uint64_t h = CHECKSUM_INICIAL;
    for (int l = 0; l < 3; l++) {
        h = fnv_bytes(c[l].W.data(), c[l].W.size() * sizeof(i32), h);
        h = fnv_bytes(c[l].b.data(), c[l].b.size() * sizeof(i32), h);
    }
    r.checksum = h;

    // Volcado como double en column-major (mismo formato que el .bin)
    for (int l = 0; l < 3; l++) {
        const double inv = std::ldexp(1.0, -c[l].fw);
        for (int k = 0; k < c[l].n_in; k++)
            for (int j = 0; j < c[l].n_out; j++)
                r.params.push_back((double)c[l].W[(size_t)j * c[l].n_in + k] * inv);
        for (int j = 0; j < c[l].n_out; j++)
            r.params.push_back((double)c[l].b[j] * inv);
    }

    // ---- diagnostico para dimensionar el hardware ----
    std::printf("# max|acc| forward = %lld (%d bits con signo; se reservan 48)\n",
                (long long)st.max_acc_fwd, bits_con_signo(st.max_acc_fwd));
    std::printf("# max|acc| backward = %lld (%d bits con signo)\n",
                (long long)st.max_acc_bwd, bits_con_signo(st.max_acc_bwd));
    std::printf("# saturaciones: acc=%llu z=%llu dZ/dA=%llu dZ'=%llu\n",
                (unsigned long long)st.sat_acc, (unsigned long long)st.sat_z,
                (unsigned long long)st.sat_dz, (unsigned long long)st.sat_dzp);
    for (int l = 0; l < 3; l++) {
        i32 mx = 0;
        for (i32 w : c[l].W) mx = std::max<i32>(mx, w < 0 ? -w : w);
        std::printf("# capa %d: W saturados=%llu, max|W|=%.4f, actualizaciones perdidas "
                    "(p!=0, delta=0)=%.2f%%\n",
                    l + 1, (unsigned long long)st.sat_w[l],
                    (double)mx * std::ldexp(1.0, -c[l].fw),
                    st.upd_total[l] ? 100.0 * (double)st.upd_perdidas[l] / (double)st.upd_total[l] : 0.0);
    }
    return r;
}

// ============================================================
// AUTOPRUEBA SINTETICA de MATVEC_T + DERIV  (--test-backward)
//
// El entrenamiento real NUNCA satura dA (max|dA| ~ 3.4e5 contra el limite
// 8388607), asi que sus vectores no ejercitan el saturador ni el peor caso
// del acumulador. Aqui se generan, con un PRNG propio y determinista
// (splitmix64, igual en Mac y Linux), pesos W, gradientes dz y activaciones a
// SINTETICOS, y se pasan por la MISMA funcion matvec_t_deriv() que usa el
// entrenamiento. Se vuelcan, por capa (c3: W3 10x64 ; c2: W2 64x128):
//   PREF_c3_W.hex  PREF_c3_dz.hex  PREF_c3_a.hex   (entradas)
//   PREF_c3_dA.hex PREF_c3_dzp.hex                 (salidas esperadas)
//   y lo mismo con c2.   W,dz,dA,dzp: 24 bits ; a: 16 bits (Q0.16)
// Casos (--caso=):
//   0 realista : W en +-0.5, dz en +-0.06            (sin saturar)
//   1 rango completo : W y dz uniformes en todo el rango de 24 bits (satura mucho)
//   2 extremos : dz = -2^23 en todas las filas, W en {-2^23, 2^23-1};
//                columna 0 = todo -2^23 (acc maximo +n*2^46), columna 1 = todo
//                2^23-1 (acc minimo). Prueba el ancho del acumulador.
//   3 bordes   : columnas que caen justo a ambos lados del umbral de
//                saturacion de dA (+-2^23) y del redondeo de rsh().
// 'a' siempre incluye los bordes 0, 1, 16383..16384, 32767..32769, 65534, 65535.
// Con --semilla=N se generan juegos distintos del mismo caso.
// ============================================================
struct Sm64 {
    uint64_t s;
    uint64_t next() {
        uint64_t z = (s += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    i64 rango(i64 lo, i64 hi) { return lo + (i64)(next() % (uint64_t)(hi - lo + 1)); }
};

static int test_backward(const Opciones& op) {
    if (op.vectores.empty()) { std::cerr << "--test-backward requiere --vectores=PREF\n"; return 1; }
    if (op.caso < 0 || op.caso > 3) { std::cerr << "--caso debe estar entre 0 y 3\n"; return 1; }
    for (int l = 0; l < 3; l++)
        if (op.fw[l] < 8 || op.fw[l] > 22) { std::cerr << "fw debe estar entre 8 y 22\n"; return 1; }

    const i64 WMAX = ((i64)1 << 23) - 1, WMIN = -((i64)1 << 23);
    const int a_borde[12] = {0, 1, 2, 16383, 16384, 32767, 32768, 32769, 49151, 49152, 65534, 65535};
    // lista de bordes del caso 3 (ver comentario de cabecera)
    const i64 w0_borde[6] = {(1 << 21) - 2, (1 << 21) - 1, (1 << 21),
                             -(1 << 21) + 1, -(1 << 21), -(1 << 21) - 1};
    const i64 w1_borde[10] = {0, (1 << 19) - 1, (1 << 19), -(1 << 19), -(1 << 19) - 1,
                              (3 << 20), (3 << 20) + (1 << 19) - 1, (3 << 20) + (1 << 19),
                              -(3 << 20) - (1 << 19), -(3 << 20) - (1 << 19) - 1};

    const int capa_n[2]   = {3, 2};
    const int n_out_l[2]  = {N_OUT, N_H2};   // c3: 10 x 64 ; c2: 64 x 128
    const int n_in_l[2]   = {N_H2, N_H1};
    Estad st;

    for (int t = 0; t < 2; t++) {
        const int n_out = n_out_l[t], n_in = n_in_l[t];
        Sm64 rng{(uint64_t)op.semilla * 1000003ULL + (uint64_t)op.caso * 7919ULL +
                 (uint64_t)t * 104729ULL + 12345ULL};
        CapaQ c;
        c.n_out = n_out; c.n_in = n_in; c.fw = op.fw[capa_n[t] - 1];
        c.W.assign((size_t)n_out * n_in, 0);
        c.b.assign(n_out, 0);
        std::vector<i32> dz(n_out), a(n_in), da(n_in), dzp(n_in);
        auto W = [&](int j, int k) -> i32& { return c.W[(size_t)j * n_in + k]; };
        auto ext = [&]() -> i32 { return (i32)((rng.next() & 1) ? WMAX : WMIN); };

        for (int k = 0; k < n_in; k++)
            a[k] = (i32)((op.caso == 3 || k < 12) ? a_borde[k % 12] : rng.rango(0, 65535));

        switch (op.caso) {
        case 0:
            for (int j = 0; j < n_out; j++) {
                dz[j] = (i32)rng.rango(-(1 << 18), (1 << 18) - 1);
                for (int k = 0; k < n_in; k++) W(j, k) = (i32)rng.rango(-(1 << 20), (1 << 20) - 1);
            }
            break;
        case 1:
            for (int j = 0; j < n_out; j++) {
                dz[j] = (i32)rng.rango(WMIN, WMAX);
                for (int k = 0; k < n_in; k++) W(j, k) = (i32)rng.rango(WMIN, WMAX);
            }
            break;
        case 2:
            for (int j = 0; j < n_out; j++) {
                dz[j] = (i32)WMIN;
                for (int k = 0; k < n_in; k++) W(j, k) = ext();
                W(j, 0) = (i32)WMIN;       // producto +2^46 en todas las filas
                W(j, 1) = (i32)WMAX;       // producto -(2^23)(2^23-1) en todas
            }
            break;
        default:   // caso 3: acc = W[0][k]*2^22 + W[1][k]*1  -> dA = W0*4 + rsh(W1,20)
            for (int j = 0; j < n_out; j++) {
                dz[j] = (j == 0) ? (1 << 22) : (j == 1 ? 1 : 0);   // filas >= 2 aportan 0
                for (int k = 0; k < n_in; k++) W(j, k) = (i32)rng.rango(WMIN, WMAX);
            }
            for (int k = 0; k < 60 && k < n_in; k++) {            // 6 x 10 combinaciones
                W(0, k) = (i32)w0_borde[k / 10];
                W(1, k) = (i32)w1_borde[k % 10];
            }
            break;
        }

        matvec_t_deriv(c, dz.data(), a.data(), dzp.data(), st, da.data());

        // ---- cobertura: recalculo del acumulador y del umbral de saturacion ----
        int n_pos = 0, n_neg = 0, n_borde = 0;
        i64 max_acc = 0;
        for (int k = 0; k < n_in; k++) {
            i64 acc = 0;
            for (int j = 0; j < n_out; j++) acc += (i64)W(j, k) * dz[j];
            max_acc = std::max<i64>(max_acc, acc < 0 ? -acc : acc);
            const i64 d = rsh(acc, c.fw);
            if (d > WMAX) n_pos++; else if (d < WMIN) n_neg++;
            if (d == WMAX || d == WMIN) n_borde++;
        }
        const std::string n = std::to_string(capa_n[t]);
        const std::string p = op.vectores + "_c" + n;
        volcar_hex(p + "_W.hex",   c.W.data(), c.W.size(), 24);
        volcar_hex(p + "_dz.hex",  dz.data(),  dz.size(),  24);
        volcar_hex(p + "_a.hex",   a.data(),   a.size(),   16);
        volcar_hex(p + "_dA.hex",  da.data(),  da.size(),  24);
        volcar_hex(p + "_dzp.hex", dzp.data(), dzp.size(), 24);
        std::printf("# caso %d semilla %u  c%s (%dx%d, fw=%d): dA satura +:%d -:%d  en borde exacto:%d  "
                    "max|acc| = %d bits con signo\n",
                    op.caso, op.semilla, n.c_str(), n_out, n_in, c.fw, n_pos, n_neg, n_borde,
                    bits_con_signo(max_acc));
    }
    return 0;
}

// ============================================================
// main
// ============================================================
int main(int argc, char** argv) {
    Opciones op;
    if (argc < 2 || !parsear(argc, argv, op)) { uso(); return 1; }

    if (op.test_lut) return test_lut();
    if (!op.cmp_a.empty()) return comparar(op.cmp_a, op.cmp_b);
    if (op.test_bw) return test_backward(op);

    if (op.modo != "double" && op.modo != "float" && op.modo != "fixed") { uso(); return 1; }
    for (int l = 0; l < 3; l++) {
        if (op.fw[l] < 8 || op.fw[l] > 22) {
            std::cerr << "fw debe estar entre 8 y 22\n";
            return 1;
        }
    }
    if (op.n < 1 || op.n > 60000) { std::cerr << "--n debe estar entre 1 y 60000\n"; return 1; }

    PesosD p0;
    if (!cargar_pesos(op.pesos, p0)) {
        std::cerr << "No se pudo leer " << op.pesos << " (esperado: 875088 bytes)\n";
        return 1;
    }

    std::ifstream image_file(op.datos + "/train-images-idx3-ubyte", std::ios::binary);
    std::ifstream label_file(op.datos + "/train-labels-idx1-ubyte", std::ios::binary);
    if (!image_file || !label_file) {
        std::cerr << "No se pudieron abrir los archivos MNIST en " << op.datos << "\n";
        return 1;
    }
    std::vector<Sample> muestras;
    for (int i = 0; i < op.n; i++) muestras.push_back(load_sample(image_file, label_file, i));

    cout << "# modo=" << op.modo << " epocas=" << op.epocas << " n=" << op.n << "\n";

    Resultado r;
    if (op.modo == "double")      r = entrenar_fp<double>(op, p0, muestras);
    else if (op.modo == "float")  r = entrenar_fp<float>(op, p0, muestras);
    else                          r = entrenar_fixed(op, p0, muestras);

    cout << "\nChecksum parametros: " << std::hex << r.checksum << std::dec << "\n";
    cout << "Tiempo total: " << r.us << " us\n";

    if (!op.volcar.empty()) {
        std::ofstream f(op.volcar, std::ios::binary | std::ios::trunc);
        f.write(reinterpret_cast<const char*>(r.params.data()),
                (std::streamsize)(r.params.size() * sizeof(double)));
        if (!f) { std::cerr << "No se pudo escribir " << op.volcar << "\n"; return 1; }
    }
    return 0;
}