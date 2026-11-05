// ============================================================
// exportar_pesos.cpp
//
// Genera los pesos iniciales de la MLP 784-128-64-10 con las MISMAS
// llamadas y en el MISMO orden que main.cpp, y los vuelca a un
// archivo binario para que el emulador de punto fijo y la FPGA
// partan exactamente de los mismos valores.
//
// IMPORTANTE: si se cambia la inicializacion en main.cpp, hay que
// cambiarla igual aqui (el orden de las llamadas a Random importa,
// porque Eigen usa std::rand y cada llamada avanza la secuencia).
//
// Uso:
//   ./exportar_pesos [archivo_salida]     (def. pesos_iniciales.bin)
//
// Formato del archivo (sin cabecera):
//   W1(128x784) b1(128) W2(64x128) b2(64) W3(10x64) b3(10)
//   - double IEEE 754 de 64 bits, little-endian (el de la maquina)
//   - cada matriz en orden de memoria de Eigen: POR COLUMNAS
//     (column-major), es decir W(i,j) esta en el indice i + j*filas.
//   - total: 109386 doubles = 875088 bytes
// ============================================================

#include <Eigen/Dense>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>

#include "checksum.h"

using Eigen::MatrixXd;

// Cantidad esperada de valores (para validar el archivo escrito)
constexpr size_t TOTAL_DOUBLES =
    128 * 784 + 128 + 64 * 128 + 64 + 10 * 64 + 10;

// Escribe los datos crudos de una matriz y devuelve false si falla
static bool escribir(std::ofstream& f, const MatrixXd& m) {
    f.write(reinterpret_cast<const char*>(m.data()),
            static_cast<std::streamsize>(m.size() * sizeof(double)));
    return static_cast<bool>(f);
}

// Lee los datos crudos hacia una matriz ya dimensionada
static bool leer(std::ifstream& f, MatrixXd& m) {
    f.read(reinterpret_cast<char*>(m.data()),
           static_cast<std::streamsize>(m.size() * sizeof(double)));
    return static_cast<bool>(f);
}

int main(int argc, char** argv) {

    const char* salida = (argc > 1) ? argv[1] : "pesos_iniciales.bin";

    // -------------------------------------------------------
    // Misma inicializacion que main.cpp (mismo orden)
    // -------------------------------------------------------
    MatrixXd W1 = MatrixXd::Random(128, 784) * 0.1;
    MatrixXd b1 = MatrixXd::Zero(128, 1);

    MatrixXd W2 = MatrixXd::Random(64, 128) * 0.1;
    MatrixXd b2 = MatrixXd::Zero(64, 1);

    MatrixXd W3 = MatrixXd::Random(10, 64) * 0.1;
    MatrixXd b3 = MatrixXd::Zero(10, 1);

    // -------------------------------------------------------
    // Escritura
    // -------------------------------------------------------
    {
        std::ofstream f(salida, std::ios::binary | std::ios::trunc);
        if (!f) {
            std::cerr << "No se pudo crear " << salida << "\n";
            return 1;
        }
        if (!(escribir(f, W1) && escribir(f, b1) &&
              escribir(f, W2) && escribir(f, b2) &&
              escribir(f, W3) && escribir(f, b3))) {
            std::cerr << "Error al escribir " << salida << "\n";
            return 1;
        }
    }

    // -------------------------------------------------------
    // Verificacion: releer el archivo y comparar bit a bit
    // -------------------------------------------------------
    MatrixXd R1(128, 784), rb1(128, 1), R2(64, 128), rb2(64, 1),
             R3(10, 64), rb3(10, 1);

    std::ifstream in(salida, std::ios::binary | std::ios::ate);
    if (!in) {
        std::cerr << "No se pudo releer " << salida << "\n";
        return 1;
    }
    const std::streamsize bytes = in.tellg();
    if (bytes != static_cast<std::streamsize>(TOTAL_DOUBLES * sizeof(double))) {
        std::cerr << "Tamano inesperado: " << bytes << " bytes\n";
        return 1;
    }
    in.seekg(0, std::ios::beg);

    if (!(leer(in, R1) && leer(in, rb1) && leer(in, R2) &&
          leer(in, rb2) && leer(in, R3) && leer(in, rb3))) {
        std::cerr << "Error al releer " << salida << "\n";
        return 1;
    }

    // Hash FNV-1a de checksum.h, mismo orden que main.cpp
    auto hash_de = [](const MatrixXd& a1, const MatrixXd& c1,
                      const MatrixXd& a2, const MatrixXd& c2,
                      const MatrixXd& a3, const MatrixXd& c3) {
        uint64_t h = CHECKSUM_INICIAL;
        h = hash_params(a1, h); h = hash_params(c1, h);
        h = hash_params(a2, h); h = hash_params(c2, h);
        h = hash_params(a3, h); h = hash_params(c3, h);
        return h;
    };

    const uint64_t h_mem  = hash_de(W1, b1, W2, b2, W3, b3);
    const uint64_t h_arch = hash_de(R1, rb1, R2, rb2, R3, rb3);

    std::printf("Archivo: %s (%lld bytes)\n", salida,
                static_cast<long long>(bytes));
    std::printf("Checksum pesos iniciales (memoria): %llx\n",
                static_cast<unsigned long long>(h_mem));
    std::printf("Checksum pesos iniciales (archivo): %llx\n",
                static_cast<unsigned long long>(h_arch));

    if (h_mem != h_arch) {
        std::cerr << "ERROR: el archivo no coincide con la memoria\n";
        return 1;
    }

    // Referencia rapida de los rangos (util para elegir formatos Q)
    std::printf("max|W1| = %g, max|W2| = %g, max|W3| = %g\n",
                W1.cwiseAbs().maxCoeff(), W2.cwiseAbs().maxCoeff(),
                W3.cwiseAbs().maxCoeff());
    std::printf("OK\n");
    return 0;
}