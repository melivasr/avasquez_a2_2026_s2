//Implementacion de forward_capa y backprop_capa
#include <cmath>
#include <cstddef>
#include "capas.h"
#include "kernels_avx2.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double sigmoid(double z) {
    return 1.0 / (1.0 + std::exp(-z));
}

void forward_capa(const MatrixXd& W, const double* x, 
    const VectorXd& b, VectorXd& a, double* z) {
    const size_t filas = static_cast<size_t>(W.rows());
    const Eigen::Index columnas = W.cols();

    // Vectorial: z = sum_j W.col(j) * x[j], en el orden j = 0..n-1
    columna_por_escalar(W.col(0).data(), x[0], z, filas);
    for (Eigen::Index j = 1; j < columnas; j++) {
        acumular_columna(W.col(j).data(), x[j], z, filas);
    }

    // Escalar: activacion
    for (size_t i = 0; i < filas; i++) {
        a(i) = sigmoid(z[i] + b(i));
    }
}

void backprop_capa(MatrixXd& W, const VectorXd& dZ, const VectorXd& a, 
    VectorXd& dZ_ant, VectorXd& b_ant, double lr) {
    const size_t filas = static_cast<size_t>(W.rows());

    for (Eigen::Index j = 0; j < W.cols(); j++) {
        double* columna = W.col(j).data();
        double dA = producto_punto(columna, dZ.data(), filas);  // vectorial
        dZ_ant(j) = dA * a(j) * (1.0 - a(j)); // escalar
        actualizar_columna(columna, dZ.data(), a(j), lr, filas); // vectorial
        b_ant(j) -= lr * dZ_ant(j); // escalar
    }
}
