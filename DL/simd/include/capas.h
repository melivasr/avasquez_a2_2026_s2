#ifndef CAPAS_H
#define CAPAS_H

#include <Eigen/Dense>

double sigmoid(double z);

// a = sigmoid(W * x + b), recorriendo W por columnas.
// z es un buffer de W.rows() .
void forward_capa(const Eigen::MatrixXd& W, const double* x, const Eigen::VectorXd& b, 
    Eigen::VectorXd& a, double* z);
// Backprop de una capa y actualizacion de W, columna por columna:
// dA = W.col(j) . dZ (antes de modificar la columna j)
// dZ_ant[j] = dA * a[j] * (1 - a[j])
// W.col(j) -= lr * dZ * a[j]
// b_ant[j] -= lr * dZ_ant[j]
void backprop_capa(Eigen::MatrixXd& W, const Eigen::VectorXd& dZ, const Eigen::VectorXd& a, 
    Eigen::VectorXd& dZ_ant, Eigen::VectorXd& b_ant, double lr);

#endif
