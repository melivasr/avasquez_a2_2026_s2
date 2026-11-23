#ifndef KERNELS_AVX2_H
#define KERNELS_AVX2_H

#include <cstddef>

// z[i] = w[i] * s. Inicializa el acumulador del forward con la columna 0.
void columna_por_escalar(const double* w, double s, double* z, size_t n);

// z[i] += w[i] * s. Paso del forward por columnas: z += W.col(j) * x[j].
void acumular_columna(const double* w, double s, double* z, size_t n);

// w[i] -= lr * (dz[i] * s). Actualizacion SGD: W.col(j) -= lr * dZ * a[j].
void actualizar_columna(double* w, const double* dz, double s,
                        double lr, size_t n);

// sum(w[i] * dz[i]) en el mismo orden de suma que Eigen (requiere n >= 4).
double producto_punto(const double* w, const double* dz, size_t n);

#endif
