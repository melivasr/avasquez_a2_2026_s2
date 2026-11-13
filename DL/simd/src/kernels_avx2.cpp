#include <immintrin.h>
#include "kernels_avx2.h"

// Equivale a "z = 0; z += w * s" salvo por el signo de cero
// (0 + (-0) = +0), y asi coincide exactamente con el primer termino de
// la suma de Eigen en el secuencial.
void columna_por_escalar(const double* w, double s, double* z, size_t n) {
    const __m256d vs = _mm256_set1_pd(s);  // PREPARACION: s en los 4 carriles
    const size_t limite = n - (n % 4);     // último múltiplo de 4 que cabe en n

    for (size_t i = 0; i < limite; i += 4) {
        __m256d vw = _mm256_loadu_pd(w + i);
        __m256d vz = _mm256_mul_pd(vw, vs);  // OPERACION
        _mm256_storeu_pd(z + i, vz); // GUARDADO
    }

    for (size_t i = limite; i < n; i++) { // filas sobrantes (W3)
        z[i] = w[i] * s;
    }
}

void acumular_columna(const double* w, double s, double* z, size_t n) {
    const __m256d vs = _mm256_set1_pd(s);
    const size_t limite = n - (n % 4);

    for (size_t i = 0; i < limite; i += 4) {
        __m256d vw = _mm256_loadu_pd(w + i);
        __m256d vz = _mm256_loadu_pd(z + i);
        vz = _mm256_add_pd(vz, _mm256_mul_pd(vw, vs)); // mul y add separados
        _mm256_storeu_pd(z + i, vz);
    }

    for (size_t i = limite; i < n; i++) {
        z[i] += w[i] * s;
    }
}

void actualizar_columna(double* w, const double* dz, double s,
                        double lr, size_t n) {
    const __m256d vs = _mm256_set1_pd(s);
    const __m256d vlr = _mm256_set1_pd(lr);
    const size_t limite = n - (n % 4);

    for (size_t i = 0; i < limite; i += 4) {
        __m256d vw = _mm256_loadu_pd(w + i);
        __m256d vdz = _mm256_loadu_pd(dz + i);
        __m256d paso = _mm256_mul_pd(vlr, _mm256_mul_pd(vdz, vs));
        _mm256_storeu_pd(w + i, _mm256_sub_pd(vw, paso));
    }

    for (size_t i = limite; i < n; i++) {
        w[i] -= lr * (dz[i] * s);
    }
}

// Orden de suma de W.col(j).dot(dZ) 
// Un registro AVX2 de 4 carriles es exactamente [acc0 | acc1]. Despues
// Eigen suma acc0 + acc1, agrega un ultimo paquete de 2 si sobra y hace
// la suma horizontal carril0 + carril1.
double producto_punto(const double* w, const double* dz, size_t n) {
    const size_t limite = n - (n % 4);

    __m256d acc = _mm256_mul_pd(_mm256_loadu_pd(w), _mm256_loadu_pd(dz));
    for (size_t i = 4; i < limite; i += 4) {
        __m256d vw = _mm256_loadu_pd(w + i);
        __m256d vdz = _mm256_loadu_pd(dz + i);
        acc = _mm256_add_pd(acc, _mm256_mul_pd(vw, vdz));
    }

    // Suma horizontal: [a0 a1 | a2 a3] -> (a0+a2, a1+a3)
    __m128d par = _mm_add_pd(_mm256_castpd256_pd128(acc),_mm256_extractf128_pd(acc, 1));

    size_t i = limite;
    if (n - i >= 2) { // paquete de 2 sobrante (W3: 8, 9)
        par = _mm_add_pd(par, _mm_mul_pd(_mm_loadu_pd(w + i), _mm_loadu_pd(dz + i)));
        i += 2;
    }

    // (a0+a2) + (a1+a3)
    double resultado = _mm_cvtsd_f64(_mm_add_sd(par, _mm_unpackhi_pd(par, par)));

    for (; i < n; i++) { // elemento impar sobrante
        resultado += w[i] * dz[i];
    }
    return resultado;
}
