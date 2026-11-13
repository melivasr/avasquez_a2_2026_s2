#include <iostream>
#include "entrenamiento.h"
#include "capas.h"
#include "kernels_avx2.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Crea la red 784-128-64-10. Hay 3 matrices de pesos, una por cada
// conexion entre capas: W1 (784 pixeles -> 128), W2 (128 -> 64) y
// W3 (64 -> 10, una salida por digito). 
Red crear_red() {
    Red red;
    red.W1 = MatrixXd::Random(128, 784) * 0.1;
    red.b1 = VectorXd::Zero(128);
    red.W2 = MatrixXd::Random(64, 128) * 0.1;
    red.b2 = VectorXd::Zero(64);
    red.W3 = MatrixXd::Random(10, 64) * 0.1;
    red.b3 = VectorXd::Zero(10);
    return red;
}

// Entrena con SGD (descenso de gradiente estocastico)
// los pesos se ajustan despues de cada imagen
// Por imagen: forward, salida, backprop W3/W2 y actualizacion de W1.

void entrenar(Red& red, const std::vector<Sample>& muestras, int epochs,
    double learning_rate, int conteo_clases[10]) {

    const int num_muestras = static_cast<int>(muestras.size());

    // Estado intermedio una sola vez, fuera del ciclo.
    VectorXd a1 = VectorXd::Zero(128);
    VectorXd a2 = VectorXd::Zero(64);
    VectorXd a3 = VectorXd::Zero(10);
    VectorXd dZ1 = VectorXd::Zero(128);
    VectorXd dZ2 = VectorXd::Zero(64);
    VectorXd dZ3 = VectorXd::Zero(10);
    std::vector<double> z(128);  // acumulador del forward (capa mas grande)

    for (int epoch = 0; epoch < epochs; epoch++) {
        double mse_total = 0.0;
        int aciertos = 0;
        Eigen::Index clase_real;
        Eigen::Index clase_predicha;

        for (int s = 0; s < num_muestras; s++) {
            const Sample& sample = muestras[s];
            const double* x = sample.x.data();

            //  Forward (vectorial por columnas)
            forward_capa(red.W1, x, red.b1, a1, z.data());
            forward_capa(red.W2, a1.data(), red.b2, a2, z.data());
            forward_capa(red.W3, a2.data(), red.b3, a3, z.data());

            //  MSE y delta de salida (escalar)
            mse_total += (a3 - sample.y).squaredNorm() / 10.0;
            for (int i = 0; i < 10; i++) {
                double dA3 = (2.0 / 10.0) * (a3(i) - sample.y(i));
                dZ3(i) = dA3 * a3(i) * (1.0 - a3(i));
                red.b3(i) -= learning_rate * dZ3(i);
            }

            // Backpropagation W3 -> dZ2, luego W2 -> dZ1
            backprop_capa(red.W3, dZ3, a2, dZ2, red.b2, learning_rate);
            backprop_capa(red.W2, dZ2, a1, dZ1, red.b1, learning_rate);

            // Actualizar W1 por columnas 
            for (Eigen::Index j = 0; j < red.W1.cols(); j++) {
                actualizar_columna(red.W1.col(j).data(), 
                dZ1.data(), x[j], learning_rate, 128);
            }

            //  Accuracy y distribución de clases 
            if (epoch == 0 && conteo_clases) {
                Eigen::Index clase;
                sample.y.maxCoeff(&clase);
                conteo_clases[clase]++;
            }

            a3.maxCoeff(&clase_predicha);
            sample.y.maxCoeff(&clase_real);
            if (clase_predicha == clase_real) {
                aciertos++;
            }
        }

        //  Métricas de la época 
        double mse_promedio = mse_total / num_muestras;
        double accuracy = (double) aciertos / num_muestras * 100.0;
        std::cout << "MSE promedio: " << mse_promedio
        << " | Accuracy: " << accuracy << "%\n";
    }
}
