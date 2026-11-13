#ifndef ENTRENAMIENTO_H
#define ENTRENAMIENTO_H

#include <Eigen/Dense>
#include <vector>
#include "mnist.h"

// Agrupa los pesos (W) y sesgos (b) de las 3 capas
struct Red {
    Eigen::MatrixXd W1, W2, W3;
    Eigen::VectorXd b1, b2, b3;
};

// Crea la red con sus valores iniciales
Red crear_red();

// Entrena "epochs" epocas sobre "muestras" e imprime MSE y accuracy por
// epoca. Si conteo_clases no es nulo, cuenta las clases en la epoca 0.
void entrenar(Red& red, const std::vector<Sample>& muestras, int epochs, 
    double learning_rate, int conteo_clases[10]);

#endif
