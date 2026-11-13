#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>
#include "checksum.h"
#include "runtime.h"
#include "mnist.h"
#include "entrenamiento.h"

using std::cout;

// Configura, carga MNIST, mide entrenar() e imprime checksum y tiempo.
int run(int argc, char** argv) {
    dl::initialize();

    // Configuracion de entrenamiento: learning rate, numero de muestras y epocas

    const double learning_rate = 0.1;
    const int num_muestras = dl::env_int("DL_SAMPLES", 1000);
    const int epochs = (argc > 1) ? dl::integer(argv[1]) : 1000;
    int conteo_clases[10] = {0};

    Red red = crear_red();

    // Lectura de MNIST (imagenes y etiquetas) y carga de muestras

    std::ifstream image_file("../data/mnist/train-images-idx3-ubyte", std::ios::binary);

    std::ifstream label_file("../data/mnist/train-labels-idx1-ubyte", std::ios::binary);

    if (!image_file || !label_file) {
        std::cerr << "No se pudieron abrir los archivos MNIST\n";
        return 1;
    }

    std::vector<Sample> muestras;

    for (int i = 0; i < num_muestras; i++){
        muestras.push_back(load_sample(image_file, label_file, i));
    }

    // Entrenamiento y medición de tiempo

    auto start = std::chrono::steady_clock::now();
    entrenar(red, muestras, epochs, learning_rate, conteo_clases);
    auto end = std::chrono::steady_clock::now();

    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Resultados: checksum de los parámetros y distribución de clases

    dl::finish(red.W1, red.b1, red.W2, red.b2, red.W3, red.b3);

    uint64_t h = CHECKSUM_INICIAL;

    h = hash_params(red.W1, h); 
    h = hash_params(red.b1, h);
    h = hash_params(red.W2, h); 
    h = hash_params(red.b2, h);
    h = hash_params(red.W3, h); 
    h = hash_params(red.b3, h);

    cout << "\nChecksum parametros: " << std::hex << h << std::dec << "\n";

    if (!dl::quiet()) {
        cout << "\nDistribución de clases:\n";
        for (int i = 0; i < 10; i++) {
            cout << "Clase " << i << ": " << conteo_clases[i] << "\n";
        }
    }
    
    cout << "\nSIMD (AVX2)\n";
    cout << "\nTiempo total: " << elapsed.count() << " us\n";

    return 0;
}

int main(int argc, char** argv) {
    try { return run(argc, argv); }
    catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
