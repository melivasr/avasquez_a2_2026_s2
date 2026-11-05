import sys
import numpy as np
from PIL import Image, ImageOps

# =========================================
# CARGAR PESOS Y SESGOS
# =========================================

W1 = np.loadtxt("../data/params/W1.txt")
b1 = np.loadtxt("../data/params/b1.txt").reshape(-1)

W2 = np.loadtxt("../data/params/W2.txt")
b2 = np.loadtxt("../data/params/b2.txt").reshape(-1)

W3 = np.loadtxt("../data/params/W3.txt")
b3 = np.loadtxt("../data/params/b3.txt").reshape(-1)


# =========================================
# FUNCION DE ACTIVACION
# =========================================

def sigmoid(z):
    z = np.clip(z, -500, 500)
    return 1.0 / (1.0 + np.exp(-z))


# =========================================
# PROCESAMIENTO DE IMAGEN
# =========================================

def cargar_imagen(ruta, test):

    imagen = Image.open(ruta).convert("L")

    # Activar si la imagen tiene fondo blanco
    # y el numero esta escrito en negro.
    if test==1: 
        imagen = ImageOps.invert(imagen)

    imagen = imagen.resize((28, 28))

    x = np.asarray(imagen, dtype=np.float64)

    # Normalizar entre 0 y 1
    x = x.reshape(784) / 255.0

    return x


# =========================================
# FORWARD PASS
# =========================================

def predecir(x):

    z1 = W1 @ x + b1
    a1 = sigmoid(z1)

    z2 = W2 @ a1 + b2
    a2 = sigmoid(z2)

    z3 = W3 @ a2 + b3
    a3 = sigmoid(z3)

    prediccion = np.argmax(a3)

    return prediccion, a3


# =========================================
# PROGRAMA PRINCIPAL
# =========================================

test = 0

if len(sys.argv) < 2:
    print("Uso: python predict.py imagen.png")
    sys.exit(1)

x = cargar_imagen(sys.argv[1], test)

resultado, activaciones = predecir(x)

print("\nResultado:", resultado)
print("\nActivaciones de salida:")

for i, valor in enumerate(activaciones):
    print(f"Numero {i}: {valor:.4f}")