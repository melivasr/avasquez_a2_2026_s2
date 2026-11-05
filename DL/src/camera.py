import cv2
import numpy as np

# =========================================
# CARGAR PARAMETROS
# =========================================

W1 = np.loadtxt("../data/params/W1.txt")
b1 = np.loadtxt("../data/params/b1.txt").reshape(-1)

W2 = np.loadtxt("../data/params/W2.txt")
b2 = np.loadtxt("../data/params/b2.txt").reshape(-1)

W3 = np.loadtxt("../data/params/W3.txt")
b3 = np.loadtxt("../data/params/b3.txt").reshape(-1)


# =========================================
# RED NEURONAL
# =========================================

def sigmoid(z):
    z = np.clip(z, -500, 500)
    return 1.0 / (1.0 + np.exp(-z))


def predecir(x):
    a1 = sigmoid(W1 @ x + b1)
    a2 = sigmoid(W2 @ a1 + b2)
    a3 = sigmoid(W3 @ a2 + b3)

    return np.argmax(a3)


# =========================================
# PROCESAR IMAGEN DE CAMARA
# =========================================

def procesar_imagen(imagen):

    gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

    # Convertir trazos oscuros en blancos
    _, binaria = cv2.threshold(
        gris, 0, 255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    # Encontrar el numero
    puntos = cv2.findNonZero(binaria)

    if puntos is None:
        return None

    x, y, w, h = cv2.boundingRect(puntos)

    numero = binaria[y:y+h, x:x+w]

    # Mantener proporciones y ajustar a 20x20
    escala = 20.0 / max(w, h)

    nuevo_w = max(1, int(w * escala))
    nuevo_h = max(1, int(h * escala))

    numero = cv2.resize(numero, (nuevo_w, nuevo_h))

    # Centrar en una imagen de 28x28
    resultado = np.zeros((28, 28), dtype=np.uint8)

    inicio_x = (28 - nuevo_w) // 2
    inicio_y = (28 - nuevo_h) // 2

    resultado[
        inicio_y:inicio_y+nuevo_h,
        inicio_x:inicio_x+nuevo_w
    ] = numero

    return resultado.reshape(784).astype(np.float64) / 255.0


# =========================================
# INICIAR CAMARA
# =========================================

camara = cv2.VideoCapture(0)

if not camara.isOpened():
    print("No se pudo abrir la camara")
    exit()

print("Camara iniciada")
print("Coloca un numero dentro del recuadro.")
print("Presiona ESPACIO para reconocer.")
print("Presiona Q para salir.")

while True:

    ret, frame = camara.read()

    if not ret:
        break

    frame = cv2.flip(frame, 1)

    alto, ancho = frame.shape[:2]

    # Region donde se escribe el numero
    x1 = ancho // 2 - 100
    y1 = alto // 2 - 100
    x2 = ancho // 2 + 100
    y2 = alto // 2 + 100

    cv2.rectangle(
        frame,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2
    )

    cv2.putText(
        frame,
        "Coloca un numero y presiona ESPACIO",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2
    )

    cv2.imshow("Reconocimiento MLP", frame)

    tecla = cv2.waitKey(1) & 0xFF

    if tecla == ord('q'):
        break

    elif tecla == 32:

        roi = frame[y1:y2, x1:x2]

        entrada = procesar_imagen(roi)
        
        if entrada is not None:

            imagen_procesada = (entrada.reshape(28, 28) * 255).astype(np.uint8)

            cv2.imshow("Entrada de la MLP", imagen_procesada)

        if entrada is not None:

            resultado = predecir(entrada)

            print("Numero reconocido:", resultado)

            cv2.putText(
                frame,
                f"Prediccion: {resultado}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.imshow("Reconocimiento MLP", frame)
            cv2.waitKey(1000)

camara.release()
cv2.destroyAllWindows()