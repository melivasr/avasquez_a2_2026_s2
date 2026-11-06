# Uso: python3 evaluar_bin.py REF.bin otro1.bin otro2.bin ...
import sys, struct
import numpy as np

RUTA = "../data/mnist/"
FORMA = [(128, 784), (128, 1), (64, 128), (64, 1), (10, 64), (10, 1)]

def cargar_bin(ruta):
    v = np.fromfile(ruta, dtype="<f8")
    if v.size != 109386:
        sys.exit(f"{ruta}: tamano inesperado {v.size}")
    out, o = [], 0
    for f, c in FORMA:
        # column-major: W(i,j) esta en el indice i + j*filas
        out.append(v[o:o + f*c].reshape((f, c), order="F"))
        o += f*c
    W1, b1, W2, b2, W3, b3 = out
    return W1, b1.ravel(), W2, b2.ravel(), W3, b3.ravel()

def sig(z): return 1.0 / (1.0 + np.exp(-np.clip(z, -500, 500)))

def predecir(P, X):
    W1, b1, W2, b2, W3, b3 = P
    a1 = sig(X @ W1.T + b1)
    a2 = sig(a1 @ W2.T + b2)
    return np.argmax(sig(a2 @ W3.T + b3), axis=1)

def leer_test():
    with open(RUTA + "t10k-images-idx3-ubyte", "rb") as f:
        _, n, fi, co = struct.unpack(">IIII", f.read(16))
        X = np.frombuffer(f.read(), dtype=np.uint8).reshape(n, fi*co)
    with open(RUTA + "t10k-labels-idx1-ubyte", "rb") as f:
        f.read(8)
        y = np.frombuffer(f.read(), dtype=np.uint8)
    return X.astype(np.float64) / 255.0, y

if len(sys.argv) < 2:
    sys.exit("Uso: python3 evaluar_bin.py REF.bin [otros.bin ...]")

X, y = leer_test()
ref = None
for ruta in sys.argv[1:]:
    p = predecir(cargar_bin(ruta), X)
    linea = f"{ruta}: test = {100*np.mean(p == y):.2f}%"
    if ref is None:
        ref = p
    else:
        linea += f" | argmax distinto de la ref: {np.sum(p != ref)} imagenes"
    print(linea)