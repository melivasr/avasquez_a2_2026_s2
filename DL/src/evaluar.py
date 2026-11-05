# evaluar.py  (NO ejecutado por mí; correr desde src/ como predict.py)
import numpy as np, struct

RUTA = "../data/mnist/"
P = "../data/params/"
W1 = np.loadtxt(P+"W1.txt"); b1 = np.loadtxt(P+"b1.txt").reshape(-1)
W2 = np.loadtxt(P+"W2.txt"); b2 = np.loadtxt(P+"b2.txt").reshape(-1)
W3 = np.loadtxt(P+"W3.txt"); b3 = np.loadtxt(P+"b3.txt").reshape(-1)

def sig(z): return 1.0 / (1.0 + np.exp(-np.clip(z, -500, 500)))

def leer(prefijo, n=None):
    with open(RUTA+prefijo+"-images-idx3-ubyte", "rb") as f:
        _, cant, filas, cols = struct.unpack(">IIII", f.read(16))
        X = np.frombuffer(f.read(), dtype=np.uint8).reshape(cant, filas*cols)
    with open(RUTA+prefijo+"-labels-idx1-ubyte", "rb") as f:
        f.read(8)
        y = np.frombuffer(f.read(), dtype=np.uint8)
    if n: X, y = X[:n], y[:n]
    return X.astype(np.float64)/255.0, y

def predecir(X):                      # X: (N,784), forward por lotes
    a1 = sig(X @ W1.T + b1)
    a2 = sig(a1 @ W2.T + b2)
    return np.argmax(sig(a2 @ W3.T + b3), axis=1)

def reporte(nombre, X, y):
    p = predecir(X)
    print(f"{nombre}: accuracy = {100*np.mean(p == y):.2f}%  (N={len(y)})")
    return p

Xtr, ytr = leer("train", 1000)        # mismo subconjunto que el entrenamiento
Xte, yte = leer("t10k")               # 10000 imagenes nunca vistas

reporte("TRAIN (1000)", Xtr, ytr)     # debe dar ~98%: valida que Python == C++
p = reporte("TEST  (10000)", Xte, yte)  # generalizacion real

print("\nAccuracy por clase en test:")
for c in range(10):
    m = yte == c
    print(f"  {c}: {100*np.mean(p[m] == c):.1f}%")

conf = np.zeros((10, 10), dtype=int)  # filas = real, columnas = predicho
for r, q in zip(yte, p): conf[r, q] += 1
print("\nMatriz de confusion (fila=real, col=predicho):\n", conf)
print("t10k[61]: real =", yte[61], " predicho =", p[61])