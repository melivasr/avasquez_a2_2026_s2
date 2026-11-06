# gen_pesos_prueba.py
# Genera un archivo de pesos (mismo formato que pesos_iniciales.bin) pensado
# para ejercitar el RTL:
#   modo "b"   : sesgos aleatorios distintos de cero (en la imagen 0 / epoca 0
#                los sesgos de pesos_iniciales.bin valen 0 y no se prueba el sumador de sesgo)
#   modo "sat" : pesos x30 y sesgos mayores, para que z sature a +-8
# Uso:  python3 gen_pesos_prueba.py pesos_iniciales.bin pesos_b.bin b
#       python3 gen_pesos_prueba.py pesos_iniciales.bin pesos_sat.bin sat
# Luego: ./emulador fixed 1 --n=1 --pesos=pesos_b.bin --vectores=vb
import sys
import numpy as np
entrada, salida, modo = sys.argv[1], sys.argv[2], sys.argv[3]
rng = np.random.default_rng(11)
v = np.fromfile(entrada, dtype="<f8").copy()
tam = [128*784, 128, 64*128, 64, 10*64, 10]      # W1 b1 W2 b2 W3 b3
off = np.cumsum([0] + tam)
for i, (a, b) in enumerate(zip(off[:-1], off[1:])):
    if i % 2 == 0:                               # matriz W
        if modo == "sat": v[a:b] *= 30
    else:                                        # sesgo b
        v[a:b] = rng.uniform(-0.6, 0.6, b - a) * (2 if modo == "sat" else 1)
v.astype("<f8").tofile(salida)
print("escrito", salida, "modo", modo)