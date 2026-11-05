import struct
from PIL import Image

# Numero a buscar
num = 3
RUTA = "../data/mnist/"

# Buscar la primera imagen cuya etiqueta sea 8
with open(RUTA + "t10k-labels-idx1-ubyte", "rb") as f:
    magic, cantidad = struct.unpack(">II", f.read(8))
    etiquetas = f.read()

indice = etiquetas.index(num)

# Extraer la imagen correspondiente
with open(RUTA + "t10k-images-idx3-ubyte", "rb") as f:
    magic, cantidad, filas, columnas = struct.unpack(">IIII", f.read(16))

    f.seek(16 + indice * filas * columnas)
    datos = f.read(filas * columnas)

    imagen = Image.frombytes("L", (columnas, filas), datos)
    imagen.save("prueba.png")

print("Imagen guardada: prueba.png")
print("Índice:", indice)
print("Etiqueta real:", etiquetas[indice])