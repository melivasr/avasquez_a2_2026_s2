# ============================================================
# Makefile - Proyecto ARQUI 2 (MLP 784-128-64-10, MNIST)
#
# Por ahora solo compila el modelo secuencial: main.cpp + lector.cpp
#
# Uso:
#   make          compila ./main
#   make test     ejecuta ./main con 2 epocas (prueba rapida)
#   make clean
#
# Variables (se pueden sobreescribir):
#   EIGEN_INC=/ruta/a/eigen3 make
#   CXX=g++ make
#   EXTRA="-g -O0" make      (depuracion)
# ============================================================

UNAME := $(shell uname -s)

ifeq ($(UNAME),Darwin)
  CXX       := clang++
  EIGEN_INC ?= /opt/homebrew/include/eigen3
else
  EIGEN_INC ?= /usr/include/eigen3
endif

# Sin -ffast-math ni -Ofast: reordenan sumas en punto flotante y
# romperian la comparacion de checksums entre versiones.
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -I$(EIGEN_INC) -I. $(EXTRA)

.PHONY: all clean test

all: main

lector.o: lector.cpp mnist.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main: main.cpp lector.o mnist.h checksum.h
	$(CXX) $(CXXFLAGS) $< lector.o -o $@

test: main
	./main 2

clean:
	rm -f lector.o main