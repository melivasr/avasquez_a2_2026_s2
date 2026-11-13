# Makefile - SIMD (AVX2) 

UNAME := $(shell uname -s)

ifeq ($(UNAME),Darwin)
  CXX := clang++
  EIGEN_INC ?= /opt/homebrew/include/eigen3
else
  EIGEN_INC ?= /usr/include/eigen3
endif

SRC := ../src
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -isystem $(EIGEN_INC) -I$(SRC) $(EXTRA)
SIMDFLAGS := -mavx2 -mno-fma -ffp-contract=off

.PHONY: all clean test validar

all: main_simd
lector_simd.o: $(SRC)/lector.cpp $(SRC)/mnist.h
	$(CXX) $(CXXFLAGS) $(SIMDFLAGS) -c $< -o $@

# Un objeto por modulo.
OBJS := main_simd.o entrenamiento.o capas.o kernels_avx2.o lector_simd.o

kernels_avx2.o:  kernels_avx2.cpp kernels_avx2.h
capas.o: capas.cpp capas.h kernels_avx2.h
entrenamiento.o: entrenamiento.cpp entrenamiento.h capas.h kernels_avx2.h $(SRC)/mnist.h
main_simd.o: main_simd.cpp entrenamiento.h $(SRC)/mnist.h $(SRC)/checksum.h $(SRC)/runtime.h

%.o: %.cpp $(CXX) $(CXXFLAGS) $(SIMDFLAGS) -flto -c $< -o $@

main_simd: $(OBJS) $(CXX) $(CXXFLAGS) $(SIMDFLAGS) -flto $(OBJS) -o $@

# Numero de epocas para test y validar. Se cambia en la terminal:
# make test EPOCAS=20
EPOCAS ?= 2

# Test: ejecuta main_simd.
test: main_simd
	./main_simd $(EPOCAS)

# Validacion secuencial == paralelo, ambos checksums deben ser iguales.
validar: main_simd
	$(MAKE) -C $(SRC) -W main.cpp main
	@cd $(SRC) && DL_QUIET=1 ./main $(EPOCAS) | grep Checksum
	@DL_QUIET=1 ./main_simd $(EPOCAS) | grep Checksum

clean:
	rm -f $(OBJS) main_simd
