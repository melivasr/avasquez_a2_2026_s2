#!/bin/sh
# ============================================================
# sim_delta_out.sh - prueba de delta_out contra la traza del emulador
# Uso (desde sim/):
#   ../src/emulador fixed 20 --n=100 --pesos=../src/pesos_iniciales.bin --vectores=vd
#   sh ../scripts/sim_delta_out.sh vd
# Simula TODAS las lineas de PREF_delta.hex (100 imagenes x 20 epocas = 2000).
# ============================================================
set -e
P=${1:?Falta el prefijo (el de --vectores=)}
RTL=${RTL:-../rtl}
if [ ! -f "${P}_delta.hex" ]; then
  echo "ERROR: no existe ${P}_delta.hex en $(pwd)."
  echo "Genera la traza primero, desde sim/:"
  echo "  ../src/emulador fixed 20 --n=100 --pesos=../src/pesos_iniciales.bin --vectores=${P}"
  exit 1
fi
N=$(wc -l < ${P}_delta.hex | tr -d ' ')
[ "$N" -gt 0 ] || { echo "ERROR: ${P}_delta.hex esta vacio"; exit 1; }
rm -f sim_delta_out.vvp
iverilog -g2012 -o sim_delta_out.vvp -s tb_delta_out \
  -Ptb_delta_out.NVEC=$N -Ptb_delta_out.F_VEC=\"${P}_delta.hex\" \
  $RTL/tb_delta_out.sv $RTL/delta_out.sv || { echo "ERROR: fallo la compilacion (iverilog)"; exit 1; }
vvp sim_delta_out.vvp > sim_delta_out.log
grep -E "DELTA_OUT|PASS|FAIL|vec " sim_delta_out.log
grep -q PASS sim_delta_out.log