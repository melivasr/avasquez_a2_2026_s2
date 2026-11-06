#!/bin/sh
# ============================================================
# sim_delta_out.sh - prueba de delta_out contra la traza del emulador
# Uso (desde sim/):
#   ../src/emulador fixed 20 --n=100 --vectores=vd
#   sh ../scripts/sim_delta_out.sh vd
# Simula TODAS las lineas de PREF_delta.hex (100 imagenes x 20 epocas = 2000).
# ============================================================
set -e
P=${1:?Falta el prefijo (el de --vectores=)}
RTL=${RTL:-../rtl}
N=$(wc -l < ${P}_delta.hex | tr -d ' ')
iverilog -g2012 -o sim_delta_out.vvp -s tb_delta_out \
  -Ptb_delta_out.NVEC=$N -Ptb_delta_out.F_VEC=\"${P}_delta.hex\" \
  $RTL/tb_delta_out.sv $RTL/delta_out.sv
vvp sim_delta_out.vvp | grep -E "DELTA_OUT|PASS|FAIL|vec "
vvp sim_delta_out.vvp | grep -q PASS