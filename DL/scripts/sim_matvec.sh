#!/bin/sh
# ============================================================
# sim_matvec.sh  -  regresion de motor_matvec contra el emulador
#
# Uso (desde src/, donde estan los vec_*.hex):
#   ../scripts/sim_matvec.sh PREFIJO [L1 L2 ...]
#   p. ej.:  ../scripts/sim_matvec.sh vec 7 32 48
#
# Para cada L simula las 3 capas y compara z y a bit a bit con los
# vectores de:  ./emulador fixed 1 --n=1 --vectores=PREFIJO
# Imprime PASS/FAIL por capa. Es SIMULACION (Icarus), no hardware.
# ============================================================
set -e
P=${1:?Falta el prefijo de los vectores (el de --vectores=)}
shift
LS=${*:-"7 32 48"}
RTL=${RTL:-../rtl}

sim() {  # L N_IN N_OUT capa archivo_x
  iverilog -g2012 -o sim_matvec.vvp -s tb_matvec \
    -Ptb_matvec.L=$1 -Ptb_matvec.N_IN=$2 -Ptb_matvec.N_OUT=$3 \
    -Ptb_matvec.F_LUT=\"${P}_lut.hex\" \
    -Ptb_matvec.F_W=\"${P}_W$4_pre.hex\" -Ptb_matvec.F_B=\"${P}_b$4_pre.hex\" \
    -Ptb_matvec.F_X=\"$5\" -Ptb_matvec.F_Z=\"${P}_z$4.hex\" -Ptb_matvec.F_A=\"${P}_a$4.hex\" \
    $RTL/tb_matvec.sv $RTL/motor_matvec.sv $RTL/lane_mac.sv $RTL/sigmoide_lut.sv
  vvp sim_matvec.vvp | grep -E "^L=|PASS|FAIL|FALLO"
}

fallos=0
for L in $LS; do
  for capa in 1 2 3; do
    case $capa in
      1) args="784 128 1 ${P}_x.hex" ;;
      2) args="128 64 2 ${P}_a1.hex" ;;
      3) args="64 10 3 ${P}_a2.hex" ;;
    esac
    echo "--- capa $capa, L=$L ---"
    out=$(sim $L $args)
    echo "$out"
    echo "$out" | grep -q PASS || fallos=$((fallos+1))
  done
done
echo "=== simulaciones con fallo: $fallos ==="
[ $fallos -eq 0 ]