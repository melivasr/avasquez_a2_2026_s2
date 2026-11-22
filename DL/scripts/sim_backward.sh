#!/bin/sh
# ============================================================
# sim_backward.sh - regresion de motor_matvec_t (MATVEC_T + DERIV) contra el emulador
#
# Uso (desde sim/, donde estan los .hex):
#   sh ../scripts/sim_backward.sh PREFIJO [L1 L2 ...]
#   p. ej.:  sh ../scripts/sim_backward.sh vb 1 7 10 32 64
#            sh ../scripts/sim_backward.sh sb_c1_s1
#
# PREFIJO puede ser:
#   - vectores del ENTRENAMIENTO:  ../src/emulador fixed 1 --n=1 --pesos=pesos_b.bin --vectores=PREFIJO
#     (usa PREFIJO_W3_pre, _dz3, _a2, _dA2, _dz2  y  _W2_pre, _dz2, _a1, _dA1, _dz1)
#   - vectores SINTETICOS:  ../src/emulador --test-backward --caso=N --semilla=S --vectores=PREFIJO
#     (usa PREFIJO_c3_* y PREFIJO_c2_*)
# Sin L's: usa 1 7 10 16 32 48 64 128. Para cada L simula las 2 capas, compara dA y
# dz_prev bit a bit e imprime PASS/FAIL. Es SIMULACION (Icarus), no hardware.
# Termina con codigo != 0 si alguna simulacion falla o no compila.
# ============================================================
set -e
P=${1:?Falta el prefijo de los vectores (el de --vectores=)}
shift
LS=${*:-"1 7 10 16 32 48 64 128"}
RTL=${RTL:-../rtl}

if [ -f "${P}_c3_W.hex" ]; then
  MODO=sintetico
  F3="${P}_c3_W.hex ${P}_c3_dz.hex ${P}_c3_a.hex ${P}_c3_dA.hex ${P}_c3_dzp.hex"
  F2="${P}_c2_W.hex ${P}_c2_dz.hex ${P}_c2_a.hex ${P}_c2_dA.hex ${P}_c2_dzp.hex"
elif [ -f "${P}_W3_pre.hex" ]; then
  MODO=entrenamiento
  F3="${P}_W3_pre.hex ${P}_dz3.hex ${P}_a2.hex ${P}_dA2.hex ${P}_dz2.hex"
  F2="${P}_W2_pre.hex ${P}_dz2.hex ${P}_a1.hex ${P}_dA1.hex ${P}_dz1.hex"
else
  echo "ERROR: no encuentro vectores con prefijo '${P}' en $(pwd)."
  echo "  entrenamiento: ../src/emulador fixed 1 --n=1 --pesos=pesos_b.bin --vectores=${P}"
  echo "  sinteticos:    ../src/emulador --test-backward --caso=1 --semilla=1 --vectores=${P}"
  exit 1
fi
for f in $F3 $F2; do
  [ -f "$f" ] || { echo "ERROR: falta $f en $(pwd). Regenera los vectores con el emulador v4 (--vectores=${P})."; exit 1; }
done

sim() {  # N_OUT N_IN L archivos(W dz a dA dzp)
  NO=$1; NI=$2; LL=$3; FW_=$4; FDZ=$5; FA=$6; FDA=$7; FDZP=$8
  iverilog -g2012 -o sim_backward.vvp -s tb_matvec_t \
    -Ptb_matvec_t.L=$LL -Ptb_matvec_t.N_IN=$NI -Ptb_matvec_t.N_OUT=$NO \
    -Ptb_matvec_t.F_W=\"$FW_\" -Ptb_matvec_t.F_DZ=\"$FDZ\" -Ptb_matvec_t.F_A=\"$FA\" \
    -Ptb_matvec_t.F_DA=\"$FDA\" -Ptb_matvec_t.F_DZP=\"$FDZP\" \
    $RTL/tb_matvec_t.sv $RTL/motor_matvec_t.sv $RTL/lane_mac.sv || { echo "ERROR: fallo la compilacion (iverilog)"; exit 1; }
  vvp sim_backward.vvp | grep -E "^L=|PASS|FAIL|FALLO"
}

echo "modo: $MODO  prefijo: $P"
fallos=0
for L in $LS; do
  echo "--- capa 3->2 (W3 10x64), L=$L ---"
  out=$(sim 10 64 $L $F3)
  echo "$out"
  echo "$out" | grep -q PASS || fallos=$((fallos+1))
  echo "--- capa 2->1 (W2 64x128), L=$L ---"
  out=$(sim 64 128 $L $F2)
  echo "$out"
  echo "$out" | grep -q PASS || fallos=$((fallos+1))
done
echo "=== simulaciones con fallo: $fallos ==="
[ $fallos -eq 0 ]