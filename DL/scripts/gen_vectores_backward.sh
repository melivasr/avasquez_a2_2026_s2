#!/bin/sh
# ============================================================
# gen_vectores_backward.sh - genera los vectores sinteticos de MATVEC_T + DERIV
# con el emulador (--test-backward) y los verifica con check_backward.py.
#
# Uso (desde sim/):
#   sh ../scripts/gen_vectores_backward.sh [NSEM]      (NSEM semillas por caso; def. 3)
# Deja los vectores en sim/ con prefijo  sb_c<caso>_s<semilla>  (p. ej. sb_c1_s2_c3_W.hex)
# y termina con codigo != 0 si algo falla. Es SIMULACION/vectores, no hardware.
# ============================================================
set -e
NSEM=${1:-3}
EMU=${EMU:-../src/emulador}
CHK=${CHK:-../scripts/check_backward.py}
[ -x "$EMU" ] || { echo "ERROR: no existe $EMU (compila con 'make emulador' en src/)"; exit 1; }
[ -f "$CHK" ] || { echo "ERROR: no existe $CHK"; exit 1; }
fallos=0
for caso in 0 1 2 3; do
  s=1
  while [ $s -le $NSEM ]; do
    P=sb_c${caso}_s${s}
    $EMU --test-backward --caso=$caso --semilla=$s --vectores=$P | sed 's/^# /  /'
    if python3 $CHK $P sint | tail -1 | grep -q PASS; then
      echo "  -> $P: PASS"
    else
      echo "  -> $P: FAIL"; fallos=$((fallos+1))
    fi
    s=$((s+1))
  done
done
echo "=== juegos con fallo: $fallos ==="
[ $fallos -eq 0 ]