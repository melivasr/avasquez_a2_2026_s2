#!/bin/sh
# ============================================================
# gen_vectores_update.sh - genera los vectores sinteticos de UPDATE con el emulador
# (--test-update) para los 3 redondeos y los verifica con check_update.py (Python independiente).
#
# Uso (desde sim/):
#   sh ../scripts/gen_vectores_update.sh [NSEM] [T]    (NSEM semillas por caso, def. 2; T imagenes, def. 4)
# Deja los vectores en sim/ con prefijo  su_<modo>_c<caso>_s<semilla>  y termina con codigo != 0 si algo falla.
# ============================================================
set -e
NSEM=${1:-2}
T=${2:-4}
EMU=${EMU:-../src/emulador}
CHK=${CHK:-../scripts/check_update.py}
[ -x "$EMU" ] || { echo "ERROR: no existe $EMU (compila con 'make emulador' en src/)"; exit 1; }
[ -f "$CHK" ] || { echo "ERROR: no existe $CHK"; exit 1; }
fallos=0
for modo in cercano estocastico piso; do
  for caso in 0 1 2 3; do
    s=1
    while [ $s -le $NSEM ]; do
      P=su_${modo}_c${caso}_s${s}
      $EMU --test-update --caso=$caso --semilla=$s --n=$T --redondeo=$modo --vectores=$P | sed 's/^# /  /'
      if python3 $CHK $P sint | tail -1 | grep -q PASS; then echo "  -> $P: PASS"; else echo "  -> $P: FAIL"; fallos=$((fallos+1)); fi
      s=$((s+1))
    done
  done
done
echo "=== juegos con fallo: $fallos ==="
[ $fallos -eq 0 ]