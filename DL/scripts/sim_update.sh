#!/bin/sh
# ============================================================
# sim_update.sh - regresion de motor_update (UPDATE) contra el emulador
#
# Uso (desde sim/, donde estan los .hex):
#   sh ../scripts/sim_update.sh PREFIJO [L1 L2 ...]
#   p. ej.:  sh ../scripts/sim_update.sh ur_estocastico 1 7 32
#            sh ../scripts/sim_update.sh su_estocastico_c1_s1
#
# PREFIJO puede ser:
#   - vectores del ENTRENAMIENTO (1 imagen, las 3 capas):
#       ../src/emulador fixed 1 --n=1 --pesos=pesos_b.bin --redondeo=estocastico --vectores=PREFIJO
#   - vectores SINTETICOS (T imagenes encadenadas, 2 capas):
#       ../src/emulador --test-update --caso=N --semilla=S --n=4 --redondeo=estocastico --vectores=PREFIJO
# El redondeo y los fw se leen de PREFIJO_modo.txt (lo escribe el emulador), asi que no pueden
# desajustarse con los vectores. Sin L's: usa 1 7 10 16 32 48 64 128.
# Compara W, b y el estado del LFSR bit a bit; imprime PASS/FAIL. Es SIMULACION (Icarus).
# Termina con codigo != 0 si algo falla, no compila o falta un archivo.
# ============================================================
set -e
P=${1:?Falta el prefijo de los vectores (el de --vectores=)}
shift
LS=${*:-"1 7 10 16 32 48 64 128"}
RTL=${RTL:-../rtl}

[ -f "${P}_modo.txt" ] || { echo "ERROR: falta ${P}_modo.txt en $(pwd). Regenera los vectores con el emulador v5 (--vectores=${P})."; exit 1; }
set -- $(cat ${P}_modo.txt)
MODO=$1; FW1=$2; FW2=$3; FW3=$4
case $MODO in cercano) RM=0;; estocastico) RM=1;; piso) RM=2;; *) echo "ERROR: modo desconocido '$MODO' en ${P}_modo.txt"; exit 1;; esac

# lista de capas: "nombre n_out n_in fw archivos(WPRE BPRE LFPRE DZ A WPOST BPOST LFPOST)"
if [ -f "${P}_u3_W_pre.hex" ]; then
  ORIGEN=sintetico
  CAPAS="u3|10|64|$FW3|${P}_u3_W_pre.hex ${P}_u3_b_pre.hex ${P}_u3_lf_pre.hex ${P}_u3_dz.hex ${P}_u3_a.hex ${P}_u3_W_post.hex ${P}_u3_b_post.hex ${P}_u3_lf_post.hex
u2|64|128|$FW2|${P}_u2_W_pre.hex ${P}_u2_b_pre.hex ${P}_u2_lf_pre.hex ${P}_u2_dz.hex ${P}_u2_a.hex ${P}_u2_W_post.hex ${P}_u2_b_post.hex ${P}_u2_lf_post.hex"
elif [ -f "${P}_W3_pre.hex" ]; then
  ORIGEN=entrenamiento
  CAPAS="capa3|10|64|$FW3|${P}_W3_pre.hex ${P}_b3_pre.hex ${P}_lf3_pre.hex ${P}_dz3.hex ${P}_a2.hex ${P}_W3_post.hex ${P}_b3_post.hex ${P}_lf3_post.hex
capa2|64|128|$FW2|${P}_W2_pre.hex ${P}_b2_pre.hex ${P}_lf2_pre.hex ${P}_dz2.hex ${P}_a1.hex ${P}_W2_post.hex ${P}_b2_post.hex ${P}_lf2_post.hex
capa1|128|784|$FW1|${P}_W1_pre.hex ${P}_b1_pre.hex ${P}_lf1_pre.hex ${P}_dz1.hex ${P}_x.hex ${P}_W1_post.hex ${P}_b1_post.hex ${P}_lf1_post.hex"
else
  echo "ERROR: no encuentro vectores con prefijo '${P}' en $(pwd)."
  echo "  entrenamiento: ../src/emulador fixed 1 --n=1 --pesos=pesos_b.bin --vectores=${P}"
  echo "  sinteticos:    ../src/emulador --test-update --caso=1 --semilla=1 --n=4 --vectores=${P}"
  exit 1
fi
for f in $(echo "$CAPAS" | cut -d'|' -f5); do
  [ -f "$f" ] || { echo "ERROR: falta $f en $(pwd). Regenera los vectores con el emulador v5."; exit 1; }
done

sim() {  # N_OUT N_IN L FW T archivos(8)
  NO=$1; NI=$2; LL=$3; FWW=$4; TT=$5; shift 5
  iverilog -g2012 -o sim_update.vvp -s tb_update \
    -Ptb_update.L=$LL -Ptb_update.N_IN=$NI -Ptb_update.N_OUT=$NO -Ptb_update.T=$TT \
    -Ptb_update.RMODE=$RM -Ptb_update.FW=$FWW \
    -Ptb_update.F_WPRE=\"$1\" -Ptb_update.F_BPRE=\"$2\" -Ptb_update.F_LFPRE=\"$3\" \
    -Ptb_update.F_DZ=\"$4\" -Ptb_update.F_A=\"$5\" \
    -Ptb_update.F_WPOST=\"$6\" -Ptb_update.F_BPOST=\"$7\" -Ptb_update.F_LFPOST=\"$8\" \
    $RTL/tb_update.sv $RTL/motor_update.sv $RTL/lane_mac.sv || { echo "ERROR: fallo la compilacion (iverilog)"; exit 1; }
  vvp sim_update.vvp | grep -E "^L=|PASS|FAIL|FALLO|^  (W|b|lfsr)"
}

echo "origen: $ORIGEN  prefijo: $P  redondeo: $MODO"
CAPAS_TMP=sim_update_capas.tmp
echo "$CAPAS" > $CAPAS_TMP
fallos=0
for L in $LS; do
  while IFS='|' read -r nom no ni fw fl <&3; do
    set -- $fl                       # $1..$8 = WPRE BPRE LFPRE DZ A WPOST BPOST LFPOST
    ndz=$(wc -l < "$4" | tr -d ' ')
    T=$((ndz / no))
    echo "--- $nom ($no x $ni, T=$T), L=$L ---"
    out=$(sim $no $ni $L $fw $T $fl)
    echo "$out"
    echo "$out" | grep -q PASS || fallos=$((fallos+1))
  done 3< $CAPAS_TMP
done
rm -f $CAPAS_TMP
echo "=== simulaciones con fallo: $fallos ==="
[ $fallos -eq 0 ]