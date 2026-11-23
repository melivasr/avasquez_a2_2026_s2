#!/usr/bin/env bash
# ============================================================
# demo_avance2.sh
#
# Versión dummy que corre con Icarus Verilog, 
# los testbenches de cada modulo del
# acelerador por separado y resume el resultado en una tabla.
#
# Uso:       desde scripts/: ./demo_avance2.sh
#
# Cada modulo, AISLADO, coincide bit a bit con el
# emulador en punto fijo para los vectores de los .hex.
# Que NO demuestra: sistema integrado, ISA, JTAG ni tiempo real
# (los ciclos son de SIMULACION).
#
# Un PASS solo se acepta como valido si el log NO contiene errores
# de lectura de archivos ($readmemh) ni la palabra FAIL/FALLO.
# ============================================================
cd "$(dirname "$0")/../sim" || { echo "No se encontro DL/sim"; exit 2; }

RTL="${RTL:-../rtl}"
OUT="logs_demo"
mkdir -p "$OUT"

# colores (solo si la salida es una terminal)
if [ -t 1 ]; then G=$'\033[32m'; R=$'\033[31m'; Y=$'\033[33m'; B=$'\033[1m'; N=$'\033[0m'
else G=""; R=""; Y=""; B=""; N=""; fi

# ---------------- encabezado ----------------
echo "${B}=== Avance 2: acelerador FPGA (MLP 784-128-64-10), simulacion con Icarus ===${N}"
date "+Fecha: %Y-%m-%d %H:%M"
iverilog -V 2>&1 | head -1
echo
echo "${B}Archivos RTL y testbench (${RTL}):${N}"
ls -1 "$RTL"/*.sv
echo

# ---------------- verificar que existan los .hex ----------------
HEX="vb_lut vb_x vb_W1_pre vb_W2_pre vb_W3_pre vb_b1_pre vb_b2_pre vb_b3_pre \
vb_a1 vb_a2 vb_a3 vb_z1 vb_z2 vb_z3 vb_dz1 vb_dz2 vb_dz3 vb_dA1 vb_dA2 vb_delta"
faltan=0
for h in $HEX; do
    if [ ! -s "$h.hex" ]; then echo "${R}Falta o esta vacio: $h.hex${N}"; faltan=1; fi
done
[ $faltan -eq 1 ] && { echo "${R}Abortando: faltan vectores dorados.${N}"; exit 2; }
echo "Vectores dorados (.hex): $(echo $HEX | wc -w | tr -d ' ') archivos encontrados"
echo

# ---------------- resultados acumulados ----------------
RES_MOD=(); RES_TEST=(); RES_STAT=(); RES_CYC=()
TOTAL_MAL=0

# run_tb "Modulo" "Prueba" top fuentes [-Pparametros...]
run_tb() {
    local mod="$1" test="$2" top="$3" srcs="$4"; shift 4
    local tag; tag=$(echo "${top}_${test}" | tr -c 'A-Za-z0-9\n' '_')
    local vvp="$OUT/$tag.vvp" log="$OUT/$tag.log"
    local stat cyc="-"

    echo "${B}> $mod | $test${N}"
    if ! iverilog -g2012 -s "$top" -o "$vvp" "$@" $srcs > "$OUT/$tag.compila.log" 2>&1; then
        echo "${R}  NO COMPILA/ELABORA:${N}"; head -5 "$OUT/$tag.compila.log" | sed 's/^/    /'
        stat="NO COMPILA"
    else
        vvp "$vvp" > "$log" 2>&1
        grep -E "error|PASS|FAIL|FALLO|ERROR|Unable" "$log" | sed 's/^/  /'
        if grep -qE "ERROR|Unable|FALLO|FAIL" "$log"; then
            stat="FAIL"
        elif grep -q "^PASS" "$log"; then
            stat="PASS"
        else
            stat="SIN VEREDICTO"
        fi
        cyc=$(grep -oE 'ciclos[^=]*=[0-9]+' "$log" | head -1 | grep -oE '[0-9]+$')
        [ -z "$cyc" ] && cyc="-"
    fi
    [ "$stat" != "PASS" ] && TOTAL_MAL=$((TOTAL_MAL + 1))
    RES_MOD+=("$mod"); RES_TEST+=("$test"); RES_STAT+=("$stat"); RES_CYC+=("$cyc")
    echo
}

FWD="$RTL/lane_mac.sv $RTL/sigmoide_lut.sv $RTL/motor_matvec.sv $RTL/tb_matvec.sv"
BWD="$RTL/lane_mac.sv $RTL/motor_matvec_t.sv $RTL/tb_matvec_t.sv"
DLT="$RTL/delta_out.sv $RTL/tb_delta_out.sv"

# ---------------- 1. FORWARD: z y a de cada capa ----------------
run_tb "Forward" "capa 1 (784->128)" tb_matvec "$FWD" \
  '-Ptb_matvec.F_LUT="vb_lut.hex"' '-Ptb_matvec.F_W="vb_W1_pre.hex"' \
  '-Ptb_matvec.F_B="vb_b1_pre.hex"' '-Ptb_matvec.F_X="vb_x.hex"' \
  '-Ptb_matvec.F_Z="vb_z1.hex"' '-Ptb_matvec.F_A="vb_a1.hex"'

run_tb "Forward" "capa 2 (128->64)" tb_matvec "$FWD" \
  -Ptb_matvec.N_IN=128 -Ptb_matvec.N_OUT=64 \
  '-Ptb_matvec.F_LUT="vb_lut.hex"' '-Ptb_matvec.F_W="vb_W2_pre.hex"' \
  '-Ptb_matvec.F_B="vb_b2_pre.hex"' '-Ptb_matvec.F_X="vb_a1.hex"' \
  '-Ptb_matvec.F_Z="vb_z2.hex"' '-Ptb_matvec.F_A="vb_a2.hex"'

run_tb "Forward" "capa 3 (64->10)" tb_matvec "$FWD" \
  -Ptb_matvec.N_IN=64 -Ptb_matvec.N_OUT=10 \
  '-Ptb_matvec.F_LUT="vb_lut.hex"' '-Ptb_matvec.F_W="vb_W3_pre.hex"' \
  '-Ptb_matvec.F_B="vb_b3_pre.hex"' '-Ptb_matvec.F_X="vb_a2.hex"' \
  '-Ptb_matvec.F_Z="vb_z3.hex"' '-Ptb_matvec.F_A="vb_a3.hex"'

# ---------------- 2. DELTA_OUT: error de salida ----------------
NVEC=$(grep -c . vb_delta.hex)
run_tb "Delta_out" "salida (${NVEC} vector)" tb_delta_out "$DLT" \
  -Ptb_delta_out.NVEC=$NVEC '-Ptb_delta_out.F_VEC="vb_delta.hex"'

# ---------------- 3. BACKWARD: dA y dz_prev ----------------
run_tb "Backward" "capa 3->2 (10x64)" tb_matvec_t "$BWD" \
  -Ptb_matvec_t.N_OUT=10 -Ptb_matvec_t.N_IN=64 \
  '-Ptb_matvec_t.F_W="vb_W3_pre.hex"' '-Ptb_matvec_t.F_DZ="vb_dz3.hex"' \
  '-Ptb_matvec_t.F_A="vb_a2.hex"' '-Ptb_matvec_t.F_DA="vb_dA2.hex"' \
  '-Ptb_matvec_t.F_DZP="vb_dz2.hex"'

run_tb "Backward" "capa 2->1 (64x128)" tb_matvec_t "$BWD" \
  '-Ptb_matvec_t.F_W="vb_W2_pre.hex"' '-Ptb_matvec_t.F_DZ="vb_dz2.hex"' \
  '-Ptb_matvec_t.F_A="vb_a1.hex"' '-Ptb_matvec_t.F_DA="vb_dA1.hex"' \
  '-Ptb_matvec_t.F_DZP="vb_dz1.hex"'

# ---------------- 4. PARALELISMO: mismo resultado con otro numero de lanes ----------------
run_tb "Forward" "capa 2, L=7 lanes" tb_matvec "$FWD" \
  -Ptb_matvec.L=7 -Ptb_matvec.N_IN=128 -Ptb_matvec.N_OUT=64 \
  '-Ptb_matvec.F_LUT="vb_lut.hex"' '-Ptb_matvec.F_W="vb_W2_pre.hex"' \
  '-Ptb_matvec.F_B="vb_b2_pre.hex"' '-Ptb_matvec.F_X="vb_a1.hex"' \
  '-Ptb_matvec.F_Z="vb_z2.hex"' '-Ptb_matvec.F_A="vb_a2.hex"'

run_tb "Backward" "capa 2->1, L=7 lanes" tb_matvec_t "$BWD" \
  -Ptb_matvec_t.L=7 \
  '-Ptb_matvec_t.F_W="vb_W2_pre.hex"' '-Ptb_matvec_t.F_DZ="vb_dz2.hex"' \
  '-Ptb_matvec_t.F_A="vb_a1.hex"' '-Ptb_matvec_t.F_DA="vb_dA1.hex"' \
  '-Ptb_matvec_t.F_DZP="vb_dz1.hex"'

# ---------------- resumen (esta es la parte para la captura) ----------------
echo "${B}==================== RESUMEN ====================${N}"
printf "%-10s %-24s %-14s %s\n" "Modulo" "Prueba" "Resultado" "Ciclos sim."
printf "%-10s %-24s %-14s %s\n" "------" "------" "---------" "-----------"
for i in "${!RES_MOD[@]}"; do
    s="${RES_STAT[$i]}"
    if [ "$s" = "PASS" ]; then c="$G"; else c="$R"; fi
    printf "%-10s %-24s %s%-14s%s %s\n" "${RES_MOD[$i]}" "${RES_TEST[$i]}" "$c" "$s" "$N" "${RES_CYC[$i]}"
done
echo
n=${#RES_MOD[@]}
if [ $TOTAL_MAL -eq 0 ]; then
    echo "${G}${B}$n/$n pruebas coinciden bit a bit con el emulador${N}"
else
    echo "${R}${B}$((n - TOTAL_MAL))/$n pruebas pasan; $TOTAL_MAL con problema (ver $OUT/)${N}"
fi
echo "${Y}Alcance: modulos AISLADOS, 1 vector por capa; ciclos de simulacion, no tiempo real.${N}"
echo "${Y}Pendiente: top integrado, ISA, JTAG, medicion de tiempo en placa, actualizacion de W.${N}"

[ $TOTAL_MAL -eq 0 ]