#!/bin/sh
# ============================================================
# sim_dl_bus.sh - regresion del motor integrado VISTO DESDE EL BUS AVALON (dl_avalon + motor_dl)
#
# 1) Genera los vectores (desde sim/), con el MISMO N y EPOCAS que se usaran aqui:
#      ../src/emulador fixed EPOCAS --n=N --pesos=../src/pesos_iniciales.bin --redondeo=estocastico --vectores=PREF
# 2) Simula:   sh ../scripts/sim_dl_bus.sh PREF N EPOCAS [L1 L2 ...]      (sin L's: 32)
#    p. ej.:   sh ../scripts/sim_dl_bus.sh e6 100 2 32        (entrenamiento completo, lento)
#              sh ../scripts/sim_dl_bus.sh m5 5 2 7 32 64     (rapido)
# El redondeo se lee de PREF_modo.txt. Para cada L compara: (nivel 1) a1,a2,a3,dz3,dz2,dz1 y W,b,LFSR de
# la imagen 0 bit a bit; (nivel 2) e^2 y aciertos por epoca y el HASH de enteros final.
# Es SIMULACION (Icarus), no hardware: los ciclos que imprime no son tiempo real.
# Termina con codigo != 0 si algo falla, no compila o falta un archivo.
# ============================================================
set -e
P=${1:?Falta el prefijo de los vectores (el de --vectores=)}
N=${2:?Falta N (imagenes por epoca, el --n= del emulador)}
E=${3:?Falta EPOCAS (las del emulador)}
shift 3
LS=${*:-"32"}
RTL=${RTL:-../rtl}

for f in modo.txt lut.hex img.hex lbl.hex epocas.hex hash.hex W1_pre.hex W2_pre.hex W3_pre.hex \
         b1_pre.hex b2_pre.hex b3_pre.hex lf1_pre.hex lf2_pre.hex lf3_pre.hex \
         W1_post.hex W2_post.hex W3_post.hex b1_post.hex b2_post.hex b3_post.hex \
         lf1_post.hex lf2_post.hex lf3_post.hex a1.hex a2.hex a3.hex dz1.hex dz2.hex dz3.hex; do
  [ -f "${P}_$f" ] || { echo "ERROR: falta ${P}_$f en $(pwd). Genera los vectores con el emulador v6:"; \
    echo "  ../src/emulador fixed $E --n=$N --pesos=../src/pesos_iniciales.bin --redondeo=estocastico --vectores=$P"; exit 1; }
done
NL=$(wc -l < ${P}_lbl.hex | tr -d ' ');   [ "$NL" -eq "$N" ] || { echo "ERROR: ${P}_lbl.hex tiene $NL etiquetas pero N=$N (usa el mismo --n= del emulador)"; exit 1; }
NE=$(wc -l < ${P}_epocas.hex | tr -d ' '); [ "$NE" -eq "$E" ] || { echo "ERROR: ${P}_epocas.hex tiene $NE epocas pero EPOCAS=$E (usa las mismas del emulador)"; exit 1; }
set -- $(cat ${P}_modo.txt)
case $1 in cercano) RM=0;; estocastico) RM=1;; piso) RM=2;; *) echo "ERROR: modo desconocido '$1' en ${P}_modo.txt"; exit 1;; esac

fallos=0
for L in $LS; do
  iverilog -g2012 -o sim_dl.vvp -s tb_dl_bus \
    -Ptb_dl_bus.L=$L -Ptb_dl_bus.N=$N -Ptb_dl_bus.EPOCHS=$E -Ptb_dl_bus.RMODE=$RM -Ptb_dl_bus.PRE=\"$P\" -Ptb_dl_bus.LUTF=\"${P}_lut.hex\" -Ptb_dl_bus.DIRECTED=${DIRECTED:-1} -Ptb_dl_bus.CHECK0=${CHECK0:-1} -Ptb_dl_bus.USE_ROM=${USE_ROM:-0} \
    $RTL/tb_dl_bus.sv $RTL/rom_dataset.sv $RTL/dl_avalon.sv $RTL/motor_dl.sv $RTL/lane_mac.sv $RTL/delta_out.sv $RTL/sigmoide_lut.sv \
    || { echo "ERROR: fallo la compilacion (iverilog)"; exit 1; }
  echo "--- L=$L  N=$N  EPOCAS=$E  redondeo=$1 ---"
  out=$(vvp sim_dl.vvp | grep -E "NIVEL|^epoca|^hash|^ciclos|^BUS|errores=|PASS|FAIL|FALLO")
  echo "$out"
  echo "$out" | grep -q "PASS" || fallos=$((fallos+1))
done
echo "=== configuraciones con fallo: $fallos ==="
[ $fallos -eq 0 ]