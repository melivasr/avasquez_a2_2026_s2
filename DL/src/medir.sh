#!/bin/bash
# uso: ./medir.sh N EPOCAS REPS     (DL_LOCAL, DL_GROUP, etc. se heredan del entorno)
N=${1:-1000}; E=${2:-5}; R=${3:-5}
export RUSTICL_ENABLE=radeonsi DL_QUIET=1 DL_SYNC=1 DL_SAMPLES=$N
for i in 1 2 3 4 5; do ./mlp_gpu $E >/dev/null 2>&1; done        # calentamiento, se descarta
for i in $(seq $R); do ./mlp_gpu $E 2>/dev/null | awk '/Tiempo/ {print $3}'; done \
  | sort -n | awk '{a[NR]=$1} END {print "min", a[1], "mediana", a[int((NR+1)/2)], "max", a[NR], "(us)"}'