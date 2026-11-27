#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "$0")/../.." && pwd)
rtl="$repo/DL/rtl"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

python3 "$repo/DL/scripts/gen_craft21_mmio.py" "$work"
cpu=("$rtl"/cpu/{TOP,DE,IF,MEM,utils,async_fifo}/*.sv)
accel=("$rtl"/acelerador/{dl_periph,dl_avalon,motor_dl,lane_mac,rom_dataset,delta_out,sigmoide_lut}.sv)

run_checked() {
    local dir=$1 marker=$2 exe=$3
    shift 3
    local log="$work/$(basename "$dir")-$exe.log"
    if ! (cd "$dir" && timeout 15s vvp "$work/$exe.vvp" "$@") >"$log" 2>&1; then
        cat "$log"
        return 1
    fi
    # Omite el resumen con cero fallos que imprime tb_imm_extend.
    local unexpected
    unexpected=$(grep -Ei '(^|[^[:alpha:]])(FATAL|ERROR|FALLO|FAIL)([^[:alpha:]]|$)|unable to open' "$log" |
        grep -vE 'Resultados: [0-9]+ PASS  +0 FAIL$' || true)
    if [[ -n $unexpected ]] || ! grep -Fq "$marker" "$log"; then
        cat "$log"
        return 1
    fi
    grep -F "$marker" "$log"
}

iverilog -g2012 -s tb_craft21_encoding -o "$work/encoding.vvp" \
    "${cpu[@]}" "$rtl/cpu/tb/tb_craft21_encoding.sv"
run_checked "$work" "PASS Craft21 encoding (8 vectors)" encoding

# Los programas MMIO no emiten comandos al acelerador. Se crean imágenes de ROM
# completas y deterministas para la elaboración y la inicialización.
for fixture in id integrado; do
    python3 - "$work/$fixture" <<'PY'
from pathlib import Path
import sys

directory = Path(sys.argv[1])
for name, count, word in (("lut.hex", 1025, "0000"),
                          ("ds_img.hex", 784, "00"), ("ds_lbl.hex", 1, "00")):
    (directory / name).write_text((word + "\n") * count)
PY
done
iverilog -g2012 -s tb_craft21_dl -o "$work/integration.vvp" \
    "${cpu[@]}" "${accel[@]}" "$rtl/craft21_dl_top.sv" "$rtl/cpu/tb/tb_craft21_dl.sv"
run_checked "$work/id" "PASS ID=444c0100 in RAM[0x100], accepted reads=1" integration +smoke
run_checked "$work/integrado" "PASS dl_periph: CFG=00140020" integration

iverilog -g2012 -s tb_craft21_wait -o "$work/wait.vvp" \
    "${cpu[@]}" "$rtl/cpu/tb/tb_craft21_wait.sv"
run_checked "$work/espera" "PASS Avalon wait:" wait

fixture="$work/regresiones"
mkdir -p "$fixture"/{programs,sim/waves,outputs/reports}
cp "$work/id/programs/data.hex" "$fixture/programs/"
python3 - "$fixture/programs" <<'PY_FIXTURES'
import sys
from pathlib import Path

programs = Path(sys.argv[1])
nop = "00580000\n"
freeze = "00600000\n"
general = ["00070021\n", "00808002\n", freeze]  # ADDI x2, 7; SW x2, 0x100; FREEZE.
(programs / "program.hex").write_text("".join(general) + nop * (4096 - len(general)))
topg = ["000500b1\n", "00050031\n", "00050051\n", "7ff00021\n"]
(programs / "topg_fixture.hex").write_text("".join(topg) + nop * 23 + freeze + nop * (16384 - 28))
PY_FIXTURES

run_legacy() {
    local bench=$1 marker=$2
    shift 2
    iverilog -g2012 -s "$bench" -o "$work/$bench.vvp" "${cpu[@]}" "$rtl/cpu/tb/$bench.sv"
    printf '%s: ' "$bench"
    run_checked "$fixture" "$marker" "$bench" "$@"
}

run_legacy tb_alu "37 PASADOS  /  0 FALLIDOS"
run_legacy tb_control_unit "24 PASADOS  /  0 FALLIDOS"
run_legacy tb_imm_extend "34 PASS  0 FAIL"
run_legacy tb_pc "8 PASADOS  /  0 FALLIDOS"
run_legacy tb_pc_decoder "2 PASADOS  /  0 FALLIDOS"
run_legacy tb_sumador_pc "2 PASADOS  /  0 FALLIDOS"
run_legacy tb_hazard_unit "Simulacion completa."
run_legacy tb_data_ram "TODOS LOS TESTS COMPLETADOS"
run_legacy tb_instr_rom "Fin de tb_instr_rom"
run_legacy tb_general_dump "FIN TB GENERAL"
grep -Eiq 'x02.*00000007' "$fixture/outputs/general_regs.txt"
# topG usa su propia imagen de prueba y espera que el PC se detenga en 0x6C.
cp "$fixture/programs/topg_fixture.hex" "$fixture/programs/program.hex"
run_legacy tb_topG "ALL TESTS PASSED" +FILE_ROM=programs/topg_fixture.hex +FILE_RAM=programs/data.hex

# Estos bancos muestran trazas y requieren inspección manual del resultado.
for bench in tb_async_fifo tb_clk_divider tb_counter tb_fifo tb_regfile; do
    run_legacy "$bench" '$finish called'
done

iverilog -g2012 -s tb_top -o "$work/tb_top.vvp" "${cpu[@]}" "$rtl/cpu/tb/tb_top.sv"
printf '%s\n' 'SKIP tb_top execution: original demo.hex, busqueda_arreglo.hex and factorial.hex are absent'
