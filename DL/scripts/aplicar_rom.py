# aplicar_rom.py - agrega USE_ROM a tb_dl_bus.sv y sim_dl_bus.sh. Ejecutar desde DL/.
# Aborta SIN escribir nada si algun ancla no aparece exactamente una vez.
import os, re, shutil, sys

def leer(f):
    if not os.path.isfile(f): sys.exit(f"ERROR: no existe {f} (ejecuta desde DL/)")
    return open(f).read()

ft = "rtl/acelerador/tb_dl_bus.sv"; t = leer(ft)
fs = "scripts/sim_dl_bus.sh"; s = leer(fs)
if "USE_ROM" in t or "USE_ROM" in s: sys.exit("ya editado")

pat = r'(parameter\s+LUTF\s*=\s*"[^"]*";)'
if len(re.findall(pat, t)) != 1: sys.exit("ABORTO tb_dl_bus: ancla LUTF")
t = re.sub(pat, lambda m: m.group(1) +
    "\n    parameter USE_ROM  = 0;   // 1: el dataset lo sirve rom_dataset en vez del modelo del testbench"
    "\n    wire [7:0] rom_px, rom_lb;", t)

a = ".px_addr(px_addr), .px_data(px_data), .lb_addr(lb_addr), .lb_data(lb_data)"
b = (".px_addr(px_addr), .px_data(USE_ROM ? rom_px : px_data), "
     ".lb_addr(lb_addr), .lb_data(USE_ROM ? rom_lb : lb_data)")
if t.count(a) != 1: sys.exit("ABORTO tb_dl_bus: ancla de puertos px/lb")
t = t.replace(a, b)

a = "// ---- modelo de la SDRAM / ROM (lectura sincrona) ----"
if t.count(a) != 1: sys.exit("ABORTO tb_dl_bus: ancla del modelo de SDRAM")
t = t.replace(a, 'rom_dataset #(.NIMG(N), .IMG_FILE({PRE, "_img.hex"}), .LBL_FILE({PRE, "_lbl.hex"})) u_rom (\n'
                 '        .clk(clk), .px_addr(px_addr), .px_data(rom_px), .lb_addr(lb_addr), .lb_data(rom_lb));\n    ' + a)

a = "-Ptb_dl_bus.CHECK0=${CHECK0:-1}"
if s.count(a) != 1: sys.exit("ABORTO sim_dl_bus.sh: ancla CHECK0")
s = s.replace(a, a + " -Ptb_dl_bus.USE_ROM=${USE_ROM:-0}")
a = "$RTL/tb_dl_bus.sv $RTL/dl_avalon.sv"
if s.count(a) != 1: sys.exit("ABORTO sim_dl_bus.sh: ancla de fuentes")
s = s.replace(a, "$RTL/tb_dl_bus.sv $RTL/rom_dataset.sv $RTL/dl_avalon.sv")

os.makedirs("bak_sint", exist_ok=True)
for f, x in ((ft, t), (fs, s)):
    shutil.copy(f, "bak_sint/" + os.path.basename(f))
    open(f, "w").write(x)
print("OK: tb_dl_bus.sv y sim_dl_bus.sh editados (originales en bak_sint/)")
