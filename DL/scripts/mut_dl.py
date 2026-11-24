# mut_dl.py - pruebas de mutacion del motor integrado (motor_dl.sv): los errores propios de la
# INTEGRACION (bases por capa, enrutado de dz, LOADX, secuenciacion, estadisticas). Los modulos
# internos (lane, arbol, UPDATE...) ya tienen sus propias mutaciones (mut_backward.py, mut_update.py).
# Uso (desde sim/, con los vectores m2 = ../src/emulador fixed 1 --n=2 ... --vectores=m2):
#   python3 ../scripts/mut_dl.py [M1 M2 ...]        (sin argumentos: todos)   env LS="7 32 64"
# Cada mutante introduce UN error en una copia del RTL y corre sim_dl.sh. Un error de COMPILACION no
# cuenta como deteccion (se informa aparte). "SOBREVIVE" = ninguna configuracion lo detecta.
# M11 es EQUIVALENTE (D_W1 es un ciclo de margen): debe sobrevivir.
import subprocess, shutil, os, sys, time
BASE = os.path.abspath("..")
F = "motor_dl.sv"
MUT = {
 "M1  base de W de la capa 3 desplazada":("parameter WB3 = WB2 + P2 * 128,","parameter WB3 = WB2 + P2 * 127,"),
 "M2  banco por pasada de la capa 3 = el de la capa 1":("default: f_pb = P1 + P2;","default: f_pb = P1;"),
 "M3  DERIV escribe dz en el banco de la propia capa":(": (f_pb(lay - 2'd1) + (dzw_row / L));",": (f_pb(lay) + (dzw_row / L));"),
 "M4  DELTA_OUT escribe dz3 en el banco de la capa 2":("d_wr_en ? (f_pb(2'd3) + (dzw_row / L))","d_wr_en ? (f_pb(2'd2) + (dzw_row / L))"),
 "M5  LOADX sin el termino de redondeo (pix>=128)":("+ {16'b0, px_data[7]};","+ 17'd0;"),
 "M6  entrada de la capa 2 leida de a2 en vez de a1":("2'd2: f_abin = AB1;","2'd2: f_abin = AB2;"),
 "M7  salto de raddr del backward sin el -1":("f_bstep = (f_np(l) - 8'd1) * f_ni(l);","f_bstep = f_np(l) * f_ni(l);"),
 "M8  lane 63 sin fila en la capa 2":("localparam integer NPG2 = (g < 64)  ?","localparam integer NPG2 = (g < 63)  ?"),
 "M9  etiqueta no se captura":("label <= lb_data[3:0];","label <= 4'd0;"),
 "M10 a_prev del DERIV desalineada":("else if (in_bwd) ra = f_abin(lay) + {3'b0, e_k};","else if (in_bwd) ra = f_abin(lay) + {3'b0, bk1};"),
 "M11 [EQUIVALENTE] DELTA_OUT: sin el ciclo de margen D_W1":("D_START: st <= D_W1;","D_START: st <= D_WAIT;"),
 "M12 raddr inicial con la capa anterior":("raddr <= f_wb(cmd_lay);","raddr <= f_wb(lay);"),
 "M13 fila del DERIV desalineada":("{1'b0, bk2};","{1'b0, bk1};"),
 "M14 aciertos: ep_hits suma siempre 1":("ep_hits <= ep_hits + {15'b0, d_hit};","ep_hits <= ep_hits + 16'd1;"),
 "M15 forward: row_base avanza L-1":("row_base <= row_base + L;","row_base <= row_base + L - 1;"),
 "M16 paso del sesgo un estado antes":("wire          u_bias  = (st == U_B1);","wire          u_bias  = (st == U_B0);"),
}
sel = sys.argv[1:]
LS = os.environ.get("LS", "7 32 64")
for nombre, (a, b) in MUT.items():
    if sel and not any(nombre.startswith(x + " ") for x in sel): continue
    d = f"{BASE}/sim/mut_dl_tmp"; shutil.rmtree(d, ignore_errors=True); shutil.copytree(f"{BASE}/rtl", d)
    s = open(f"{d}/{F}").read(); assert s.count(a) == 1, (nombre, a, s.count(a))
    open(f"{d}/{F}", "w").write(s.replace(a, b))
    t0 = time.time()
    r = subprocess.run(f"RTL={d} sh ../scripts/sim_dl.sh m2 2 1 {LS}", shell=True, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if "fallo la compilacion" in txt: res = "ERROR DE COMPILACION (mutante invalido)"
    elif r.returncode != 0:
        cuales = [l.split()[1] for l in txt.splitlines() if l.startswith("--- L=")]
        malas = []
        bloques = txt.split("--- L=")[1:]
        for bl in bloques:
            if "PASS" not in bl: malas.append(bl.split()[0])
        res = "detectado en L = " + " ".join(malas)
    else: res = "SOBREVIVE"
    print(f"{nombre:52s} -> {res}  ({time.time()-t0:.0f}s)", flush=True)
shutil.rmtree(f"{BASE}/sim/mut_dl_tmp", ignore_errors=True)