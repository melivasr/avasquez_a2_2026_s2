# mut_update.py - pruebas de mutacion del RTL de UPDATE.
# Uso (desde sim/, con los vectores su_<modo>_c<caso>_s1 de gen_vectores_update.sh):
#   python3 ../scripts/mut_update.py [U1 U2 ...]        (sin argumentos: todos)
# Cada mutante introduce UN error en una copia del RTL y corre sim_update.sh sobre
# 12 juegos (3 redondeos x 4 casos, semilla 1) con L = 7 y 32. Se imprime en cuales se detecta,
# con la clave  c=cercano e=estocastico p=piso  seguida del caso (p. ej. e1 = estocastico, caso 1).
# "SOBREVIVE" = ningun juego lo detecta. Los marcados [EQUIVALENTE] deben sobrevivir: su efecto
# queda fuera de las filas y columnas que existen, o se pierde en un truncamiento.
import subprocess, shutil, os, sys, time
BASE = os.path.abspath("..")                 # se ejecuta desde sim/
LN, MU = "lane_mac.sv", "motor_update.sv"
SAT_W = """    assign nw = (w_sub >  25'sd8388607) ? 24'sh7FFFFF :
                (w_sub < -25'sd8388608) ? 24'sh800000 :
                                          w_sub[23:0];"""
SAT_B = """    assign bnew  = (b_sub >  33'sd8388607) ? 24'sh7FFFFF :
                   (b_sub < -33'sd8388608) ? 24'sh800000 :
                                             b_sub[23:0];"""
SAT_Z = """    wire signed [26:0] dzp_c = (lr_s >  54'sd67108863) ? 27'sh3FFFFFF :     //  2^26 - 1
                               (lr_s < -54'sd67108864) ? 27'sh4000000 :     // -2^26
                                                         lr_s[26:0];"""
MUT = {
 "U1  R no se suma en 'cercano' (= piso)":(LN,"(rmode == 2'd0) ? (32'h1 << (SHU - 1)) : 32'h0;","(rmode == 2'd0) ? 32'h0 : 32'h0;"),
 "U2  R de empate 2^25-1 en pesos":(LN,"(rmode == 2'd0) ? (32'h1 << (SHU - 1)) : 32'h0;","(rmode == 2'd0) ? ((32'h1 << (SHU - 1)) - 32'h1) : 32'h0;"),
 "U3  polinomio del LFSR 0x80200001":(LN,"32'h80200003","32'h80200001"),
 "U4  LFSR no avanza en el paso del sesgo":(LN,"assign lf_nb = stoch ? lf_next : lf;","assign lf_nb = lf;"),
 "U5  estado LFSR no se devuelve al banco":(LN,"else if (b_upd)  lfmem[dsel]     <= lf_nb;","else if (1'b0)   lfmem[dsel]     <= lf_nb;"),
 "U6  banco LFSR leido siempre en la entrada 0":(LN,"lf_rd <= lfmem[dsel];","lf_rd <= lfmem['0];"),
 "U7  sin saturacion de W":(LN,SAT_W,"    assign nw = w_sub[23:0];"),
 "U8  sin saturacion de b":(LN,SAT_B,"    assign bnew = b_sub[23:0];"),
 "U9  sin saturacion de dZ' (27 b)":(LN,SAT_Z,"    wire signed [26:0] dzp_c = lr_s[26:0];"),
 "U10 dZ' sin redondeo (sin +128)":(LN,"wire signed [53:0] lr_s = (prod + 54'sd128) >>> 8;","wire signed [53:0] lr_s = prod >>> 8;"),
 "U11 delta truncado a 16 bits":(LN,"delta_q <= dsh[23:0];","delta_q <= {{8{dsh[15]}}, dsh[15:0]};"),
 "U12 escritura de W con direccion 1 etapa antes":(LN,"else if (uv3)    wmem[wa3]     <= nw;","else if (uv3)    wmem[wa2]     <= nw;"),
 "U13 Rb con mascara de SHB+1 bits":(LN,"(lf_next & ((32'h1 << SHB) - 32'h1))","(lf_next & ((32'h1 << (SHB+1)) - 32'h1))"),
 "U14 db con desplazamiento SHB+1":(LN,"wire signed [31:0] db     = db_sum >>> SHB;","wire signed [31:0] db     = db_sum >>> (SHB+1);"),
 "U15 Rb de empate 2^9-1 en el sesgo":(LN,"(rmode == 2'd0) ? (32'h1 << (SHB - 1)) : 32'h0;","(rmode == 2'd0) ? ((32'h1 << (SHB - 1)) - 32'h1) : 32'h0;"),
 "U16 estocastico = rmode 2 (mapeo cambiado)":(LN,"wire  stoch = (rmode == 2'd1);","wire  stoch = (rmode == 2'd2);"),
 "U17 LFSR avanza tambien en cercano/piso":(LN,"else if (uv1 && stoch)   lf <= lf_next;","else if (uv1)            lf <= lf_next;"),
 "U18 dZ' solo se carga en la pasada 0":(MU,"wire       dzp_ld  = (st == S_P2);","wire       dzp_ld  = (st == S_P2) && (p == 0);"),
 "U19 done un ciclo antes":(MU,"S_B1:  if (p == PASSES - 1) st <= S_FIN;","S_B1:  if (p == PASSES - 1) begin st <= S_IDLE; done <= 1'b1; end"),
 "U20 dz*lr no se calcula (op_sel siempre UST)":(MU,"wire [1:0] op_sel  = (st == S_P1) ? 2'd2 : 2'd3;","wire [1:0] op_sel  = 2'd3;"),
 "U21 a[k] desalineada (sin registro)":(MU,".a_q(x_q),",".a_q(amem[k]),"),
 "U22 pasada fija en 0 (dz y LFSR)":(MU,".raddr(raddr), .bsel(p),              .dsel(p),",".raddr(raddr), .bsel(p),              .dsel('0),"),
 "U23 [EQUIVALENTE] valido sin row_ok":(LN,"uv1 <= u_issue & row_ok;","uv1 <= u_issue;"),
 "U24 [EQUIVALENTE] sesgo sin row_ok":(LN,"wire                      b_upd = u_bias & row_ok;","wire                      b_upd = u_bias;"),
}
sel = sys.argv[1:]
JUEGOS = [(m, c) for m in ("cercano", "estocastico", "piso") for c in range(4)]
LS = os.environ.get("LS", "7 32")
for nombre, (f, a, b) in MUT.items():
    if sel and not any(nombre.startswith(x + " ") for x in sel): continue
    d = f"{BASE}/sim/mut_upd_tmp"; shutil.rmtree(d, ignore_errors=True); shutil.copytree(f"{BASE}/rtl/acelerador", d)
    s = open(f"{d}/{f}").read(); assert s.count(a) == 1, (nombre, a, s.count(a))
    open(f"{d}/{f}", "w").write(s.replace(a, b))
    det = []; t0 = time.time()
    for m, c in JUEGOS:
        r = subprocess.run(f"RTL={d} sh ../scripts/sim_update.sh su_{m}_c{c}_s1 {LS}", shell=True, capture_output=True, text=True)
        if r.returncode != 0: det.append(f"{m[0]}{c}")
    print(f"{nombre:48s} -> {'detectado en '+str(len(det))+'/12: '+' '.join(det) if det else 'SOBREVIVE'}  ({time.time()-t0:.0f}s)", flush=True)
shutil.rmtree(f"{BASE}/sim/mut_upd_tmp", ignore_errors=True)