import subprocess, shutil, os, sys, time
# Pruebas de mutacion del RTL de MATVEC_T+DERIV. Uso (desde sim/, con sb_c0..3_s1 generados por
# gen_vectores_backward.sh):  python3 ../scripts/mut_backward.py [R1 R2 ...]   (sin args: todos)
# Cada mutante introduce UN error en una copia del RTL y corre sim_backward.sh; "SOBREVIVE" = ningun caso lo detecta.
# R14 es un mutante EQUIVALENTE (segundo saturador, logica muerta): debe sobrevivir.
BASE=os.path.abspath("..")   # se ejecuta desde sim/
M="motor_matvec_t.sv"; LN="lane_mac.sv"
MUT={
 "R1  lane sin fila NO se anula (lane_mac)":(LN,"prod <= ok_q ? (w_q * op2) : 48'sd0;","prod <= (w_q * op2);"),
 "R19 row_ok=1 fijo (motor)":(M,".row_ok(p < NPG)",".row_ok(1'b1)"),
 "R9  mascara p<=NPG (off-by-one)":(M,".row_ok(p < NPG)",".row_ok(p <= NPG)"),
 "R2  arbol con una hoja de menos":(M,"assign t0[i] = prod_lane[i];","assign t0[i] = (i == L-1) ? '0 : prod_lane[i];"),
 "R13 hojas de relleno != 0":(M,"assign t0[i] = '0;","assign t0[i] = 1;"),
 "R3  se descarta la acumulacion entre pasadas":(M,"(c_first ? '0 : colacc)","'0"),
 "R8  dsel fijo en 0 (dz de pasada 0 siempre)":(M,".dsel(p)",".dsel('0)"),
 "R4  fw mal en el rsh de dA (FW-1)":(M,"wire signed [TW-1:0] shf = rnd >>> FW;","wire signed [TW-1:0] shf = rnd >>> (FW-1);"),
 "R5  sin saturador de dA":(M,"""    wire signed [23:0]   da_c = (shf > DA_MAX) ? 24'sh7FFFFF :
                                (shf < DA_MIN) ? 24'sh800000 :
                                                 shf[23:0];""","    wire signed [23:0]   da_c = shf[23:0];"),
 "R6  s sin redondeo":(M,"wire [32:0] pa_r = pa + 33'd32768;","wire [32:0] pa_r = pa;"),
 "R7  acumulador de 53 bits (falta 1 bit)":(M,"parameter TW     = 48 + $clog2(N_OUT)","parameter TW     = 47 + $clog2(N_OUT)"),
 "R10 a[k] desalineado (1 ciclo tarde)":(M,"a_rd  <= amem[e_k];","a_rd  <= amem[k1];"),
 "R11 done un ciclo antes":(M,"assign wr_last = v2 && (k2 == N_IN - 1);","assign wr_last = v1 && (k1 == N_IN - 1);"),
 "R15 tuberia de control 1 etapa corta":(M,"localparam D = 2 + S;","localparam D = 1 + S;"),
 "R17 raddr no avanza al cambiar de columna":(M,"raddr <= raddr - (PASSES - 1) * N_IN + 1;","raddr <= raddr - (PASSES - 1) * N_IN;"),
 "R12 multiplicador sin signo (op2)":(LN,"wire signed [23:0] op2    = bw ? dz_q : a_s;","wire        [23:0] op2    = bw ? dz_q : a_s;"),
 "R14 [EQUIVALENTE] 2o saturador en dz_prev":(M,"dzpmem[k2] <= p3_s[23:0];","dzpmem[k2] <= (p3_s > 8388607) ? 24'sh7FFFFF : (p3_s < -8388608) ? 24'sh800000 : p3_s[23:0];"),
}
sel=sys.argv[1:] 
LS=os.environ.get("LS","1 7 10 32 48 64")
for nombre,(f,a,b) in MUT.items():
    if sel and not any(nombre.startswith(x+" ") for x in sel): continue
    d=f"{BASE}/sim/mut_rtl_tmp"; shutil.rmtree(d,ignore_errors=True); shutil.copytree(f"{BASE}/rtl",d)
    s=open(f"{d}/{f}").read(); assert s.count(a)==1,(nombre,a); open(f"{d}/{f}","w").write(s.replace(a,b))
    det=[]; t0=time.time()
    for caso in range(4):
        r=subprocess.run(f"cd {BASE}/sim && RTL={d} sh ../scripts/sim_backward.sh sb_c{caso}_s1 {LS}",shell=True,capture_output=True,text=True)
        if r.returncode!=0: det.append(caso)
    print(f"{nombre:46s} -> {'detectado por casos '+str(det) if det else 'SOBREVIVE'}   ({time.time()-t0:.0f}s)",flush=True)