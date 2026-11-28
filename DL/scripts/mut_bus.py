# mut_bus.py - pruebas de mutacion del periferico de bus (dl_avalon.sv) y de la ruta de lectura del host
# (mux de direccion y seleccion de dato en motor_dl.sv).
# Uso (desde sim/, con los vectores m2 = ../src/emulador fixed 1 --n=2 ... --vectores=m2):
#   python3 ../scripts/mut_bus.py [B1 B2 ... H1 ...]       (sin argumentos: todos)    env LS="32"
# Un error de COMPILACION no cuenta como deteccion. "SOBREVIVE" = ninguna configuracion lo detecta.
import subprocess, shutil, os, sys, time, signal
BASE = os.path.abspath("..")
A, M = "dl_avalon.sv", "motor_dl.sv"
MUT = {
 "B1  opcode no se comprueba":(A,"avs_writedata[6:0] != 7'b0001011 ||","1'b0 ||"),
 "B2  funct7 > 15 no se comprueba":(A,"|| avs_writedata[31:29] != 3'b000","|| 1'b0"),
 "B3  busy sin el ciclo de comando en vuelo":(A,"wire         busy_eff = m_busy | pend;","wire         busy_eff = m_busy;"),
 "B4  done sin enmascarar el comando en vuelo":(A,"wire         done_eff = m_done & ~pend;","wire         done_eff = m_done;"),
 "B5  CMD aceptado con el motor ocupado":(A,"                        if (busy_eff) err <= 1'b1;\n                        else if (avs_writedata[6:0]","                        if (1'b0) err <= 1'b1;\n                        else if (avs_writedata[6:0]"),
 "B6  W1C no limpia err":(A,"A_STATUS: if (avs_writedata[2]) err <= 1'b0;","A_STATUS: ;"),
 "B7  LD_DATA aceptado con el motor ocupado":(A,"if (busy_eff || ld_addr[21:20] == 2'd3) err <= 1'b1;","if (ld_addr[21:20] == 2'd3) err <= 1'b1;"),
 "B8  LD_DATA acepta kind 3 (solo lectura)":(A,"if (busy_eff || ld_addr[21:20] == 2'd3) err <= 1'b1;","if (busy_eff) err <= 1'b1;"),
 "B9  autoincremento de W: ultima columna mal":(A,"if (col == f_ni(lay) - 10'd1) begin","if (col == f_ni(lay)) begin"),
 "B10 kinds 1..3 no autoincrementan":(A,"end else row = row + 8'd1;","end else row = row;"),
 "B11 RD_DATA sin waitrequest":(A,"assign avs_waitrequest = avs_read && rd_sel && !rd_ok && !busy_eff;","assign avs_waitrequest = 1'b0;"),
 "B12 ID erroneo":(A,"localparam [31:0] ID_VALUE = 32'h444C_0100;","localparam [31:0] ID_VALUE = 32'h444C_0101;"),
 "B13 CFG sin FW":(A,"A_CFG:      avs_readdata <= {8'b0, FW[7:0], L[15:0]};","A_CFG:      avs_readdata <= {16'b0, L[15:0]};"),
 "B14 ARG no se latchea al escribir CMD":(A,"cmd_arg_r <= arg_reg;","cmd_arg_r <= 16'd0;"),
 "B15 funct3 (capa) tomado de otros bits":(A,"cmd_lay_r <= avs_writedata[13:12];","cmd_lay_r <= avs_writedata[14:13];"),
 "B16 EP_E2_HI con los bits equivocados":(A,"avs_readdata <= {16'b0, ep_e2[47:32]};","avs_readdata <= {16'b0, ep_e2[31:16]};"),
 "B17 CLRSTAT no limpia CYC_BUSY":(A,"if (avs_writedata[28:25] == 4'd6) cyc_busy <= 32'd0;","if (1'b0) cyc_busy <= 32'd0;"),
 "B18 RD_DATA ocupado no devuelve BAD0BAD0":(A,"avs_readdata <= busy_eff ? 32'hBAD0_BAD0 : hr_data;","avs_readdata <= hr_data;"),
 "B19 comando sin validar la capa/operacion":(A,"!cmd_valido(avs_writedata[28:25], avs_writedata[14:12])","1'b0"),
 "B20 operaciones 0 y 8..15 aceptadas":(A,"default:                cmd_valido = 1'b0;","default:                cmd_valido = 1'b1;"),
 "B21 MATVEC_T acepta la capa 1":(A,"4'd4:                   cmd_valido = (f3 == 3'd2 || f3 == 3'd3);","4'd4:                   cmd_valido = (f3 >= 3'd1 && f3 <= 3'd3);"),
 "B22 MATVEC/UPDATE aceptan la capa 0":(A,"4'd2, 4'd5:             cmd_valido = (f3 >= 3'd1 && f3 <= 3'd3);","4'd2, 4'd5:             cmd_valido = (f3 <= 3'd3);"),
 "B23 funct3 con bit 2 ignorado":(A,"avs_writedata[28:25], avs_writedata[14:12])","avs_writedata[28:25], {1'b0, avs_writedata[13:12]})"),
 "H1  ventana: W siempre con la direccion del datapath":(M,"wire [AW-1:0] raddr_l  = busy ? raddr : hr_waddr;","wire [AW-1:0] raddr_l  = raddr;"),
 "H2  ventana: banco siempre con la pasada del datapath":(M,"wire [BW-1:0] bsel_l   = busy ? psel  : hr_paddr;","wire [BW-1:0] bsel_l   = psel;"),
 "H3  ventana: kind 2 (LFSR) devuelve dz":(M,"(hr_kind_q == 2'd2) ? hr_lf_sel :","(hr_kind_q == 2'd2) ? {{8{hr_dz_sel[23]}}, hr_dz_sel} :"),
 "H4  ventana: sesgo siempre de la lane 0":(M,"hr_bias_q <= rdb_l[hr_row % L];","hr_bias_q <= rdb_l[0];"),
 "H5  ventana: lane de seleccion siempre 0":(M,"hr_lane_q <= hr_row % L;","hr_lane_q <= 0;"),
}
sel = sys.argv[1:]
LS = os.environ.get("LS", "32")
for nombre, (f, a, b) in MUT.items():
    if sel and not any(nombre.startswith(x + " ") for x in sel): continue
    d = f"{BASE}/sim/mut_bus_tmp"; shutil.rmtree(d, ignore_errors=True); shutil.copytree(f"{BASE}/rtl/acelerador", d)
    s = open(f"{d}/{f}").read(); assert s.count(a) == 1, (nombre, a, s.count(a))
    open(f"{d}/{f}", "w").write(s.replace(a, b))
    t0 = time.time()
    # limite de tiempo por mutante (env TO, def. 240 s): un mutante que deja al motor sin terminar es
    # DETECTADO por timeout, pero no debe bloquear toda la corrida
    p = subprocess.Popen(f"RTL={d} sh ../scripts/sim_dl_bus.sh m2 2 1 {LS}", shell=True, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        txt, _ = p.communicate(timeout=int(os.environ.get("TO", "240"))); rc = p.returncode; tout = False
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); txt, _ = p.communicate(); rc = -9; tout = True
    if tout: res = "detectado (timeout: el motor no termina)"
    elif "fallo la compilacion" in txt: res = "ERROR DE COMPILACION (mutante invalido)"
    elif rc != 0:
        fallas = [l.strip() for l in txt.splitlines() if l.startswith("BUS: FALLA")]
        extra = (" | " + fallas[0][11:]) if fallas else ""
        res = "detectado" + extra
    else: res = "SOBREVIVE"
    print(f"{nombre:54s} -> {res}  ({time.time()-t0:.0f}s)", flush=True)
shutil.rmtree(f"{BASE}/sim/mut_bus_tmp", ignore_errors=True)