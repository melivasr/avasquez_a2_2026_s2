// ============================================================
// dl_avalon.sv  -  periferico Avalon-MM del motor DL (esclavo de 32 bits, direcciones de PALABRA)
//
// Es la cara del motor hacia el bus (Platform Designer): lo manejan por igual el CPU RV32I y el
// maestro JTAG-to-Avalon (decision D1-A). Por ahora lo maneja el testbench.
//
// MAPA DE REGISTROS (direccion de palabra; byte = 4*palabra)        acceso
//   0x00 ID        32'h444C_0100 ("DL", v1.0)                          RO
//   0x01 CMD       palabra de comando (ver abajo); dispara la operacion  WO
//   0x02 ARG       argumento de 16 bits (indice de imagen, o rmode)      RW
//   0x03 STATUS    [0] busy  [1] done  [2] err (sticky; escribir 1 a bit 2 lo limpia)   RO/W1C
//   0x04 CYC_BUSY  ciclos con el motor ocupado desde el ultimo CLRSTAT   RO
//   0x05..0x09 CYC_LDX, CYC_FWD, CYC_DEL, CYC_BWD, CYC_UPD  ciclos por operacion         RO
//   0x0A EP_E2_LO / 0x0B EP_E2_HI (16 b) / 0x0C EP_HITS   suma de e^2 (F32) y aciertos   RO
//   0x0D CFG       {8'b0, FW[7:0], L[15:0]}                              RO
//   0x10 LD_ADDR   {kind[1:0], lay[1:0], row[7:0], col[9:0]} (22 bits)   RW
//   0x11 LD_DATA   escribir carga una palabra en LD_ADDR y AUTOINCREMENTA  WO
//   0x12 RD_ADDR   igual formato                                          RW
//   0x13 RD_DATA   leer devuelve la palabra en RD_ADDR y AUTOINCREMENTA   RO
//   kind: 0 = W[row][col] (24 b con signo) ; 1 = b[row] ; 2 = estado del LFSR de la fila (32 b) ;
//         3 = dz[row] (solo lectura). Autoincremento: kind 0 -> col, y al llegar a la ultima
//         columna pasa a la fila siguiente; kinds 1..3 -> fila. lay = 1..3.
// PALABRA CMD (formato tipo custom-0 de RISC-V, 32 bits):
//   [31:25] funct7 = operacion (1 LOADX, 2 MATVEC, 3 DELTA_OUT, 4 MATVEC_T, 5 UPDATE, 6 CLRSTAT,
//                   7 SETMODE)   [14:12] funct3 = capa   [6:0] opcode = 7'b0001011   resto = 0
//   El argumento sale del registro ARG en el momento de escribir CMD.
// ERRORES (STATUS.err, sin efecto sobre la operacion en curso): CMD con opcode distinto, con
//   funct7 > 15, con operacion 0 u 8..15, o con capa invalida para la operacion (ver cmd_valido); CMD con el motor ocupado; LD_DATA con el motor ocupado o con kind 3; RD_DATA con el
//   motor ocupado (devuelve 32'hBAD0BAD0).
// BUS: readLatency = 1 (readdata valido el ciclo siguiente a la aceptacion); waitrequest SOLO en
//   RD_DATA mientras el dato no esta listo (2 ciclos tras cada cambio de RD_ADDR). Las escrituras
//   nunca esperan (1 por ciclo).
// STATUS.busy incluye el ciclo de "comando en vuelo" para que el host no vea busy=0 y done=1 viejo
//   justo despues de escribir CMD.
// SIMULACION: nada de esto esta sintetizado [VERIFICAR].
// ============================================================
module dl_avalon #(
    parameter L        = 32,
    parameter INIT_LANES = 0,       // Precarga W, b y LFSR desde archivos por lane.
    parameter FW       = 20,
    parameter LUT_FILE = "lut.hex"
) (
    input  logic        clk,
    input  logic        rst,                 // sincrono, activo en alto

    // ---- esclavo Avalon-MM ----
    input  logic [4:0]  avs_address,
    input  logic        avs_read,
    input  logic        avs_write,
    input  logic [31:0] avs_writedata,
    output logic [31:0] avs_readdata,
    output logic        avs_waitrequest,

    // ---- dataset (hacia el adaptador de ROM M10K / SDRAM) ----
    output logic [25:0] px_addr,
    input  logic [7:0]  px_data,
    output logic [15:0] lb_addr,
    input  logic [7:0]  lb_data
);
    localparam [4:0] A_ID = 5'h00, A_CMD = 5'h01, A_ARG = 5'h02, A_STATUS = 5'h03, A_CYC_BUSY = 5'h04,
                     A_CYC_LDX = 5'h05, A_CYC_FWD = 5'h06, A_CYC_DEL = 5'h07, A_CYC_BWD = 5'h08,
                     A_CYC_UPD = 5'h09, A_E2LO = 5'h0A, A_E2HI = 5'h0B, A_HITS = 5'h0C, A_CFG = 5'h0D,
                     A_LDADDR = 5'h10, A_LDDATA = 5'h11, A_RDADDR = 5'h12, A_RDDATA = 5'h13;
    localparam [31:0] ID_VALUE = 32'h444C_0100;

    // ---------------- motor ----------------
    logic        cmd_valid_r;
    logic [3:0]  cmd_op_r;
    logic [1:0]  cmd_lay_r;
    logic [15:0] cmd_arg_r;
    logic        ld_we_q;
    logic [1:0]  ld_kind_q, ld_lay_q;
    logic [7:0]  ld_row_q;
    logic [9:0]  ld_col_q;
    logic [31:0] ld_data_q;
    wire         m_busy, m_done;
    wire  [47:0] ep_e2;
    wire  [15:0] ep_hits;
    wire  [31:0] cyc_ldx, cyc_fwd, cyc_del, cyc_bwd, cyc_upd;
    logic [21:0] rd_addr, ld_addr;
    wire  [31:0] hr_data;

    motor_dl #(.L(L), .INIT_LANES(INIT_LANES), .FW(FW), .LUT_FILE(LUT_FILE)) u_motor (
        .clk(clk), .rst(rst),
        .cmd_valid(cmd_valid_r), .cmd_op(cmd_op_r), .cmd_lay(cmd_lay_r), .cmd_arg(cmd_arg_r),
        .busy(m_busy), .done(m_done),
        .ld_we(ld_we_q), .ld_kind(ld_kind_q), .ld_lay(ld_lay_q), .ld_row(ld_row_q),
        .ld_col(ld_col_q), .ld_data(ld_data_q),
        .px_addr(px_addr), .px_data(px_data), .lb_addr(lb_addr), .lb_data(lb_data),
        .ep_e2(ep_e2), .ep_hits(ep_hits),
        .cyc_ldx(cyc_ldx), .cyc_fwd(cyc_fwd), .cyc_del(cyc_del), .cyc_bwd(cyc_bwd), .cyc_upd(cyc_upd),
        .hr_kind(rd_addr[21:20]), .hr_lay(rd_addr[19:18]), .hr_row(rd_addr[17:10]), .hr_col(rd_addr[9:0]),
        .hr_data(hr_data)
    );

    // ---------------- estado del periferico ----------------
    logic        pend;                 // comando aceptado y aun no visto por el motor
    logic        err;
    logic [15:0] arg_reg;
    logic [31:0] cmd_word;
    logic [31:0] cyc_busy;
    logic        rd_ok, rd_v1;         // dato de RD_DATA listo (2 ciclos tras cambiar RD_ADDR)
    wire         busy_eff = m_busy | pend;
    wire         done_eff = m_done & ~pend;

    // numero de columnas por capa (para el autoincremento)
    function automatic [9:0] f_ni(input [1:0] l);
        case (l) 2'd1: f_ni = 10'd784; 2'd2: f_ni = 10'd128; default: f_ni = 10'd64; endcase
    endfunction

    // Un comando mal formado NO debe llegar al motor: con una capa invalida (p. ej. MATVEC con capa 0) el
    // motor no termina nunca y solo un reset lo libera. Validos: op 1,3,6,7 con funct3 = 0 (sin capa);
    // op 2 (MATVEC) y 5 (UPDATE) con capa 1..3; op 4 (MATVEC_T) con capa 2..3 (la capa 1 no tiene dA0).
    function automatic cmd_valido(input [3:0] op, input [2:0] f3);
        case (op)
            4'd1, 4'd3, 4'd6, 4'd7: cmd_valido = (f3 == 3'd0);
            4'd2, 4'd5:             cmd_valido = (f3 >= 3'd1 && f3 <= 3'd3);
            4'd4:                   cmd_valido = (f3 == 3'd2 || f3 == 3'd3);
            default:                cmd_valido = 1'b0;
        endcase
    endfunction

    wire rd_sel  = (avs_address == A_RDDATA);
    assign avs_waitrequest = avs_read && rd_sel && !rd_ok && !busy_eff;
    wire rd_acc  = avs_read && rd_sel && !avs_waitrequest;
    wire rd_upd  = (avs_write && avs_address == A_RDADDR) || (rd_acc && !busy_eff);

    // siguiente direccion tras una carga o lectura
    function automatic [21:0] f_inc(input [21:0] a);
        reg [1:0] kind, lay; reg [7:0] row; reg [9:0] col;
        begin
            kind = a[21:20]; lay = a[19:18]; row = a[17:10]; col = a[9:0];
            if (kind == 2'd0) begin
                if (col == f_ni(lay) - 10'd1) begin col = 10'd0; row = row + 8'd1; end
                else col = col + 10'd1;
            end else row = row + 8'd1;
            f_inc = {kind, lay, row, col};
        end
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            cmd_valid_r <= 1'b0;  pend <= 1'b0;  err <= 1'b0;  ld_we_q <= 1'b0;
            arg_reg <= 16'd0;  cmd_word <= 32'd0;  cyc_busy <= 32'd0;
            ld_addr <= 22'd0;  rd_addr <= 22'd0;  rd_ok <= 1'b1;  rd_v1 <= 1'b0;
        end else begin
            cmd_valid_r <= 1'b0;  pend <= 1'b0;  ld_we_q <= 1'b0;
            if (busy_eff) cyc_busy <= cyc_busy + 32'd1;

            // ---- RD_DATA listo 2 ciclos despues de cada cambio de RD_ADDR ----
            if (rd_upd) begin rd_ok <= 1'b0; rd_v1 <= 1'b1; end
            else begin rd_v1 <= 1'b0; if (rd_v1) rd_ok <= 1'b1; end

            if (avs_write) begin
                case (avs_address)
                    A_CMD: begin
                        cmd_word <= avs_writedata;
                        if (busy_eff) err <= 1'b1;
                        else if (avs_writedata[6:0] != 7'b0001011 || avs_writedata[31:29] != 3'b000 ||
                                 !cmd_valido(avs_writedata[28:25], avs_writedata[14:12])) err <= 1'b1;
                        else begin
                            cmd_valid_r <= 1'b1;  pend <= 1'b1;
                            cmd_op_r  <= avs_writedata[28:25];
                            cmd_lay_r <= avs_writedata[13:12];
                            cmd_arg_r <= arg_reg;
                            if (avs_writedata[28:25] == 4'd6) cyc_busy <= 32'd0;       // CLRSTAT
                        end
                    end
                    A_ARG:    arg_reg <= avs_writedata[15:0];
                    A_STATUS: if (avs_writedata[2]) err <= 1'b0;                        // W1C
                    A_LDADDR: ld_addr <= avs_writedata[21:0];
                    A_LDDATA: begin
                        if (busy_eff || ld_addr[21:20] == 2'd3) err <= 1'b1;
                        else begin
                            ld_we_q <= 1'b1;
                            ld_kind_q <= ld_addr[21:20];  ld_lay_q <= ld_addr[19:18];
                            ld_row_q  <= ld_addr[17:10];  ld_col_q <= ld_addr[9:0];
                            ld_data_q <= avs_writedata;
                            ld_addr   <= f_inc(ld_addr);
                        end
                    end
                    A_RDADDR: rd_addr <= avs_writedata[21:0];
                    default: ;
                endcase
            end
            if (rd_acc) begin
                if (busy_eff) err <= 1'b1;
                else          rd_addr <= f_inc(rd_addr);
            end
        end
    end

    // ---------------- lectura de registros (readLatency = 1) ----------------
    always_ff @(posedge clk) begin
        if (avs_read && !avs_waitrequest) begin
            case (avs_address)
                A_ID:       avs_readdata <= ID_VALUE;
                A_CMD:      avs_readdata <= cmd_word;
                A_ARG:      avs_readdata <= {16'b0, arg_reg};
                A_STATUS:   avs_readdata <= {29'b0, err, done_eff, busy_eff};
                A_CYC_BUSY: avs_readdata <= cyc_busy;
                A_CYC_LDX:  avs_readdata <= cyc_ldx;
                A_CYC_FWD:  avs_readdata <= cyc_fwd;
                A_CYC_DEL:  avs_readdata <= cyc_del;
                A_CYC_BWD:  avs_readdata <= cyc_bwd;
                A_CYC_UPD:  avs_readdata <= cyc_upd;
                A_E2LO:     avs_readdata <= ep_e2[31:0];
                A_E2HI:     avs_readdata <= {16'b0, ep_e2[47:32]};
                A_HITS:     avs_readdata <= {16'b0, ep_hits};
                A_CFG:      avs_readdata <= {8'b0, FW[7:0], L[15:0]};
                A_LDADDR:   avs_readdata <= {10'b0, ld_addr};
                A_RDADDR:   avs_readdata <= {10'b0, rd_addr};
                A_RDDATA:   avs_readdata <= busy_eff ? 32'hBAD0_BAD0 : hr_data;
                default:    avs_readdata <= 32'd0;
            endcase
        end
    end
endmodule
