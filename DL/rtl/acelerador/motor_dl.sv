// ============================================================
// motor_dl.sv  -  motor integrado de la MLP 784-128-64-10 (SGD por imagen)
//
// UN SOLO banco de L lanes (decision D2: compartido) ejecuta las tres capas. Cada lane guarda,
// en la MISMA RAM, las filas  r = lane + L*pasada  de las TRES capas, con direcciones base
// distintas; los bancos por pasada (sesgo, dz, LFSR) tambien se concatenan por capa.
// Reusa la logica ya validada de: forward (motor_matvec), DELTA_OUT (delta_out), backward
// (motor_matvec_t, con DERIV fusionado) y UPDATE (motor_update), pero con las dimensiones de la
// capa elegidas en tiempo de ejecucion por el comando.
//
// COMANDOS (puerto cmd_*, un pulso de cmd_valid con el motor en reposo; 'done' se limpia al
// aceptar el comando y se activa al terminar):
//   op  nombre      lay   arg      accion
//   1   LOADX        -    imagen   lee la imagen (uint8) y su etiqueta por px_*/lb_*, pixel -> Q0.16
//   2   MATVEC      1..3   -       a_lay = sigmoide(W_lay * a_(lay-1) + b_lay)       (a_0 = x)
//   3   DELTA_OUT    -     -       dz3, e^2 y acierto (acumula en ep_e2 / ep_hits)
//   4   MATVEC_T    3 o 2  -       dz_(lay-1) = (W_lay^T dz_lay) .* a(1-a)  (DERIV fusionado)
//   5   UPDATE      1..3   -       W_lay, b_lay  -=  lr*dz_lay*a^T  (redondeo segun rmode)
//   6   CLRSTAT      -     -       pone a cero ep_e2, ep_hits y los contadores de ciclos
//   7   SETMODE      -    rmode    0 cercano, 1 estocastico, 2 piso
// Orden por imagen (el "CPU" es el testbench por ahora): LOADX; MATVEC 1,2,3; DELTA_OUT;
// MATVEC_T 3, 2; UPDATE 3, 2, 1. Los gradientes usan SIEMPRE los pesos previos: UPDATE solo
// va despues de todos los MATVEC_T (regla fundamental del SGD por imagen).
//
// Decision D1 (A): la salida de DELTA_OUT y de DERIV se escribe DIRECTO en el banco dz de la
// lane de la fila correspondiente; no hay memoria de dz central ni copia.
//
// Memoria de activaciones unica (1 lectura sincrona + 1 escritura): x[0..783], a1[784..911],
// a2[912..975], a3[976..985]. Cada operacion usa a lo sumo UNA lectura por ciclo.
//
// Carga inicial (host, motor en reposo) por el puerto ld_*: kind 0 = W[row][col] (24 b),
// 1 = b[row] (24 b), 2 = estado del LFSR de la fila (32 b). La LUT de la sigmoide se carga por
// $readmemh (LUT_FILE). La ruta de carga usa un multiplicador (solo para la carga) [VERIFICAR area].
//
// Contadores de ciclos por operacion (cyc_*): solo cuentan ciclos de reloj de ESTE diseno en
// SIMULACION; en la placa serviran para el perfilado (tiempo = ciclos / fclk).
//
// SystemVerilog (Icarus -g2012 y Quartus). Nada de esto esta sintetizado [VERIFICAR].
// ============================================================
module motor_dl #(
    parameter L        = 32,            // numero de lanes
    parameter FW       = 20,            // bits fraccionarios de W y b
    parameter LUT_FILE = "lut.hex",
    // ---- derivados: NO se sobreescriben desde fuera ----
    parameter P1  = (128 + L - 1) / L,                 // pasadas de la capa 1 (128 filas)
    parameter P2  = (64  + L - 1) / L,                 // capa 2 (64 filas)
    parameter P3  = (10  + L - 1) / L,                 // capa 3 (10 filas)
    parameter WB2 = P1 * 784,                          // base de W de la capa 2 en cada lane
    parameter WB3 = WB2 + P2 * 128,                    // base de W de la capa 3
    parameter WT  = WB3 + P3 * 64,                     // entradas de W por lane
    parameter AW  = $clog2(WT),
    parameter PT  = P1 + P2 + P3,                      // entradas de los bancos por pasada
    parameter BW  = (PT > 1) ? $clog2(PT) : 1,
    parameter S   = $clog2(L),                         // etapas del arbol sumador
    parameter LP  = 1 << $clog2(L),                    // hojas del arbol
    parameter LW  = $clog2(L) + 1,
    parameter CW  = $clog2(128 + L),
    parameter TW  = 54                                 // ancho del arbol: 48 + clog2(64)
) (
    input  logic                clk,
    input  logic                rst,          // sincrono, activo en alto

    // ---- puerto de comandos ----
    input  logic                cmd_valid,
    input  logic [3:0]          cmd_op,
    input  logic [1:0]          cmd_lay,
    input  logic [15:0]         cmd_arg,
    output logic                busy,
    output logic                done,         // nivel; se limpia al aceptar el siguiente comando

    // ---- carga inicial desde el host ----
    input  logic                ld_we,
    input  logic [1:0]          ld_kind,      // 0 W, 1 b, 2 LFSR
    input  logic [1:0]          ld_lay,       // 1..3
    input  logic [7:0]          ld_row,
    input  logic [9:0]          ld_col,
    input  logic [31:0]         ld_data,

    // ---- dataset (modelo de la SDRAM: lectura sincrona, dato 1 ciclo despues) ----
    output logic [25:0]         px_addr,      // imagen*784 + pixel
    input  logic [7:0]          px_data,
    output logic [15:0]         lb_addr,      // indice de imagen
    input  logic [7:0]          lb_data,

    // ---- estadisticas / perfilado ----
    output logic [47:0]         ep_e2,        // suma de e^2 (F32) desde CLRSTAT
    output logic [15:0]         ep_hits,      // aciertos desde CLRSTAT
    output logic [31:0]         cyc_ldx, cyc_fwd, cyc_del, cyc_bwd, cyc_upd,

    // ---- lectura del host (decision D3-A): SOLO con el motor en reposo (busy=0). El host presenta
    //      (kind, lay, row, col) y el dato aparece 2 ciclos despues del cambio de direccion
    //      (1 ciclo para las RAM sincronas + 1 de seleccion). kind: 0 W, 1 b, 2 LFSR, 3 dz ----
    input  logic [1:0]          hr_kind,
    input  logic [1:0]          hr_lay,
    input  logic [7:0]          hr_row,
    input  logic [9:0]          hr_col,
    output logic [31:0]         hr_data
);

    // ============================================================
    // Constantes por capa (funciones de 'lay')
    // ============================================================
    localparam [9:0] AB0 = 10'd0, AB1 = 10'd784, AB2 = 10'd912, AB3 = 10'd976;

    function automatic [9:0] f_ni(input [1:0] l);          // entradas
        case (l) 2'd1: f_ni = 10'd784; 2'd2: f_ni = 10'd128; default: f_ni = 10'd64; endcase
    endfunction
    function automatic [7:0] f_no(input [1:0] l);          // filas (neuronas)
        case (l) 2'd1: f_no = 8'd128; 2'd2: f_no = 8'd64; default: f_no = 8'd10; endcase
    endfunction
    function automatic [7:0] f_np(input [1:0] l);          // pasadas
        case (l) 2'd1: f_np = P1; 2'd2: f_np = P2; default: f_np = P3; endcase
    endfunction
    function automatic [AW-1:0] f_wb(input [1:0] l);       // base de W
        case (l) 2'd1: f_wb = 0; 2'd2: f_wb = WB2; default: f_wb = WB3; endcase
    endfunction
    function automatic [BW-1:0] f_pb(input [1:0] l);       // base de los bancos por pasada
        case (l) 2'd1: f_pb = 0; 2'd2: f_pb = P1; default: f_pb = P1 + P2; endcase
    endfunction
    function automatic [9:0] f_abin(input [1:0] l);        // activacion de ENTRADA de la capa
        case (l) 2'd1: f_abin = AB0; 2'd2: f_abin = AB1; default: f_abin = AB2; endcase
    endfunction
    function automatic [9:0] f_about(input [1:0] l);       // activacion de SALIDA de la capa
        case (l) 2'd1: f_about = AB1; 2'd2: f_about = AB2; default: f_about = AB3; endcase
    endfunction
    function automatic [15:0] f_bstep(input [1:0] l);      // (pasadas-1)*entradas
        case (l)
            2'd1: f_bstep = (P1 - 1) * 784;
            2'd2: f_bstep = (P2 - 1) * 128;
            default: f_bstep = (P3 - 1) * 64;
        endcase
    endfunction

    // Offset de fila en el banco de W de una lane: q * entradas(lay), con q = fila / L.
    // Se escribe con desplazamientos para NO inferir un multiplicador (784 = 512 + 256 + 16).
    function automatic [AW-1:0] f_rowoff(input [1:0] l, input [AW-1:0] q);
        case (l)
            2'd1:    f_rowoff = (q << 9) + (q << 8) + (q << 4);
            2'd2:    f_rowoff = q << 7;
            default: f_rowoff = q << 6;
        endcase
    endfunction

    // ============================================================
    // Estado y registros de control
    // ============================================================
    typedef enum logic [4:0] {
        S_IDLE,
        LX_LBL, LX_LB2, LX_RUN, LX_FIN,
        F_BIAS, F_RUN, F_DRAIN, F_SIG, F_FLUSH,
        D_START, D_W1, D_WAIT,
        B_RUN, B_WAIT,
        U_P0, U_P1, U_P2, U_STR, U_B0, U_B1, U_FIN
    } estado_t;
    estado_t st;

    localparam [3:0] OP_LOADX = 4'd1, OP_MATVEC = 4'd2, OP_DELTA = 4'd3, OP_MATVEC_T = 4'd4,
                     OP_UPDATE = 4'd5, OP_CLRSTAT = 4'd6, OP_SETMODE = 4'd7;

    logic [1:0]    lay;
    logic [15:0]   img;
    logic [3:0]    label;
    logic [1:0]    rmode;
    logic [7:0]    p;                // pasada actual
    logic [9:0]    k;                // columna actual
    logic [AW-1:0] raddr;            // direccion de W en cada lane (avanza sin parar entre pasadas)
    logic [CW-1:0] row_base;         // primera fila de la pasada (forward)
    logic [1:0]    dcnt, fcnt;
    logic [LW-1:0] sg;

    wire in_ldx = (st == LX_LBL) || (st == LX_LB2) || (st == LX_RUN) || (st == LX_FIN);
    wire in_fwd = (st == F_BIAS) || (st == F_RUN) || (st == F_DRAIN) || (st == F_SIG) || (st == F_FLUSH);
    wire in_del = (st == D_START) || (st == D_W1) || (st == D_WAIT);
    wire in_bwd = (st == B_RUN) || (st == B_WAIT);
    wire in_upd = (st == U_P0) || (st == U_P1) || (st == U_P2) || (st == U_STR) ||
                  (st == U_B0) || (st == U_B1) || (st == U_FIN);
    assign busy = (st != S_IDLE);

    // ============================================================
    // Memoria de activaciones: 1 lectura sincrona + 1 escritura
    // ============================================================
    logic [15:0] amem [0:985];
    logic [15:0] a_rd;                       // dato leido (valido 1 ciclo despues de ra)
    logic [9:0]  ra;                         // direccion de lectura
    logic [3:0]  d_rd_addr;                  // de delta_out
    logic [6:0]  e_k;                        // columna en la etapa E del backward

    always_comb begin
        if (in_del)      ra = AB3 + {6'b0, d_rd_addr};
        else if (in_bwd) ra = f_abin(lay) + {3'b0, e_k};
        else             ra = f_abin(lay) + k;          // forward y UPDATE: a[k] difundida
    end
    always_ff @(posedge clk) a_rd <= amem[ra];

    // ============================================================
    // LOADX: pixel uint8 -> Q0.16 = min(pix*257 + (pix>=128), 65535)   (== emulador, 256/256)
    // ============================================================
    wire [25:0] img_ext = {10'b0, img};
    wire [25:0] px_base = (img_ext << 9) + (img_ext << 8) + (img_ext << 4);     // img*784
    assign px_addr = px_base + {16'b0, k};
    assign lb_addr = img;

    logic         lx_v1;
    logic [9:0]   lx_k1;
    wire  [16:0]  qq  = {1'b0, px_data, 8'b0} + {9'b0, px_data} + {16'b0, px_data[7]};
    wire  [15:0]  q16 = qq[16] ? 16'hFFFF : qq[15:0];
    always_ff @(posedge clk) begin
        lx_v1 <= (st == LX_RUN);
        lx_k1 <= k;
    end

    // ============================================================
    // Forward: sigmoide compartida y captura de z
    // ============================================================
    logic v1, v2;                            // validez del MAC retrasada 1 y 2 ciclos
    always_ff @(posedge clk) begin v1 <= (st == F_RUN); v2 <= v1; end
    wire load_bias = (st == F_BIAS);

    wire  [L*16-1:0] z_lane;
    logic [L*16-1:0] zreg;
    wire  [CW-1:0]   row_cur = row_base + sg;
    wire             sig_in_valid = (st == F_SIG) && (row_cur < f_no(lay));
    wire signed [15:0] sig_z = zreg[sg*16 +: 16];
    wire             sig_out_valid;
    wire  [15:0]     sig_a;
    logic [CW-1:0]   rd1, rd2;               // la fila viaja junto al dato por las 2 etapas de la LUT

    sigmoide_lut #(.LUT_FILE(LUT_FILE)) u_sig (
        .clk(clk), .in_valid(sig_in_valid), .z(sig_z), .out_valid(sig_out_valid), .a(sig_a)
    );
    always_ff @(posedge clk) begin
        rd1 <= row_cur;
        rd2 <= rd1;
    end

    // ============================================================
    // DELTA_OUT
    // ============================================================
    wire               d_wr_en, d_done, d_hit;
    wire  [3:0]        d_wr_addr, d_pred;
    wire signed [23:0] d_wr_data;
    wire  [35:0]       d_e2;
    wire               d_start = (st == D_START);

    delta_out #(.N(10)) u_delta (
        .clk(clk), .rst(rst), .start(d_start), .label(label),
        .rd_addr(d_rd_addr), .rd_data(a_rd),
        .wr_en(d_wr_en), .wr_addr(d_wr_addr), .wr_data(d_wr_data),
        .done(d_done), .e2_sum(d_e2), .pred(d_pred), .hit(d_hit)
    );

    // ============================================================
    // Backward (MATVEC_T + DERIV): arbol sumador entre lanes, pasadas intercaladas por columna
    // ============================================================
    wire [9:0] b_ni    = f_ni(lay);
    wire       b_first = (p == 8'd0);
    wire       b_last  = (p == f_np(lay) - 8'd1);
    wire       rd_v_b  = (st == B_RUN);
    wire signed [47:0] prod_lane [0:L-1];

    wire signed [TW-1:0] t0 [0:LP-1];
    logic signed [TW-1:0] tr [0:(S > 0 ? S : 1)*LP - 1];
    genvar i, s, n;
    generate
        for (i = 0; i < LP; i = i + 1) begin : hojas
            if (i < L) begin : real_lane
                assign t0[i] = prod_lane[i];
            end else begin : relleno
                assign t0[i] = '0;
            end
        end
        for (s = 1; s <= S; s = s + 1) begin : niveles
            for (n = 0; n < (LP >> s); n = n + 1) begin : nodos
                if (s == 1) begin : desde_hojas
                    always_ff @(posedge clk) tr[(s-1)*LP + n] <= t0[2*n] + t0[2*n+1];
                end else begin : desde_nivel
                    always_ff @(posedge clk) tr[(s-1)*LP + n] <= tr[(s-2)*LP + 2*n] + tr[(s-2)*LP + 2*n+1];
                end
            end
        end
    endgenerate
    wire signed [TW-1:0] tree_out = (S == 0) ? t0[0] : tr[(S > 0 ? S-1 : 0)*LP];

    localparam D = 2 + S;                     // latencia issue -> salida del arbol
    logic [D-1:0]   bvpipe, bfpipe, blpipe;
    logic [D*7-1:0] bkpipe;
    always_ff @(posedge clk) begin
        bfpipe <= {bfpipe[D-2:0], b_first};
        blpipe <= {blpipe[D-2:0], b_last};
        bkpipe <= {bkpipe[(D-1)*7-1:0], k[6:0]};
        if (rst) bvpipe <= '0; else bvpipe <= {bvpipe[D-2:0], rd_v_b};
    end
    wire         c_v     = bvpipe[D-1];
    wire         c_first = bfpipe[D-1];
    wire         c_last  = blpipe[D-1];
    wire [6:0]   c_k     = bkpipe[(D-1)*7 +: 7];

    logic signed [TW-1:0] colacc, e_sum;
    logic                 e_v;
    wire  signed [TW-1:0] sum_now = (c_first ? '0 : colacc) + tree_out;
    always_ff @(posedge clk) begin
        if (c_v) colacc <= sum_now;
        e_sum <= sum_now;
        e_k   <= c_k;
        if (rst) e_v <= 1'b0; else e_v <= c_v && c_last;
    end

    // etapa 1: dA = sat24(rsh(acc,FW)) ; lectura de a_prev[k] por el puerto compartido
    localparam signed [TW-1:0] DA_MAX =  8388607;
    localparam signed [TW-1:0] DA_MIN = -8388608;
    wire signed [TW-1:0] bnd = e_sum + ({{(TW-1){1'b0}}, 1'b1} << (FW-1));
    wire signed [TW-1:0] shf = bnd >>> FW;
    wire signed [23:0]   da_c = (shf > DA_MAX) ? 24'sh7FFFFF : (shf < DA_MIN) ? 24'sh800000 : shf[23:0];
    logic signed [23:0] da_q1, da_q2;
    logic               bv1, bv2;
    logic [6:0]         bk1, bk2;
    always_ff @(posedge clk) begin
        da_q1 <= da_c;  bk1 <= e_k;
        if (rst) bv1 <= 1'b0; else bv1 <= e_v;
    end
    // etapa 2: s = rsh(a*(65536-a),16)  (a_rd ya contiene a_prev[e_k] de 1 ciclo antes)
    wire [32:0] pa   = a_rd * (17'd65536 - {1'b0, a_rd});
    wire [32:0] pa_r = pa + 33'd32768;
    logic [15:0] s_q;
    always_ff @(posedge clk) begin
        s_q   <= pa_r[31:16];
        da_q2 <= da_q1;  bk2 <= bk1;
        if (rst) bv2 <= 1'b0; else bv2 <= bv1;
    end
    // etapa 3: dz_prev = rsh(dA*s,16): 24 bits, sin saturador (|dz_prev| <= 2^21, demostrado)
    wire signed [40:0] p3   = da_q2 * $signed({1'b0, s_q});
    wire signed [40:0] p3_s = (p3 + 41'sd32768) >>> 16;
    wire               wr_last_b = bv2 && ({3'b0, bk2} == b_ni - 10'd1);

    // ============================================================
    // Escritura de dz en los bancos de las lanes (decision D1-A): DELTA_OUT o DERIV
    // ============================================================
    wire               dzw_en   = d_wr_en | bv2;
    wire  [7:0]        dzw_row  = d_wr_en ? {4'b0, d_wr_addr} : {1'b0, bk2};
    wire  [BW-1:0]     dzw_bank = d_wr_en ? (f_pb(2'd3) + (dzw_row / L))
                                          : (f_pb(lay - 2'd1) + (dzw_row / L));
    wire signed [23:0] dzw_data = d_wr_en ? d_wr_data : p3_s[23:0];

    // ============================================================
    // Control de las lanes
    // ============================================================
    wire [BW-1:0] psel   = f_pb(lay) + p[BW-1:0];
    wire [1:0]    op_sel = in_fwd ? 2'd0 : in_bwd ? 2'd1 : (st == U_P1) ? 2'd2 : 2'd3;
    wire          dzp_ld  = (st == U_P2);
    wire          lf_ld   = (st == U_P2);
    wire          u_issue = (st == U_STR);
    wire          u_bias  = (st == U_B1);

    // ---- carga del host ----
    wire [AW-1:0] ld_waddr = f_wb(ld_lay) + f_rowoff(ld_lay, ld_row / L) + ld_col;
    wire [BW-1:0] ld_paddr = f_pb(ld_lay) + (ld_row / L);

    // ---- lectura del host: con el motor en reposo las lanes reciben la direccion del host ----
    wire signed [23:0] rdw_l  [0:L-1];
    wire signed [23:0] rdb_l  [0:L-1];
    wire        [31:0] rdlf_l [0:L-1];
    wire signed [23:0] rddz_l [0:L-1];
    wire [AW-1:0] hr_waddr = f_wb(hr_lay) + f_rowoff(hr_lay, hr_row / L) + hr_col;
    wire [BW-1:0] hr_paddr = f_pb(hr_lay) + (hr_row / L);
    wire [AW-1:0] raddr_l  = busy ? raddr : hr_waddr;
    wire [BW-1:0] bsel_l   = busy ? psel  : hr_paddr;

    genvar g;
    generate
        for (g = 0; g < L; g = g + 1) begin : lanes
            localparam integer NPG1 = (g < 128) ? (128 - g + L - 1) / L : 0;
            localparam integer NPG2 = (g < 64)  ? (64  - g + L - 1) / L : 0;
            localparam integer NPG3 = (g < 10)  ? (10  - g + L - 1) / L : 0;
            wire [7:0] npg = (lay == 2'd1) ? NPG1 : (lay == 2'd2) ? NPG2 : NPG3;
            wire       rok = (p < npg);                       // la fila de esta lane existe en la pasada
            wire       mine = ((ld_row % L) == g);
            wire       we_d = dzw_en && ((dzw_row % L) == g);
            wire signed [15:0] z;

            lane_mac #(
                .ADDR_W(AW), .BIAS_AW(BW), .ACC_W(52), .SH(FW + 4)
            ) u_lane (
                .clk(clk),
                .w_we(ld_we && mine && ld_kind == 2'd0),  .w_waddr(ld_waddr), .w_wdata(ld_data[23:0]),
                .b_we(ld_we && mine && ld_kind == 2'd1),  .b_waddr(ld_paddr), .b_wdata(ld_data[23:0]),
                .dz_we(we_d),                             .dz_waddr(dzw_bank), .dz_wdata(dzw_data),
                .lf_we(ld_we && mine && ld_kind == 2'd2), .lf_waddr(ld_paddr), .lf_wdata(ld_data),
                .raddr(raddr_l), .bsel(bsel_l), .dsel(bsel_l),
                .op_sel(op_sel), .row_ok(rok),
                .load_bias(load_bias), .mac_en(v2), .a_q(a_rd),
                .rmode(rmode), .dzp_ld(dzp_ld), .lf_ld(lf_ld), .u_issue(u_issue), .u_bias(u_bias),
                .z_out(z), .prod_out(prod_lane[g]),
                .rd_w(rdw_l[g]), .rd_b(rdb_l[g]), .rd_lf(rdlf_l[g]), .rd_dz(rddz_l[g])
            );
            assign z_lane[g*16 +: 16] = z;
        end
    endgenerate

    // ---- seleccion del dato leido por el host (todas las fuentes con 1 ciclo de latencia) ----
    logic [LW-1:0]      hr_lane_q;
    logic [1:0]         hr_kind_q;
    logic signed [23:0] hr_bias_q;
    always_ff @(posedge clk) begin
        hr_lane_q <= hr_row % L;
        hr_kind_q <= hr_kind;
        hr_bias_q <= rdb_l[hr_row % L];          // el sesgo se lee de forma asincrona: se registra aqui
    end
    wire signed [23:0] hr_w_sel  = rdw_l[hr_lane_q];
    wire signed [23:0] hr_dz_sel = rddz_l[hr_lane_q];
    wire        [31:0] hr_lf_sel = rdlf_l[hr_lane_q];
    assign hr_data = (hr_kind_q == 2'd0) ? {{8{hr_w_sel[23]}}, hr_w_sel} :
                     (hr_kind_q == 2'd1) ? {{8{hr_bias_q[23]}}, hr_bias_q} :
                     (hr_kind_q == 2'd2) ? hr_lf_sel :
                                           {{8{hr_dz_sel[23]}}, hr_dz_sel};

    // ============================================================
    // Escritura de la memoria de activaciones (LOADX o sigmoide; nunca a la vez)
    // ============================================================
    always_ff @(posedge clk) begin
        if (lx_v1)              amem[lx_k1]                    <= q16;
        else if (sig_out_valid) amem[f_about(lay) + rd2]       <= sig_a;
    end

    // ============================================================
    // FSM principal
    // ============================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE;  done <= 1'b0;
            ep_e2 <= 0;  ep_hits <= 0;
            cyc_ldx <= 0; cyc_fwd <= 0; cyc_del <= 0; cyc_bwd <= 0; cyc_upd <= 0;
            rmode <= 2'd0;
        end else begin
            case (st)
                S_IDLE: if (cmd_valid) begin
                    done <= 1'b0;  lay <= cmd_lay;  img <= cmd_arg;
                    p <= 0;  k <= 0;  row_base <= 0;  raddr <= f_wb(cmd_lay);
                    case (cmd_op)
                        OP_LOADX:    st <= LX_LBL;
                        OP_MATVEC:   st <= F_BIAS;
                        OP_DELTA:    st <= D_START;
                        OP_MATVEC_T: st <= B_RUN;
                        OP_UPDATE:   st <= U_P0;
                        OP_CLRSTAT:  begin ep_e2 <= 0; ep_hits <= 0;
                                           cyc_ldx <= 0; cyc_fwd <= 0; cyc_del <= 0; cyc_bwd <= 0; cyc_upd <= 0;
                                           done <= 1'b1; end
                        OP_SETMODE:  begin rmode <= cmd_arg[1:0]; done <= 1'b1; end
                        default:     done <= 1'b1;
                    endcase
                end

                // ---------------- LOADX ----------------
                LX_LBL: st <= LX_LB2;                          // se presenta lb_addr
                LX_LB2: begin label <= lb_data[3:0]; st <= LX_RUN; k <= 0; end
                LX_RUN: begin
                    if (k == 10'd783) st <= LX_FIN; else k <= k + 1'b1;
                end
                LX_FIN: begin st <= S_IDLE; done <= 1'b1; end   // la ultima escritura cae en este mismo flanco

                // ---------------- forward ----------------
                F_BIAS:  begin st <= F_RUN; k <= 0; end
                F_RUN: begin
                    raddr <= raddr + 1'b1;
                    if (k == f_ni(lay) - 10'd1) begin st <= F_DRAIN; dcnt <= 0; end
                    else k <= k + 1'b1;
                end
                F_DRAIN: begin
                    dcnt <= dcnt + 1'b1;
                    if (dcnt == 2'd2) begin zreg <= z_lane; st <= F_SIG; sg <= 0; end
                end
                F_SIG: begin
                    sg <= sg + 1'b1;
                    if (sg == L - 1) begin
                        if (p == f_np(lay) - 8'd1) begin st <= F_FLUSH; fcnt <= 0; end
                        else begin p <= p + 1'b1; row_base <= row_base + L; st <= F_BIAS; end
                    end
                end
                F_FLUSH: begin
                    fcnt <= fcnt + 1'b1;
                    if (fcnt == 2'd1) begin st <= S_IDLE; done <= 1'b1; end
                end

                // ---------------- DELTA_OUT ----------------
                D_START: st <= D_W1;
                // D_W1 es un ciclo de MARGEN: delta_out limpia 'done' en el mismo flanco en que acepta
                // 'start', asi que ya seria seguro mirar d_done en D_WAIT (mutante M11 de mut_dl.py: equivalente).
                D_W1:    st <= D_WAIT;
                D_WAIT:  if (d_done) begin
                    ep_e2   <= ep_e2 + {12'b0, d_e2};
                    ep_hits <= ep_hits + {15'b0, d_hit};
                    st <= S_IDLE;  done <= 1'b1;
                end

                // ---------------- backward ----------------
                B_RUN: begin
                    if (!b_last) begin
                        p <= p + 1'b1;
                        raddr <= raddr + b_ni;
                    end else begin
                        p <= 0;
                        raddr <= raddr - f_bstep(lay) + 1'b1;
                        if (k == b_ni - 10'd1) st <= B_WAIT; else k <= k + 1'b1;
                    end
                end
                B_WAIT: ;                                       // termina con wr_last_b

                // ---------------- UPDATE ----------------
                U_P0: st <= U_P1;
                U_P1: st <= U_P2;
                U_P2: begin st <= U_STR; k <= 0; end
                U_STR: begin
                    raddr <= raddr + 1'b1;
                    if (k == f_ni(lay) - 10'd1) st <= U_B0; else k <= k + 1'b1;
                end
                U_B0: st <= U_B1;
                U_B1: if (p == f_np(lay) - 8'd1) st <= U_FIN;
                      else begin p <= p + 1'b1; st <= U_P0; end
                U_FIN: begin st <= S_IDLE; done <= 1'b1; end

                default: st <= S_IDLE;
            endcase

            if (wr_last_b) begin st <= S_IDLE; done <= 1'b1; end    // ultima columna del DERIV escrita

            // contadores de ciclos por operacion
            if (in_ldx) cyc_ldx <= cyc_ldx + 1'b1;
            if (in_fwd) cyc_fwd <= cyc_fwd + 1'b1;
            if (in_del) cyc_del <= cyc_del + 1'b1;
            if (in_bwd) cyc_bwd <= cyc_bwd + 1'b1;
            if (in_upd) cyc_upd <= cyc_upd + 1'b1;
        end
    end

endmodule
