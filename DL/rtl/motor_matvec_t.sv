// ============================================================
// motor_matvec_t.sv
//
// MATVEC_T + DERIV de UNA capa, con L lanes y arbol sumador entre lanes
// (alternativa A1 del diseno, con pasadas INTERCALADAS por columna).
// Replica matvec_t_deriv() de emulador.cpp, bit a bit, para cualquier L:
//
//   acc[k]     = sum_j W[j][k] * dz[j]               (enteros, sin saturar)
//   dA[k]      = sat24( rsh(acc[k], FW) )            (Q2.22)
//   s[k]       = rsh( a[k] * (65536 - a[k]), 16 )    (a(1-a), <= 16384)
//   dz_prev[k] = rsh( dA[k] * s[k], 16 )             (Q2.22)
//   rsh(v,s) = (v + 2^(s-1)) >>> s   (desplazamiento aritmetico)
//
// W es N_OUT x N_IN (filas x columnas); dz tiene N_OUT valores; a y los
// resultados tienen N_IN valores. (Capa 3->2: N_OUT=10, N_IN=64.
// Capa 2->1: N_OUT=64, N_IN=128.)  W NO se transpone: en cada ciclo cada
// lane lee la columna k de SU fila, igual que en el forward.
//
// Reparto de filas (igual que motor_matvec): fila r -> lane r mod L, pasada r / L.
//
// Orden de emision (intercalado, recomendado): para cada columna k se emiten
// las PASSES pasadas seguidas (k,p0),(k,p1),...; el arbol suma las L lanes de
// cada pasada y un acumulador suma las PASSES salidas del arbol antes de
// emitir un dA[k]. No hay buffer de sumas parciales.
//
// Lanes sin fila (capa con N_OUT < L, o ultima pasada incompleta): su
// producto es EXACTAMENTE 0 (row_ok=0 en la lane), nunca basura.
//
// Segundo saturador (el de dz_prev): NO se implementa. Cota demostrada:
// |dA| <= 2^23, s <= 16384 = 2^14  =>  |dA*s| <= 2^37  =>  |dz_prev| <= 2^21 < 2^23,
// asi que nunca satura (mutante equivalente verificado con el emulador).
// El saturador de dA SI es imprescindible (acc llega a ~2^52).
//
// Ciclos (SIMULACION, contados por tb_matvec_t desde el start hasta 'done'):
//     ciclos = N_IN * PASSES + S + 5      (S = $clog2(L) etapas del arbol)
// verificado con L = 1, 7 y 32 en las dos capas (p. ej. L=32: 74 y 266). NO es tiempo real.
// 'done' se activa en el MISMO flanco en que se escribe la ultima columna, asi que
// done visible => todos los resultados visibles.
//
// Interfaces de carga/lectura: las usara despues el bus; por ahora las maneja el
// testbench. Lectura de resultados ASINCRONA (solo testbench; para sintesis habra
// que pasarla a sincrona [VERIFICAR]).
//
// SystemVerilog (Icarus -g2012 y Quartus).
// ============================================================
module motor_matvec_t #(
    parameter L     = 32,            // numero de lanes
    parameter N_IN  = 128,           // columnas de W = salidas del backward
    parameter N_OUT = 64,            // filas de W = longitud de dz
    parameter FW    = 20,            // bits fraccionarios de W
    // ---- derivados: NO se sobreescriben desde fuera ----
    parameter PASSES = (N_OUT + L - 1) / L,               // pasadas
    parameter KW     = $clog2(N_IN),                      // bits de columna
    parameter RW     = $clog2(N_OUT),                     // bits de fila
    parameter BW     = (PASSES > 1) ? $clog2(PASSES) : 1, // bits de pasada
    parameter AW     = $clog2(PASSES * N_IN),             // direccion de W por lane
    parameter S      = $clog2(L),                         // etapas del arbol
    parameter LP     = 1 << $clog2(L),                    // hojas del arbol (potencia de 2)
    // Ancho del acumulador/arbol: N_OUT productos de hasta 2^46 -> 46 + clog2(N_OUT) + 2 bits
    // (= 54 para N_OUT=64, justo el peor caso medido con el caso 2 de --test-backward).
    parameter TW     = 48 + $clog2(N_OUT)
) (
    input  logic                clk,
    input  logic                rst,        // sincrono, activo en alto
    input  logic                start,      // pulso de 1 ciclo (solo en reposo)
    output logic                done,       // nivel; se limpia con el siguiente start

    // carga de pesos W[fila][col]
    input  logic                w_we,
    input  logic [RW-1:0]       w_row,
    input  logic [KW-1:0]       w_col,
    input  logic signed [23:0]  w_data,
    // carga de dz (Q2.22, 24 bits) por fila
    input  logic                dz_we,
    input  logic [RW-1:0]       dz_row,
    input  logic signed [23:0]  dz_data,
    // carga de a_prev (Q0.16) por columna
    input  logic                a_we,
    input  logic [KW-1:0]       a_addr,
    input  logic [15:0]         a_data,

    // lectura de resultados (asincrona, para el testbench)
    input  logic [KW-1:0]       y_addr,
    output logic signed [23:0]  da_data,    // dA[k]      (Q2.22)
    output logic signed [23:0]  dzp_data    // dz_prev[k] (Q2.22)
);

    // ============================================================
    // FSM de emision: una (columna, pasada) por ciclo
    // ============================================================
    typedef enum logic [1:0] {S_IDLE, S_RUN, S_WAIT} estado_t;
    estado_t       st;
    logic [KW-1:0] k;          // columna
    logic [BW-1:0] p;          // pasada
    logic [AW-1:0] raddr;      // direccion de W en cada lane = p*N_IN + k

    wire rd_v     = (st == S_RUN);
    wire is_first = (p == 0);
    wire is_last  = (p == PASSES - 1);
    logic          wr_last;    // se esta escribiendo la ultima columna (ver abajo)

    always_ff @(posedge clk) begin
        if (rst) begin
            st   <= S_IDLE;
            done <= 1'b0;
        end else begin
            case (st)
                S_IDLE: if (start) begin
                    st    <= S_RUN;
                    done  <= 1'b0;
                    k     <= 0;
                    p     <= 0;
                    raddr <= 0;
                end
                S_RUN: begin
                    if (!is_last) begin              // siguiente pasada de la misma columna
                        p     <= p + 1'b1;
                        raddr <= raddr + N_IN;
                    end else begin                   // siguiente columna, pasada 0
                        p     <= 0;
                        raddr <= raddr - (PASSES - 1) * N_IN + 1;
                        if (k == N_IN - 1) st <= S_WAIT;
                        else               k  <= k + 1'b1;
                    end
                end
                default: ;
            endcase
            if (wr_last) begin                       // ultima columna escrita
                done <= 1'b1;
                st   <= S_IDLE;
            end
        end
    end

    // ============================================================
    // Lanes
    // ============================================================
    wire signed [47:0] prod_lane [0:L-1];
    wire [AW-1:0] w_addr_lane = (w_row / L) * N_IN + w_col;
    wire [BW-1:0] d_pass      = dz_row / L;

    generate
        for (genvar g = 0; g < L; g = g + 1) begin : lanes
            // numero de pasadas en las que la lane g tiene fila (fila = pasada*L + g < N_OUT)
            localparam integer NPG = (g < N_OUT) ? (N_OUT - g + L - 1) / L : 0;
            wire we_w = w_we  && ((w_row  % L) == g);
            wire we_d = dz_we && ((dz_row % L) == g);

            lane_mac #(
                .ADDR_W(AW), .BIAS_AW(BW), .ACC_W(52), .SH(FW + 4)
            ) u_lane (
                .clk(clk),
                .w_we(we_w),   .w_waddr(w_addr_lane), .w_wdata(w_data),
                .b_we(1'b0),   .b_waddr('0),          .b_wdata(24'sd0),   // sin uso (forward)
                .dz_we(we_d),  .dz_waddr(d_pass),     .dz_wdata(dz_data),
                .raddr(raddr), .bsel('0),             .dsel(p),
                .bw(1'b1),     .row_ok(p < NPG),
                .load_bias(1'b0), .mac_en(1'b0),
                .a_q(16'd0),
                .z_out(),      .prod_out(prod_lane[g])
            );
        end
    endgenerate

    // ============================================================
    // Arbol sumador segmentado (1 registro por nivel)
    //   nivel 0 = productos de las lanes (ya registrados en la lane), rellenado
    //   con ceros hasta LP hojas; nivel s = suma de pares del nivel s-1.
    // ============================================================
    wire signed [TW-1:0] t0 [0:LP-1];
    logic signed [TW-1:0] tr [0:(S > 0 ? S : 1)*LP - 1];   // nivel s en [(s-1)*LP + i]

    generate
        for (genvar i = 0; i < LP; i = i + 1) begin : hojas
            if (i < L) begin : real_lane
                assign t0[i] = prod_lane[i];               // signed 48 -> TW: extiende signo
            end else begin : relleno
                assign t0[i] = '0;
            end
        end
        for (genvar s = 1; s <= S; s = s + 1) begin : niveles
            for (genvar i = 0; i < (LP >> s); i = i + 1) begin : nodos
                if (s == 1) begin : desde_hojas
                    always_ff @(posedge clk) tr[(s-1)*LP + i] <= t0[2*i] + t0[2*i+1];
                end else begin : desde_nivel
                    always_ff @(posedge clk) tr[(s-1)*LP + i] <= tr[(s-2)*LP + 2*i] + tr[(s-2)*LP + 2*i+1];
                end
            end
        end
    endgenerate

    wire signed [TW-1:0] tree_out = (S == 0) ? t0[0] : tr[(S > 0 ? S-1 : 0)*LP];

    // ============================================================
    // Tuberia de control: acompana a cada (columna, pasada) desde el issue
    // hasta la salida del arbol (D = 2 + S ciclos)
    // ============================================================
    localparam D = 2 + S;
    logic [D-1:0] vpipe;                    // validez (con reset)
    logic [D-1:0] fpipe, lpipe;             // primera / ultima pasada
    logic [D*KW-1:0] kpipe;                 // columna (D campos de KW bits)

    // registros de desplazamiento: la etapa 0 es la mas reciente
    always_ff @(posedge clk) begin
        fpipe <= {fpipe[D-2:0], is_first};
        lpipe <= {lpipe[D-2:0], is_last};
        kpipe <= {kpipe[(D-1)*KW-1:0], k};
        if (rst) vpipe <= '0;
        else     vpipe <= {vpipe[D-2:0], rd_v};
    end

    wire           c_v     = vpipe[D-1];
    wire           c_first = fpipe[D-1];
    wire           c_last  = lpipe[D-1];
    wire [KW-1:0]  c_k     = kpipe[(D-1)*KW +: KW];

    // ============================================================
    // Acumulacion entre pasadas (la salida del arbol de la pasada 0 inicia la suma)
    // ============================================================
    logic signed [TW-1:0] colacc;
    logic signed [TW-1:0] e_sum;            // suma completa de la columna (etapa E)
    logic                 e_v;
    logic [KW-1:0]        e_k;
    wire  signed [TW-1:0] sum_now = (c_first ? '0 : colacc) + tree_out;

    always_ff @(posedge clk) begin
        if (c_v) colacc <= sum_now;
        e_sum <= sum_now;
        e_k   <= c_k;
        if (rst) e_v <= 1'b0;
        else     e_v <= c_v && c_last;
    end

    // ============================================================
    // Post-proceso: dA = sat24(rsh(acc,FW)) ; s = a(1-a) ; dz_prev = rsh(dA*s,16)
    // ============================================================
    logic [15:0]          amem [0:N_IN-1];
    always_ff @(posedge clk) if (a_we) amem[a_addr] <= a_data;

    // etapa 1 (visible en e+1): dA saturado y lectura de a[k]
    localparam signed [TW-1:0] DA_MAX =  8388607;     //  2^23 - 1
    localparam signed [TW-1:0] DA_MIN = -8388608;     // -2^23
    wire signed [TW-1:0] rnd = e_sum + ({{(TW-1){1'b0}}, 1'b1} << (FW-1));
    wire signed [TW-1:0] shf = rnd >>> FW;
    wire signed [23:0]   da_c = (shf > DA_MAX) ? 24'sh7FFFFF :
                                (shf < DA_MIN) ? 24'sh800000 :
                                                 shf[23:0];
    logic signed [23:0] da_q1;
    logic [15:0]        a_rd;
    logic               v1;
    logic [KW-1:0]      k1;
    always_ff @(posedge clk) begin
        da_q1 <= da_c;
        a_rd  <= amem[e_k];                              // lectura sincrona, alineada con da_q1
        k1    <= e_k;
        if (rst) v1 <= 1'b0; else v1 <= e_v;
    end

    // etapa 2 (visible en e+2): s = rsh(a*(65536-a),16)
    wire [32:0] pa  = a_rd * (17'd65536 - {1'b0, a_rd});          // 16 x 17 bits, sin signo
    wire [32:0] pa_r = pa + 33'd32768;
    logic signed [23:0] da_q2;
    logic [15:0]        s_q;
    logic               v2;
    logic [KW-1:0]      k2;
    always_ff @(posedge clk) begin
        s_q   <= pa_r[31:16];                            // rsh 16 (resultado <= 16384)
        da_q2 <= da_q1;
        k2    <= k1;
        if (rst) v2 <= 1'b0; else v2 <= v1;
    end

    // etapa 3 (escritura): dz_prev = rsh(dA * s, 16)
    wire signed [40:0] p3   = da_q2 * $signed({1'b0, s_q});      // 24 x 17 con signo
    wire signed [40:0] p3_r = p3 + 41'sd32768;
    wire signed [40:0] p3_s = p3_r >>> 16;

    logic signed [23:0] damem  [0:N_IN-1];
    logic signed [23:0] dzpmem [0:N_IN-1];
    always_ff @(posedge clk) begin
        if (v2) begin
            damem[k2]  <= da_q2;
            dzpmem[k2] <= p3_s[23:0];
        end
    end
    assign wr_last = v2 && (k2 == N_IN - 1);

    assign da_data  = damem[y_addr];
    assign dzp_data = dzpmem[y_addr];

endmodule