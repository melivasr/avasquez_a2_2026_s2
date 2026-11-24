// ============================================================
// lane_mac.sv   (v3: forward + backward + UPDATE)
//
// Una lane del motor vectorial: atiende UNA fila de W a la vez.
// Contiene:
//   - RAM de pesos de sus filas (24 bits con signo, Q4.20), con escritura de vuelta (UPDATE)
//   - memoria de sesgos (una entrada por pasada), con escritura de vuelta
//   - banco de dz (una entrada por pasada)
//   - banco de estados del LFSR de sus filas (32 bits, una entrada por pasada)   [v3]
//   - MULTIPLICADOR COMPARTIDO 27x27 con signo (un DSP en modo 27x27 [VERIFICAR en el
//     manual de la Cyclone V]); sus dos operandos se eligen con op_sel:
//        op_sel  nombre   operando 1   operando 2        uso
//          0     FWD      W (24 b)     a  (Q0.16)        forward:  W*a
//          1     BWD      W (24 b)     dz (Q2.22)        backward: W*dz
//          2     ULR      dz (24 b)    6554 (= lr, F16)  UPDATE, por fila: dz*lr
//          3     UST      dZ' (27 b)   a  (Q0.16)        UPDATE, por columna: dZ'*a
//   - acumulador de ACC_W bits y conversion a z (Q4.12) para el forward
//
// FORWARD (op_sel=0), replica MATVEC de matvec_sigmoide() de emulador.cpp:
//     acc = (b << 16) + sum_k W[j][k] * a[k]          (F = FW+16)
//     z   = sat16( (acc + 2^(SH-1)) >>> SH ),  SH = FW+4   -> Q4.12
// BACKWARD (op_sel=1): la lane NO acumula; entrega en prod_out el producto W*dz (48 bits)
//     y el motor suma ENTRE LANES. Fila inexistente (row_ok=0) -> prod = EXACTAMENTE 0.
// UPDATE, replica actualizar() de emulador.cpp, con la fila j de la lane:
//     dZ'      = sat27( rsh(dz*6554, 8) )                   (1 vez por fila; ULR)
//     W[j][k] <- sat24( W[j][k] - ((dZ'*a[k] + R) >>> SHU) ),  SHU = 46 - FW   (k en orden)
//     b[j]    <- sat24( b[j]    - ((dZ'      + Rb) >>> SHB) ), SHB = 30 - FW   (tras la ultima columna)
//   R / Rb segun rmode:  0 = al mas cercano (2^(s-1)) ; 1 = ESTOCASTICO (bits bajos del
//   LFSR tras un paso por peso y un paso mas para el sesgo) ; 2 = piso (0).
//   LFSR de Galois de 32 bits, mascara 0x80200003; un estado por FILA (banco por lane, opcion A).
//   Requiere SHU < 32 (FW >= 15) en modo estocastico: el LFSR tiene 32 bits.
//
// Segmentacion de UPDATE por columna (elemento emitido en el ciclo t):
//   t   : raddr presentado (u_issue=1); el padre lee a[k]
//   t+1 : w_q, a_q validos -> producto dZ'*a ; se avanza el LFSR y se prepara R
//   t+2 : prod valido -> delta = (prod + R) >>> SHU
//   t+3 : W - delta, saturar y ESCRIBIR de vuelta en wmem  (el valido viaja con el elemento)
// Segmentacion del forward/backward: raddr/dsel/a_q alineados en t ->
//   t+1: w_q, dz_q, ok_q ; t+2: prod valido (prod_out) ; forward: mac_en=1 suma en t+2.
//
// Nota sobre el acumulador forward: el emulador acumula en 64 bits y satura a 48 ANTES de
// convertir a z. Aqui ACC_W=52 nunca desborda y se omite esa saturacion: es equivalente,
// porque si |acc| >= 2^47 el resultado z ya satura a +-8 en ambos casos.
//
// SystemVerilog (Icarus -g2012 y Quartus).
// ============================================================
module lane_mac #(
    parameter ADDR_W  = 12,   // bits de direccion de la RAM de pesos
    parameter BIAS_AW = 2,    // bits de direccion de las memorias por pasada (sesgo, dz, LFSR)
    parameter ACC_W   = 52,   // ancho del acumulador forward (con signo)
    parameter SH      = 24    // FW + 4 : desplazamiento acc -> Q4.12 (tambien fija FW de UPDATE)
) (
    input                          clk,

    // ---- carga desde el host / testbench (solo con el motor en reposo) ----
    input                          w_we,
    input      [ADDR_W-1:0]        w_waddr,
    input  signed [23:0]           w_wdata,
    input                          b_we,
    input      [BIAS_AW-1:0]       b_waddr,
    input  signed [23:0]           b_wdata,
    input                          dz_we,
    input      [BIAS_AW-1:0]       dz_waddr,
    input  signed [23:0]           dz_wdata,
    input                          lf_we,        // estado del LFSR de la fila (pasada lf_waddr)
    input      [BIAS_AW-1:0]       lf_waddr,
    input      [31:0]              lf_wdata,

    // ---- control desde el motor ----
    input      [ADDR_W-1:0]        raddr,      // direccion de lectura de W
    input      [BIAS_AW-1:0]       bsel,       // pasada actual (sesgo)
    input      [BIAS_AW-1:0]       dsel,       // pasada actual (dz, LFSR)
    input      [1:0]               op_sel,     // 0 FWD, 1 BWD, 2 ULR, 3 UST (nivel; ver arriba)
    input                          row_ok,     // 1 si la fila de esta lane existe en la pasada
    input                          load_bias,  // forward: acc <= b << 16
    input                          mac_en,     // forward: acc <= acc + prod
    input      [15:0]              a_q,        // activacion (Q0.16), difundida

    // ---- control de UPDATE ----
    input      [1:0]               rmode,      // 0 cercano, 1 estocastico, 2 piso (nivel)
    input                          dzp_ld,     // dzp_r <= sat27(rsh(prod,8))   (prod = dz*lr)
    input                          lf_ld,      // lf <= estado leido del banco (pasada dsel)
    input                          u_issue,    // este ciclo se emite un elemento (columna) de UPDATE
    input                          u_bias,     // paso del sesgo: b, y estado del LFSR de vuelta al banco

    // ---- resultados ----
    output signed [15:0]           z_out,      // forward: Q4.12, saturado a +-8
    output signed [47:0]           prod_out,   // backward: W*dz (0 si row_ok=0), valido en t+2

    // ---- lectura para el host (ventana de pesos): solo reflejan los puertos de lectura ya
    //      existentes; el motor les presenta la direccion del host cuando esta en reposo ----
    output signed [23:0]           rd_w,       // W[raddr]    (1 ciclo despues de raddr)
    output signed [23:0]           rd_b,       // b[bsel]     (combinacional)
    output        [31:0]           rd_lf,      // LFSR[dsel]  (1 ciclo despues de dsel)
    output signed [23:0]           rd_dz       // dz[dsel]    (1 ciclo despues de dsel)
);

    localparam FWL = SH - 4;          // bits fraccionarios de W
    localparam SHU = 46 - FWL;        // dZ'(F30) * a(F16) = F46 -> F fw
    localparam SHB = 30 - FWL;        // dZ'(F30)          -> F fw
    localparam signed [26:0] LR_Q = 27'sd6554;   // lr = 0.1 en F16 (igual que el emulador)

    // ---------------- memorias ----------------
    logic signed [23:0] wmem  [0:(1<<ADDR_W)-1];      // pesos (lectura sincrona)
    logic signed [23:0] bmem  [0:(1<<BIAS_AW)-1];     // sesgos (lectura asincrona, pocas entradas)
    logic signed [23:0] dzmem [0:(1<<BIAS_AW)-1];     // dz por pasada (lectura sincrona)
    logic        [31:0] lfmem [0:(1<<BIAS_AW)-1];     // estados LFSR por pasada (lectura sincrona)

    // ---- senales de UPDATE declaradas aqui porque las usan las escrituras ----
    logic                     uv1, uv2, uv3;           // validez (fila existe) del elemento en cada etapa
    logic [ADDR_W-1:0]        wa1, wa2, wa3;           // direccion de escritura que viaja con el elemento
    logic signed [23:0]       nw;                      // peso nuevo (etapa t+3)
    logic signed [23:0]       bnew;                    // sesgo nuevo
    logic [31:0]              lf_nb;                   // estado LFSR tras el paso del sesgo
    wire                      b_upd = u_bias & row_ok;

    // La escritura del host tiene prioridad; solo ocurre con el motor en reposo.
    always_ff @(posedge clk) begin
        if (w_we)        wmem[w_waddr] <= w_wdata;
        else if (uv3)    wmem[wa3]     <= nw;
        if (b_we)        bmem[b_waddr] <= b_wdata;
        else if (b_upd)  bmem[bsel]    <= bnew;
        if (dz_we)       dzmem[dz_waddr] <= dz_wdata;
        if (lf_we)       lfmem[lf_waddr] <= lf_wdata;
        else if (b_upd)  lfmem[dsel]     <= lf_nb;
    end

    // ---------------- multiplicador compartido 27x27 ----------------
    logic signed [23:0]       w_q;
    logic signed [23:0]       dz_q;
    logic                     ok_q;
    logic signed [53:0]       prod;
    logic signed [ACC_W-1:0]  acc;
    logic signed [26:0]       dzp_r;                   // dZ' de la fila actual (UPDATE)

    wire signed [23:0] a_s    = {8'b0, a_q};           // Q0.16 sin signo -> 24 bits con signo (positivo)
    wire signed [26:0] w_x    = {{3{w_q[23]}},  w_q};  // extensiones de signo a 27 bits
    wire signed [26:0] dz_x   = {{3{dz_q[23]}}, dz_q};
    wire signed [26:0] a_x    = {{3{a_s[23]}},  a_s};
    wire signed [26:0] m1 = (op_sel == 2'd3) ? dzp_r : (op_sel == 2'd2) ? dz_x : w_x;
    wire signed [26:0] m2 = (op_sel == 2'd1) ? dz_x  : (op_sel == 2'd2) ? LR_Q : a_x;
    wire signed [23:0] bias_q = bmem[bsel];

    always_ff @(posedge clk) begin
        w_q  <= wmem[raddr];                        // etapa 1: lectura de W
        dz_q <= dzmem[dsel];                        //          lectura de dz
        ok_q <= row_ok;
        prod <= ok_q ? (m1 * m2) : 54'sd0;          // etapa 2: producto 27x27. Fila inexistente -> 0 exacto
        if (load_bias)
            acc <= bias_q <<< 16;                   // acc inicia con b << 16
        else if (mac_en)
            acc <= acc + prod;                      // etapa 3: acumular (forward)
    end

    assign prod_out = prod[47:0];                   // W*dz cabe en 48 bits con signo

    // ---------------- acc -> z (Q4.12) ----------------
    // rsh(v, s) = (v + 2^(s-1)) >> s, desplazamiento aritmetico
    wire signed [ACC_W-1:0] rnd = acc + ({{(ACC_W-1){1'b0}}, 1'b1} << (SH-1));
    wire signed [ACC_W-1:0] shf = rnd >>> SH;

    // saturacion a 16 bits con signo (+-8 en Q4.12)
    localparam signed [ACC_W-1:0] Z_MAX =  32767;
    localparam signed [ACC_W-1:0] Z_MIN = -32768;

    assign z_out = (shf > Z_MAX) ? 16'sh7FFF :
                   (shf < Z_MIN) ? 16'sh8000 :
                                   shf[15:0];

    // ============================================================
    // UPDATE
    // ============================================================
    function automatic logic [31:0] lfsr_step(input logic [31:0] s);   // Galois 32 bits
        lfsr_step = (s >> 1) ^ (s[0] ? 32'h80200003 : 32'h0);
    endfunction

    logic [31:0] lf, lf_rd, r_q;
    logic signed [23:0] w_q2, w_q3, delta_q;
    wire  stoch = (rmode == 2'd1);
    wire  [31:0] lf_next = lfsr_step(lf);

    // R (pesos): se calcula en t+1, con el estado ya avanzado, y se registra junto al producto
    wire [31:0] r_c = stoch ? (lf_next & ((32'h1 << SHU) - 32'h1)) :
                      (rmode == 2'd0) ? (32'h1 << (SHU - 1)) : 32'h0;

    // dZ' = sat27( rsh(prod, 8) ), con prod = dz*lr (op_sel = ULR). Se captura con dzp_ld.
    wire signed [53:0] lr_s = (prod + 54'sd128) >>> 8;
    wire signed [26:0] dzp_c = (lr_s >  54'sd67108863) ? 27'sh3FFFFFF :     //  2^26 - 1
                               (lr_s < -54'sd67108864) ? 27'sh4000000 :     // -2^26
                                                         lr_s[26:0];

    // delta = (prod + R) >>> SHU  (etapa t+2) ; nuevo peso = sat24(W - delta) (etapa t+3)
    wire signed [53:0] dsum = prod + $signed({22'b0, r_q});
    wire signed [53:0] dsh  = dsum >>> SHU;
    wire signed [24:0] w_sub = w_q3 - delta_q;
    assign nw = (w_sub >  25'sd8388607) ? 24'sh7FFFFF :
                (w_sub < -25'sd8388608) ? 24'sh800000 :
                                          w_sub[23:0];

    always_ff @(posedge clk) begin
        lf_rd <= lfmem[dsel];                              // lectura sincrona del banco de estados
        if (dzp_ld) dzp_r <= dzp_c;
        // etapa t -> t+1
        uv1 <= u_issue & row_ok;   wa1 <= raddr;
        // etapa t+1 -> t+2
        w_q2 <= w_q;  uv2 <= uv1;  wa2 <= wa1;  r_q <= r_c;
        if (lf_ld)               lf <= lf_rd;
        else if (uv1 && stoch)   lf <= lf_next;            // un paso por peso (solo estocastico)
        // etapa t+2 -> t+3
        delta_q <= dsh[23:0];  w_q3 <= w_q2;  uv3 <= uv2;  wa3 <= wa2;
    end

    // ---- paso del sesgo (u_bias): un paso mas del LFSR, b <- sat24(b - db), estado de vuelta al banco ----
    wire [31:0] rb_c = stoch ? (lf_next & ((32'h1 << SHB) - 32'h1)) :
                       (rmode == 2'd0) ? (32'h1 << (SHB - 1)) : 32'h0;
    wire signed [31:0] db_sum = dzp_r + $signed({1'b0, rb_c});
    wire signed [31:0] db     = db_sum >>> SHB;
    wire signed [32:0] b_sub  = bias_q - db;
    assign bnew  = (b_sub >  33'sd8388607) ? 24'sh7FFFFF :
                   (b_sub < -33'sd8388608) ? 24'sh800000 :
                                             b_sub[23:0];
    assign lf_nb = stoch ? lf_next : lf;                   // cercano/piso no avanzan el estado

    assign rd_w  = w_q;
    assign rd_b  = bias_q;
    assign rd_lf = lf_rd;
    assign rd_dz = dz_q;

endmodule