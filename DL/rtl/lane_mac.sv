// ============================================================
// lane_mac.sv   (v2: forward + backward)
//
// Una lane del motor vectorial: atiende UNA fila de W a la vez.
// Contiene:
//   - RAM de pesos de sus filas (24 bits con signo, formato Q4.20)
//   - memoria de sesgos de sus filas (una entrada por pasada)
//   - banco de dz de sus filas (una entrada por pasada)   [v2, backward]
//   - MULTIPLICADOR COMPARTIDO 24x24 con signo (cabe en un DSP en modo
//     27x27 [VERIFICAR en el manual de la Cyclone V]); el 2.o operando se
//     elige con 'bw':  bw=0 -> a (Q0.16, forward) ; bw=1 -> dz (Q2.22, backward)
//   - acumulador de ACC_W bits y conversion a z (Q4.12) para el forward
//
// FORWARD (bw=0), replica MATVEC de matvec_sigmoide() de emulador.cpp:
//     acc = (b << 16) + sum_k W[j][k] * a[k]          (F = FW+16)
//     z   = sat16( (acc + 2^(SH-1)) >>> SH ),  SH = FW+4   -> Q4.12
// BACKWARD (bw=1): la lane NO acumula; entrega en prod_out el producto
//     W[j][k] * dz[j]   (48 bits con signo) y el motor suma ENTRE LANES
// con un arbol (W^T * dz suma sobre filas = entre lanes). Si la fila de esta
// lane no existe en la pasada actual (row_ok=0) entrega EXACTAMENTE 0, nunca
// basura (la RAM de pesos de una lane sin fila puede tener cualquier valor).
//
// Segmentacion (la lane recibe 'raddr', 'dsel' y 'a_q' alineados):
//   ciclo t   : se presenta raddr/dsel/row_ok (y el padre lee su RAM de activaciones)
//   ciclo t+1 : w_q, dz_q, ok_q y a_q validos   -> se calcula el producto
//   ciclo t+2 : prod valido (prod_out); en forward mac_en=1 y acc <= acc + prod
//
// Nota sobre el acumulador: el emulador acumula en 64 bits y satura a 48
// ANTES de convertir a z. Aqui el acumulador es mas ancho (ACC_W=52), de
// modo que nunca desborda, y se omite esa saturacion a 48 bits: es
// equivalente, porque si |acc| >= 2^47 el resultado z ya satura a +-8 en
// ambos casos.
//
// SystemVerilog (Icarus -g2012 y Quartus).
// ============================================================
module lane_mac #(
    parameter ADDR_W  = 12,   // bits de direccion de la RAM de pesos
    parameter BIAS_AW = 2,    // bits de direccion de las memorias de sesgo y de dz (una entrada por pasada)
    parameter ACC_W   = 52,   // ancho del acumulador (con signo)
    parameter SH      = 24    // FW + 4 : desplazamiento acc -> Q4.12
) (
    input                          clk,

    // ---- carga de pesos y sesgos (host / testbench) ----
    input                          w_we,
    input      [ADDR_W-1:0]        w_waddr,
    input  signed [23:0]           w_wdata,
    input                          b_we,
    input      [BIAS_AW-1:0]       b_waddr,
    input  signed [23:0]           b_wdata,

    // ---- carga del banco de dz (backward) ----
    input                          dz_we,
    input      [BIAS_AW-1:0]       dz_waddr,
    input  signed [23:0]           dz_wdata,

    // ---- control desde el motor ----
    input      [ADDR_W-1:0]        raddr,      // direccion de lectura de W
    input      [BIAS_AW-1:0]       bsel,       // pasada actual (sesgo)
    input      [BIAS_AW-1:0]       dsel,       // pasada actual (dz)
    input                          bw,         // 0: forward (a) ; 1: backward (dz). Nivel estatico
    input                          row_ok,     // 1 si la fila de esta lane existe en la pasada (alineado con raddr)
    input                          load_bias,  // acc <= b << 16
    input                          mac_en,     // acc <= acc + prod
    input      [15:0]              a_q,        // activacion (Q0.16), difundida

    // ---- resultados ----
    output signed [15:0]           z_out,      // forward: Q4.12, saturado a +-8
    output signed [47:0]           prod_out    // backward: W*dz (0 si row_ok=0), valido en t+2
);

    // ---------------- memorias ----------------
    logic signed [23:0] wmem  [0:(1<<ADDR_W)-1];      // pesos (lectura sincrona)
    logic signed [23:0] bmem  [0:(1<<BIAS_AW)-1];     // sesgos (lectura asincrona, pocas entradas)
    logic signed [23:0] dzmem [0:(1<<BIAS_AW)-1];     // dz por pasada (lectura sincrona)

    always_ff @(posedge clk) begin
        if (w_we)  wmem[w_waddr]   <= w_wdata;
        if (b_we)  bmem[b_waddr]   <= b_wdata;
        if (dz_we) dzmem[dz_waddr] <= dz_wdata;
    end

    // ---------------- MAC segmentado ----------------
    logic signed [23:0]       w_q;
    logic signed [23:0]       dz_q;
    logic                     ok_q;
    logic signed [47:0]       prod;
    logic signed [ACC_W-1:0]  acc;

    wire signed [23:0] a_s    = {8'b0, a_q};        // Q0.16 sin signo -> 24 bits con signo (positivo)
    wire signed [23:0] op2    = bw ? dz_q : a_s;    // 2.o operando del multiplicador compartido
    wire signed [23:0] bias_q = bmem[bsel];

    always_ff @(posedge clk) begin
        w_q  <= wmem[raddr];                        // etapa 1: lectura de W
        dz_q <= dzmem[dsel];                        //          lectura de dz (backward)
        ok_q <= row_ok;
        // etapa 2: producto 24x24 (48 bits). Fila inexistente -> 0 exacto.
        prod <= ok_q ? (w_q * op2) : 48'sd0;
        if (load_bias)
            acc <= bias_q <<< 16;                   // acc inicia con b << 16
        else if (mac_en)
            acc <= acc + prod;                      // etapa 3: acumular (forward)
    end

    assign prod_out = prod;

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

endmodule