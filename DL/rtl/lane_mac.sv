// ============================================================
// lane_mac.sv
//
// Una lane del motor vectorial: atiende UNA fila de W a la vez.
// Contiene:
//   - RAM de pesos de sus filas (24 bits con signo, formato Q4.20)
//   - memoria de sesgos de sus filas (una entrada por pasada)
//   - MAC segmentado (producto 24x16 y acumulador de ACC_W bits)
//   - conversion del acumulador a z (Q4.12, redondeo y saturacion)
//
// Replica la parte MATVEC de matvec_sigmoide() de emulador.cpp:
//     acc = (b << 16) + sum_k W[j][k] * a[k]          (F = FW+16)
//     z   = sat16( (acc + 2^(SH-1)) >>> SH ),  SH = FW+4   -> Q4.12
//
// Segmentacion (la lane recibe 'raddr' y 'a_q' alineados):
//   ciclo t   : se presenta raddr (y el padre lee su RAM de activaciones)
//   ciclo t+1 : w_q y a_q validos      -> se calcula el producto
//   ciclo t+2 : prod valido            -> mac_en=1, acc <= acc + prod
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
    parameter BIAS_AW = 2,    // bits de direccion de la memoria de sesgos
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

    // ---- control desde el motor ----
    input      [ADDR_W-1:0]        raddr,      // direccion de lectura de W
    input      [BIAS_AW-1:0]       bsel,       // pasada actual (sesgo)
    input                          load_bias,  // acc <= b << 16
    input                          mac_en,     // acc <= acc + prod
    input      [15:0]              a_q,        // activacion (Q0.16), difundida

    // ---- resultado ----
    output signed [15:0]           z_out       // Q4.12, saturado a +-8
);

    // ---------------- memorias ----------------
    logic signed [23:0] wmem [0:(1<<ADDR_W)-1];       // pesos (lectura sincrona)
    logic signed [23:0] bmem [0:(1<<BIAS_AW)-1];      // sesgos (lectura asincrona, pocas entradas)

    always_ff @(posedge clk) begin
        if (w_we) wmem[w_waddr] <= w_wdata;
        if (b_we) bmem[b_waddr] <= b_wdata;
    end

    // ---------------- MAC segmentado ----------------
    logic signed [23:0]       w_q;
    logic signed [40:0]       prod;
    logic signed [ACC_W-1:0]  acc;

    wire signed [16:0] a_s    = {1'b0, a_q};        // Q0.16 sin signo -> con signo
    wire signed [23:0] bias_q = bmem[bsel];

    always_ff @(posedge clk) begin
        w_q  <= wmem[raddr];                        // etapa 1: lectura de W
        prod <= w_q * a_s;                          // etapa 2: producto (41 bits)
        if (load_bias)
            acc <= bias_q <<< 16;                   // acc inicia con b << 16
        else if (mac_en)
            acc <= acc + prod;                      // etapa 3: acumular
    end

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