// ============================================================
// sigmoide_lut.sv
//
// Sigmoide por LUT con interpolacion lineal. Replica EXACTAMENTE
// lut_sigmoide() de emulador.cpp:
//     u = z + 32768          (z Q4.12 con signo -> u 0..65535)
//     i = u >> 6             (10 bits altos: indice de la tabla)
//     f = u & 63             (6 bits bajos: fraccion de interpolacion)
//     a = l0 + (((l1 - l0) * f + 32) >> 6)     l0=LUT[i], l1=LUT[i+1]
//
// Entrada z : Q4.12 con signo, 16 bits (ya saturado a +-8).
// Salida a  : Q0.16 sin signo, 16 bits.
// Latencia  : 2 ciclos (etapa 1: lectura de la ROM; etapa 2: interpolacion).
// Rendimiento: 1 resultado por ciclo (segmentado).
//
// La tabla (1025 entradas de 16 bits) se carga con $readmemh desde el
// archivo que vuelca el emulador (--vectores=PREF  ->  PREF_lut.hex).
// [VERIFICAR] que Quartus infiere la ROM en M10K con 2 puertos de lectura.
// SystemVerilog (subconjunto sintetizable): Icarus -g2012 y Quartus.
// ============================================================
module sigmoide_lut #(
    parameter LUT_FILE = "lut.hex"
) (
    input                    clk,
    input                    in_valid,
    input  signed [15:0]     z,
    output logic               out_valid,
    output logic [15:0]        a
);

    // ROM de 1025 puntos sobre [-8, 8], paso 1/64, Q0.16
    logic [15:0] rom [0:1024];
    initial $readmemh(LUT_FILE, rom);

    // ---------------- Etapa 1: indexar y leer la ROM ----------------
    // Sumar 32768 a z equivale a invertir su bit de signo.
    wire [15:0] u  = {~z[15], z[14:0]};
    wire [9:0]  ix = u[15:6];

    logic        v1;
    logic [5:0]  f1;
    logic [15:0] l0, l1;

    always_ff @(posedge clk) begin
        v1 <= in_valid;
        f1 <= u[5:0];
        l0 <= rom[{1'b0, ix}];
        l1 <= rom[{1'b0, ix} + 11'd1];     // ix+1 llega hasta 1024
    end

    // ---------------- Etapa 2: interpolacion lineal ------------------
    // (l1 - l0) es >= 0 (la sigmoide es monotona), pero se calcula con
    // signo para ser fiel al emulador (aritmetica i32 con desplazamiento
    // aritmetico).
    wire signed [17:0] d    = $signed({2'b00, l1}) - $signed({2'b00, l0});
    wire signed [24:0] prod = d * $signed({1'b0, f1});
    wire signed [24:0] sum  = prod + 25'sd32;
    wire signed [24:0] sh   = sum >>> 6;

    always_ff @(posedge clk) begin
        out_valid <= v1;
        a         <= l0 + sh[15:0];
    end

endmodule
