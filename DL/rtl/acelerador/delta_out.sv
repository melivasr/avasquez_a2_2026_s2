// ============================================================
// delta_out.sv
//
// Comando DELTA_OUT: a partir de la salida a3 y la etiqueta calcula, para
// cada neurona k de la capa de salida (replica el bloque DELTA_OUT de
// entrenar_fixed() en emulador.cpp):
//
//     y   = (k == etiqueta) ? 65536 : 0                  (one-hot, F16)
//     e   = a3[k] - y                                     (F16, con signo)
//     s   = (a3[k] * (65536 - a3[k]) + 32768) >> 16       (a(1-a), F16)
//     dz3[k] = (e * s * K02 + 2^25) >>> 26                (Q2.22, 24 bits)
//     e2     = suma de e*e sobre k                        (F32, para el MSE)
//     pred   = argmax(a3), el PRIMER maximo si hay empates
//     hit    = (pred == etiqueta)
//
// Notas de diseno:
//  * Hay UN solo redondeo en el producto triple (el ">>> 26" final); el
//    producto e*s*K02 se calcula exacto. El unico redondeo previo es el de s.
//  * dz3 NO necesita saturador de 24 bits: |e|<=65536, s<=16384 y K02=13107
//    dan |e*s*K02| < 2^44, y tras el desplazamiento |dz3| <= 209712, muy por
//    debajo de 2^23. (El emulador tiene el saturador, pero no puede activarse.)
//  * K02 = 13107 es 2/10 en F16 y depende de N_OUT=10: si la capa de salida
//    cambia, hay que recalcularlo (round(2/N_OUT * 65536)).
//  * Son solo N=10 elementos: se procesan en SERIE (1 por ciclo) con un
//    solo juego de multiplicadores. Costo ~N+8 ciclos, despreciable frente a
//    los ~7000 ciclos por imagen del resto del motor.
//
// Protocolo de la RAM de a3: lectura SINCRONA (rd_data llega 1 ciclo despues
// de rd_addr), como un M10K. Escribe dz3 por (wr_en, wr_addr, wr_data),
// tambien con 1 ciclo de registro; 'done' se activa en el mismo ciclo en que
// el ultimo dz3 ya quedo capturado por una RAM sincrona.
//
// Segmentacion (t = ciclo en que se presenta rd_addr de un elemento):
//   t+1 : rd_data valido -> e, a*(65536-a)
//   t+2 : s, e*s, e*e
//   t+3 : (e*s)*K02, redondeo, acumula e2, argmax; se registran wr_*
//   t+4 : wr_en/wr_addr/wr_data visibles (la RAM los captura al final de t+4)
// SystemVerilog (Icarus -g2012 y Quartus).
// ============================================================
module delta_out #(
    parameter N   = 10,              // neuronas de salida
    parameter K02 = 13107,           // 2/N en F16 (emulador: K02_Q)
    parameter AW  = $clog2(N)
) (
    input  logic                clk,
    input  logic                rst,
    input  logic                start,       // pulso de 1 ciclo (solo en reposo)
    input  logic [3:0]          label,       // etiqueta (0..N-1), estable al hacer start

    // lectura de a3 (Q0.16), RAM sincrona
    output logic [AW-1:0]       rd_addr,
    input  logic [15:0]         rd_data,

    // escritura de dz3 (Q2.22, 24 bits con signo)
    output logic                wr_en,
    output logic [AW-1:0]       wr_addr,
    output logic signed [23:0]  wr_data,

    output logic                done,        // nivel; se limpia con el siguiente start
    output logic [35:0]         e2_sum,      // suma de e^2 de esta imagen (F32)
    output logic [3:0]          pred,        // argmax (primer maximo)
    output logic                hit          // pred == etiqueta
);

    typedef enum logic [1:0] {S_IDLE, S_RUN, S_DRAIN} estado_t;
    estado_t       st;
    logic [AW-1:0] cnt;
    logic [2:0]    dcnt;
    logic [3:0]    lab;

    wire issue = (st == S_RUN);
    assign rd_addr = cnt;

    // ---------------- etapa 1 (t+1): e y a*(1-a) ----------------
    logic          v1;
    logic [AW-1:0] k1;

    wire        [16:0] y_c = (k1 == lab) ? 17'd65536 : 17'd0;
    wire signed [17:0] e_c = $signed({2'b00, rd_data}) - $signed({1'b0, y_c});
    wire        [31:0] q_c = rd_data * (17'd65536 - {1'b0, rd_data});

    logic signed [17:0] e_r;
    logic        [31:0] q_r;
    logic        [15:0] a_r;
    logic [AW-1:0]      k_r;
    logic               v_r;

    // ---------------- etapa 2 (t+2): s, e*s, e*e ----------------
    wire        [31:0] s_full = (q_r + 32'd32768) >> 16;       // a(1-a), F16
    wire signed [35:0] em_c   = e_r * $signed({1'b0, s_full[15:0]});
    wire signed [35:0] e2_c   = e_r * e_r;

    logic signed [35:0] em_r;
    logic signed [35:0] e2_r;
    logic        [15:0] a_r2;
    logic [AW-1:0]      k_r2;
    logic               v_r2;

    // ---------------- etapa 3 (t+3): *K02, redondeo, acumuladores ----------------
    localparam signed [15:0] KS = K02;
    wire signed [55:0] prod_c = em_r * KS;
    wire signed [55:0] dz_c   = (prod_c + 56'sd33554432) >>> 26;   // +2^25, >>> 26

    logic [15:0] best;

    always_ff @(posedge clk) begin
        // segmentacion: indices y validez viajan junto con los datos
        v1   <= issue;
        k1   <= cnt;
        e_r  <= e_c;
        q_r  <= q_c;
        a_r  <= rd_data;
        k_r  <= k1;
        v_r  <= v1;
        em_r <= em_c;
        e2_r <= e2_c;
        a_r2 <= a_r;
        k_r2 <= k_r;
        v_r2 <= v_r;

        // salida registrada hacia la RAM de dz3
        wr_en   <= v_r2;
        wr_addr <= k_r2;
        wr_data <= dz_c[23:0];

        // acumulacion de e^2 y argmax (estrictamente mayor -> primer maximo)
        if (v_r2) begin
            e2_sum <= e2_sum + e2_r[35:0];   // e2_r >= 0
            if (k_r2 == 0 || a_r2 > best) begin
                best <= a_r2;
                pred <= {{(4-AW){1'b0}}, k_r2};
            end
        end

        // control
        if (rst) begin
            st   <= S_IDLE;
            done <= 1'b0;
        end else begin
            case (st)
                S_IDLE: if (start) begin
                    st     <= S_RUN;
                    cnt    <= 0;
                    lab    <= label;
                    done   <= 1'b0;
                    e2_sum <= 0;
                end
                S_RUN: begin
                    if (cnt == N - 1) begin
                        st   <= S_DRAIN;
                        dcnt <= 0;
                    end else
                        cnt <= cnt + 1'b1;
                end
                S_DRAIN: begin                 // vaciar la segmentacion (4 ciclos)
                    dcnt <= dcnt + 1'b1;
                    if (dcnt == 3) begin
                        st   <= S_IDLE;
                        done <= 1'b1;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    assign hit = (pred == lab);

endmodule
