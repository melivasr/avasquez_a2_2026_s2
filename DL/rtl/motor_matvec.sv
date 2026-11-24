// ============================================================
// motor_matvec.sv
//
// Motor de UNA capa:  a_out = sigmoide(W * a_in + b)
// con L lanes en paralelo (parametrizable). Replica matvec_sigmoide()
// de emulador.cpp, bit a bit, para cualquier L.
//
// Reparto de filas (opcion A del diseno): la fila r va a la lane r mod L,
// en la pasada r / L. Si N_OUT no es multiplo de L, las lanes sobrantes de
// la ultima pasada se ignoran (sus filas no existen).
//
// Una pasada = 1 ciclo (sesgo) + N_IN ciclos (MACs) + 3 (vaciado de la
// segmentacion) + L ciclos (sigmoide serial, compartida entre lanes).
// Total ~ PASADAS * (N_IN + L + 4) ciclos. [Medido en simulacion, no en placa]
//
// Interfaces de carga/lectura: son las que usara despues el bus
// (JTAG/CPU). Por ahora las maneja el testbench.
//
// SystemVerilog (Icarus -g2012 y Quartus).
// ============================================================
module motor_matvec #(
    parameter L        = 32,            // numero de lanes
    parameter N_IN     = 784,           // entradas de la capa
    parameter N_OUT    = 128,           // neuronas de la capa
    parameter FW       = 20,            // bits fraccionarios de W y b
    parameter LUT_FILE = "lut.hex",
    // ---- derivados: NO se sobreescriben desde fuera ----
    parameter PASSES   = (N_OUT + L - 1) / L,               // pasadas
    parameter KW       = $clog2(N_IN),                      // bits de columna
    parameter RW       = $clog2(N_OUT),                     // bits de fila
    parameter BW       = (PASSES > 1) ? $clog2(PASSES) : 1, // bits de pasada
    parameter AW       = $clog2(PASSES * N_IN),             // direccion de W por lane
    parameter LW       = $clog2(L) + 1,                     // contador de lanes
    // row_base + sg puede llegar a N_OUT-1+L-1 (lanes sobrantes de la ultima
    // pasada): el indice debe cubrir N_OUT+L.
    parameter CW       = $clog2(N_OUT + L)
) (
    input  logic                clk,
    input  logic                rst,        // sincrono, activo en alto
    input  logic                start,      // pulso de 1 ciclo (solo en reposo)
    output logic                done,       // nivel; se limpia con el siguiente start

    // carga de la activacion de entrada x (Q0.16)
    input  logic                x_we,
    input  logic [KW-1:0]       x_waddr,
    input  logic [15:0]         x_wdata,

    // carga de pesos: W[fila][col] y sesgos b[fila]
    input  logic                w_we,
    input  logic [RW-1:0]       w_row,
    input  logic [KW-1:0]       w_col,
    input  logic signed [23:0]  w_data,
    input  logic                b_we,
    input  logic [RW-1:0]       b_row,
    input  logic signed [23:0]  b_data,

    // lectura de resultados (asincrona, para el testbench)
    input  logic [RW-1:0]       y_addr,
    output logic [15:0]         y_data,     // a_out  (Q0.16)
    output logic [15:0]         zy_data     // z      (Q4.12), para depurar
);

    // ---------------- RAM de activaciones de entrada ----------------
    logic [15:0] xmem [0:N_IN-1];
    logic [15:0] x_q;
    always_ff @(posedge clk) if (x_we) xmem[x_waddr] <= x_wdata;

    // ---------------- buffers de salida ----------------
    logic [15:0] aout [0:N_OUT-1];
    logic [15:0] zout [0:N_OUT-1];
    assign y_data  = aout[y_addr];
    assign zy_data = zout[y_addr];

    // ---------------- control ----------------
    typedef enum logic [2:0] {S_IDLE, S_BIAS, S_RUN, S_DRAIN, S_SIG, S_FLUSH} estado_t;

    estado_t     st;
    logic [BW-1:0] pass;
    logic [CW-1:0] row_base;      // primera fila de la pasada
    logic [KW-1:0] k;             // columna actual dentro de la pasada
    logic [AW-1:0] raddr;         // direccion de W (avanza sin parar entre pasadas)
    logic [1:0]    dcnt;
    logic [LW-1:0] sg;            // lane que se envia a la sigmoide
    logic [1:0]    fcnt;
    logic          v1, v2;        // validez del MAC retrasada 1 y 2 ciclos

    wire rd_v      = (st == S_RUN);
    wire load_bias = (st == S_BIAS);

    // z de todas las lanes y su registro de captura (L x 16 bits)
    wire  [L*16-1:0] z_lane;
    logic  [L*16-1:0] zreg;

    // fila que se esta enviando a la sigmoide
    wire [CW-1:0] row_cur = row_base + sg;
    wire        sig_in_valid = (st == S_SIG) && (row_cur < N_OUT);
    wire signed [15:0] sig_z = zreg[sg*16 +: 16];

    always_ff @(posedge clk) begin
        x_q <= xmem[k];         // lectura sincrona, alineada con w_q de las lanes
        v1  <= rd_v;
        v2  <= v1;

        if (rst) begin
            st   <= S_IDLE;
            done <= 1'b0;
        end else begin
            case (st)
                S_IDLE: if (start) begin
                    st       <= S_BIAS;
                    done     <= 1'b0;
                    pass     <= 0;
                    row_base <= 0;
                    raddr    <= 0;
                    k        <= 0;
                end

                S_BIAS: begin               // acc <= b << 16 en todas las lanes
                    st <= S_RUN;
                    k  <= 0;
                end

                S_RUN: begin                // un MAC por ciclo en cada lane
                    raddr <= raddr + 1'b1;
                    if (k == N_IN - 1) begin
                        st   <= S_DRAIN;
                        dcnt <= 0;
                    end else
                        k <= k + 1'b1;
                end

                S_DRAIN: begin              // vaciar la segmentacion (3 ciclos)
                    dcnt <= dcnt + 1'b1;
                    if (dcnt == 2) begin
                        zreg <= z_lane;     // acc ya completo
                        st   <= S_SIG;
                        sg   <= 0;
                    end
                end

                S_SIG: begin                // sigmoide serial: una lane por ciclo
                    sg <= sg + 1'b1;
                    if (sg == L - 1) begin
                        if (pass == PASSES - 1) begin
                            st   <= S_FLUSH;
                            fcnt <= 0;
                        end else begin
                            pass     <= pass + 1'b1;
                            row_base <= row_base + L;
                            st       <= S_BIAS;
                        end
                    end
                end

                S_FLUSH: begin              // esperar la latencia de la sigmoide (2)
                    fcnt <= fcnt + 1'b1;
                    if (fcnt == 1) begin
                        st   <= S_IDLE;
                        done <= 1'b1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    // ---------------- sigmoide compartida y escritura de resultados ----------------
    wire        sig_out_valid;
    wire [15:0] sig_a;

    sigmoide_lut #(.LUT_FILE(LUT_FILE)) u_sig (
        .clk(clk), .in_valid(sig_in_valid), .z(sig_z),
        .out_valid(sig_out_valid), .a(sig_a)
    );

    // la fila viaja junto al dato por las 2 etapas de la sigmoide
    logic [CW-1:0] rd1, rd2;
    always_ff @(posedge clk) begin
        rd1 <= row_cur;
        rd2 <= rd1;
        if (sig_in_valid)  zout[row_cur[RW-1:0]] <= sig_z;
        if (sig_out_valid) aout[rd2[RW-1:0]]     <= sig_a;
    end

    // ---------------- lanes ----------------
    wire [AW-1:0] w_addr_lane = (w_row / L) * N_IN + w_col;
    wire [BW-1:0] b_pass      = b_row / L;

    generate
        for (genvar g = 0; g < L; g = g + 1) begin : lanes
            wire we_w = w_we && ((w_row % L) == g);
            wire we_b = b_we && ((b_row % L) == g);
            wire signed [15:0] z;

            lane_mac #(
                .ADDR_W(AW), .BIAS_AW(BW), .ACC_W(52), .SH(FW + 4)
            ) u_lane (
                .clk(clk),
                .w_we(we_w),  .w_waddr(w_addr_lane), .w_wdata(w_data),
                .b_we(we_b),  .b_waddr(b_pass),      .b_wdata(b_data),
                // ---- puertos del backward: sin uso en este motor (solo forward) ----
                .dz_we(1'b0), .dz_waddr('0), .dz_wdata(24'sd0),
                .lf_we(1'b0), .lf_waddr('0), .lf_wdata(32'd0),
                .dsel('0), .op_sel(2'd0), .row_ok(1'b1),
                .rmode(2'd0), .dzp_ld(1'b0), .lf_ld(1'b0), .u_issue(1'b0), .u_bias(1'b0),
                .raddr(raddr), .bsel(pass),
                .load_bias(load_bias), .mac_en(v2),
                .a_q(x_q),
                .z_out(z), .prod_out()
            );

            assign z_lane[g*16 +: 16] = z;
        end
    endgenerate

endmodule