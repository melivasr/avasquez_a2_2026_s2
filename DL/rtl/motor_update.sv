// ============================================================
// motor_update.sv
//
// UPDATE de UNA capa con L lanes. Replica actualizar() de emulador.cpp bit a bit,
// para cualquier L y para los tres redondeos:
//   para cada fila j (lane = j mod L, pasada = j / L):
//     dZ'      = sat27( rsh(dz[j]*6554, 8) )                       (una vez por fila)
//     W[j][k] <- sat24( W[j][k] - ((dZ'*a[k] + R) >>> (46-FW)) )    k = 0..N_IN-1, en orden
//     b[j]    <- sat24( b[j]    - ((dZ'      + Rb) >>> (30-FW)) )   despues de la ultima columna
//   R, Rb segun rmode: 0 al mas cercano, 1 ESTOCASTICO (LFSR de Galois de 32 bits, un estado por
//   FILA, un paso por peso en orden de columna y uno mas para el sesgo), 2 piso.
//
// W es N_OUT x N_IN; dz tiene N_OUT valores; a tiene N_IN valores (Q0.16).
// Las filas son independientes entre si (el paralelismo valido esta DENTRO de una imagen).
// La actualizacion es EN SITIO: cada lane lee W[j][k], lo modifica y lo reescribe en su RAM;
// no se materializa dW. El estado del LFSR de cada fila vive en el banco de su lane (opcion A)
// y PERSISTE entre imagenes y epocas: se lee al empezar la fila y se devuelve al terminarla,
// asi el resultado no depende de L.
//
// Secuencia por pasada (cada lane procesa su fila de esa pasada):
//   S_P0 : se presenta la pasada (lee dz[j] y el estado del LFSR, lecturas sincronas)
//   S_P1 : op_sel=ULR -> producto dz*lr
//   S_P2 : dzp_ld (captura dZ'), lf_ld (carga el estado del LFSR)
//   S_STR: N_IN ciclos, un elemento (columna) por ciclo; la lane escribe W 3 ciclos despues
//   S_B0 : ultimo producto en la etapa del multiplicador
//   S_B1 : paso del sesgo (b y estado del LFSR de vuelta al banco)
//   -> siguiente pasada, o S_FIN en la ultima: 1 ciclo para que la ultima escritura de W
//      aterrice; 'done' se activa en el MISMO flanco, asi que done visible => todo visible.
// Ciclos por imagen (SIMULACION, contados por tb_update desde el start hasta done), exactos:
//     PASSES*(N_IN+5) + 1      (p. ej. L=32: 70 y 267 en capas 3 y 2; 3157 en capa 1). NO es tiempo real.
//
// Carga (host / testbench, SOLO con el motor en reposo): W, b, dz, a y estados del LFSR.
// No hay puertos de lectura de pesos: el testbench lee las RAM por referencia jerarquica; el
// acceso del host por JTAG/bus se agregara en la integracion [PENDIENTE].
//
// SystemVerilog (Icarus -g2012 y Quartus).
// ============================================================
module motor_update #(
    parameter L     = 32,            // numero de lanes
    parameter N_IN  = 128,           // columnas de W (longitud de a)
    parameter N_OUT = 64,            // filas de W (longitud de dz)
    parameter FW    = 20,            // bits fraccionarios de W (>= 15 si se usa estocastico)
    // ---- derivados: NO se sobreescriben desde fuera ----
    parameter PASSES = (N_OUT + L - 1) / L,
    parameter KW     = $clog2(N_IN),
    parameter RW     = $clog2(N_OUT),
    parameter BW     = (PASSES > 1) ? $clog2(PASSES) : 1,
    parameter AW     = $clog2(PASSES * N_IN)
) (
    input  logic                clk,
    input  logic                rst,        // sincrono, activo en alto
    input  logic                start,      // pulso de 1 ciclo (solo en reposo)
    output logic                done,       // nivel; se limpia con el siguiente start
    input  logic [1:0]          rmode,      // 0 cercano, 1 estocastico, 2 piso (estatico durante el UPDATE)

    // carga de W[fila][col]
    input  logic                w_we,
    input  logic [RW-1:0]       w_row,
    input  logic [KW-1:0]       w_col,
    input  logic signed [23:0]  w_data,
    // carga de sesgos
    input  logic                b_we,
    input  logic [RW-1:0]       b_row,
    input  logic signed [23:0]  b_data,
    // carga de dz (Q2.22, 24 bits) por fila
    input  logic                dz_we,
    input  logic [RW-1:0]       dz_row,
    input  logic signed [23:0]  dz_data,
    // carga de a (Q0.16) por columna
    input  logic                a_we,
    input  logic [KW-1:0]       a_addr,
    input  logic [15:0]         a_data,
    // carga del estado del LFSR de cada fila
    input  logic                lf_we,
    input  logic [RW-1:0]       lf_row,
    input  logic [31:0]         lf_data
);

    // ============================================================
    // FSM
    // ============================================================
    typedef enum logic [2:0] {S_IDLE, S_P0, S_P1, S_P2, S_STR, S_B0, S_B1, S_FIN} estado_t;
    estado_t       st;
    logic [KW-1:0] k;
    logic [BW-1:0] p;          // pasada actual
    logic [AW-1:0] raddr;      // = p*N_IN + k (avanza sin parar entre pasadas)
    logic [15:0]   amem [0:N_IN-1];
    logic [15:0]   x_q;        // a[k] registrada, alineada con w_q (t+1)

    always_ff @(posedge clk) begin
        if (a_we) amem[a_addr] <= a_data;
        x_q <= amem[k];
        if (rst) begin
            st   <= S_IDLE;
            done <= 1'b0;
        end else begin
            case (st)
                S_IDLE: if (start) begin
                    st <= S_P0;  done <= 1'b0;
                    p <= 0;  k <= 0;  raddr <= 0;
                end
                S_P0:  st <= S_P1;
                S_P1:  st <= S_P2;
                S_P2:  begin st <= S_STR; k <= 0; end
                S_STR: begin
                    raddr <= raddr + 1'b1;
                    if (k == N_IN - 1) st <= S_B0;
                    else               k  <= k + 1'b1;
                end
                S_B0:  st <= S_B1;
                S_B1:  if (p == PASSES - 1) st <= S_FIN;
                       else begin p <= p + 1'b1; st <= S_P0; end
                S_FIN: begin st <= S_IDLE; done <= 1'b1; end
                default: st <= S_IDLE;
            endcase
        end
    end

    // senales de control hacia las lanes (combinacionales desde el estado)
    wire [1:0] op_sel  = (st == S_P1) ? 2'd2 : 2'd3;     // ULR en S_P1; UST el resto (incluye S_B0)
    wire       dzp_ld  = (st == S_P2);
    wire       lf_ld   = (st == S_P2);
    wire       u_issue = (st == S_STR);
    wire       u_bias  = (st == S_B1);

    // ============================================================
    // Lanes
    // ============================================================
    wire [AW-1:0] w_addr_lane = (w_row / L) * N_IN + w_col;
    wire [BW-1:0] b_pass      = b_row  / L;
    wire [BW-1:0] d_pass      = dz_row / L;
    wire [BW-1:0] l_pass      = lf_row / L;

    generate
        for (genvar g = 0; g < L; g = g + 1) begin : lanes
            // pasadas en las que la lane g tiene fila (fila = pasada*L + g < N_OUT)
            localparam integer NPG = (g < N_OUT) ? (N_OUT - g + L - 1) / L : 0;
            wire we_w = w_we  && ((w_row  % L) == g);
            wire we_b = b_we  && ((b_row  % L) == g);
            wire we_d = dz_we && ((dz_row % L) == g);
            wire we_l = lf_we && ((lf_row % L) == g);

            lane_mac #(
                .ADDR_W(AW), .BIAS_AW(BW), .ACC_W(52), .SH(FW + 4)
            ) u_lane (
                .clk(clk),
                .w_we(we_w),   .w_waddr(w_addr_lane), .w_wdata(w_data),
                .b_we(we_b),   .b_waddr(b_pass),      .b_wdata(b_data),
                .dz_we(we_d),  .dz_waddr(d_pass),     .dz_wdata(dz_data),
                .lf_we(we_l),  .lf_waddr(l_pass),     .lf_wdata(lf_data),
                .raddr(raddr), .bsel(p),              .dsel(p),
                .op_sel(op_sel), .row_ok(p < NPG),
                .load_bias(1'b0), .mac_en(1'b0),
                .a_q(x_q),
                .rmode(rmode), .dzp_ld(dzp_ld), .lf_ld(lf_ld), .u_issue(u_issue), .u_bias(u_bias),
                .z_out(),      .prod_out()
            );
        end
    endgenerate

endmodule