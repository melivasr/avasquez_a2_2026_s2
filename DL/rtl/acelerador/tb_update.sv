// ============================================================
// tb_update.sv
//
// Testbench de motor_update (UPDATE de una capa) contra los vectores dorados del emulador.
// Carga W, b y el estado del LFSR de cada fila, encadena T imagenes (para cada una carga
// dz y a y ejecuta un UPDATE) y compara BIT A BIT el estado final:
//     W (24 b), b (24 b) y estado del LFSR (32 b)  contra  F_WPOST, F_BPOST, F_LFPOST.
// Como UPDATE reescribe las RAM de las lanes y aun no hay puertos de lectura, el testbench
// las lee por REFERENCIA JERARQUICA (solo simulacion).
//
// Orígenes de datos (parametros -P):
//   Entrenamiento (--vectores=PREF, T=1): capa n con (dz_n, a_n):
//     capa 3: N_OUT=10 N_IN=64   F_DZ=PREF_dz3 F_A=PREF_a2   F_WPRE=PREF_W3_pre ... F_LFPOST=PREF_lf3_post
//     capa 2: N_OUT=64 N_IN=128  F_DZ=PREF_dz2 F_A=PREF_a1
//     capa 1: N_OUT=128 N_IN=784 F_DZ=PREF_dz1 F_A=PREF_x
//   Sinteticos (--test-update, T imagenes): PREF_u3_*, PREF_u2_*
// RMODE debe coincidir con el --redondeo con que se generaron los vectores (sim_update.sh lo
// lee de PREF_modo.txt). L (lanes) tambien es parametro: el resultado debe ser el mismo.
//
// Esto es SIMULACION: los ciclos que imprime no son tiempo real.
// ============================================================
`timescale 1ns/1ps

module tb_update;
    parameter L      = 32;
    parameter N_IN   = 128;
    parameter N_OUT  = 64;
    parameter T      = 1;           // imagenes encadenadas
    parameter RMODE  = 0;           // 0 cercano, 1 estocastico, 2 piso
    parameter FW     = 20;
    parameter F_WPRE  = "pre_W.hex";
    parameter F_BPRE  = "pre_b.hex";
    parameter F_LFPRE = "pre_lf.hex";
    parameter F_DZ    = "dz.hex";
    parameter F_A     = "a.hex";
    parameter F_WPOST  = "post_W.hex";
    parameter F_BPOST  = "post_b.hex";
    parameter F_LFPOST = "post_lf.hex";

    localparam KW = $clog2(N_IN);
    localparam RW = $clog2(N_OUT);

    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 1;

    logic                 start = 0;
    wire                  done;
    logic                 w_we = 0;   logic [RW-1:0] w_row = 0;  logic [KW-1:0] w_col = 0;
    logic signed [23:0]   w_data = 0;
    logic                 b_we = 0;   logic [RW-1:0] b_row = 0;  logic signed [23:0] b_data = 0;
    logic                 dz_we = 0;  logic [RW-1:0] dz_row = 0; logic signed [23:0] dz_data = 0;
    logic                 a_we = 0;   logic [KW-1:0] a_addr = 0; logic [15:0] a_data = 0;
    logic                 lf_we = 0;  logic [RW-1:0] lf_row = 0; logic [31:0] lf_data = 0;

    motor_update #(.L(L), .N_IN(N_IN), .N_OUT(N_OUT), .FW(FW)) dut (
        .clk(clk), .rst(rst), .start(start), .done(done), .rmode(RMODE[1:0]),
        .w_we(w_we), .w_row(w_row), .w_col(w_col), .w_data(w_data),
        .b_we(b_we), .b_row(b_row), .b_data(b_data),
        .dz_we(dz_we), .dz_row(dz_row), .dz_data(dz_data),
        .a_we(a_we), .a_addr(a_addr), .a_data(a_data),
        .lf_we(lf_we), .lf_row(lf_row), .lf_data(lf_data)
    );

    // tablas doradas (un valor por linea, hex)
    logic [23:0] wpre   [0:N_OUT*N_IN-1];
    logic [23:0] bpre   [0:N_OUT-1];
    logic [31:0] lfpre  [0:N_OUT-1];
    logic [23:0] dztab  [0:T*N_OUT-1];
    logic [15:0] atab   [0:T*N_IN-1];
    logic [23:0] wpost  [0:N_OUT*N_IN-1];
    logic [23:0] bpost  [0:N_OUT-1];
    logic [31:0] lfpost [0:N_OUT-1];
    // copia del estado final leido de las RAM de las lanes
    logic [23:0] wcap  [0:N_OUT*N_IN-1];
    logic [23:0] bcap  [0:N_OUT-1];
    logic [31:0] lfcap [0:N_OUT-1];

    // ---- BASURA en las RAM de todas las lanes antes de cargar los datos reales ----
    // (ver tb_matvec_t.sv): una lane sin fila que escribiera donde no debe, o leyera de mas,
    // fallaria tambien en simulacion. La carga sobreescribe solo las entradas reales.
    // ---- Y captura del estado final por lane (referencia jerarquica) ----
    reg cap = 0;
    generate
        for (genvar g = 0; g < L; g = g + 1) begin : lane_tb
            integer i, pp, kk, r;
            initial begin
                for (i = 0; i < (1 << dut.AW); i = i + 1) dut.lanes[g].u_lane.wmem[i]  = $random;
                for (i = 0; i < (1 << dut.BW); i = i + 1) begin
                    dut.lanes[g].u_lane.dzmem[i] = $random;
                    dut.lanes[g].u_lane.bmem[i]  = $random;
                    dut.lanes[g].u_lane.lfmem[i] = $random;
                end
            end
            always @(posedge cap) begin
                for (pp = 0; pp < dut.PASSES; pp = pp + 1) begin
                    r = pp * L + g;
                    if (r < N_OUT) begin
                        for (kk = 0; kk < N_IN; kk = kk + 1)
                            wcap[r*N_IN + kk] = dut.lanes[g].u_lane.wmem[pp*N_IN + kk];
                        bcap[r]  = dut.lanes[g].u_lane.bmem[pp];
                        lfcap[r] = dut.lanes[g].u_lane.lfmem[pp];
                    end
                end
            end
        end
    endgenerate

    integer r, c, t, ciclos, ciclos_tot, eW, eb, el, hayx;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_update.vcd");
            $dumpvars(0, tb_update);
        end
        $readmemh(F_WPRE, wpre);   $readmemh(F_BPRE, bpre);   $readmemh(F_LFPRE, lfpre);
        $readmemh(F_DZ, dztab);    $readmemh(F_A, atab);
        $readmemh(F_WPOST, wpost); $readmemh(F_BPOST, bpost); $readmemh(F_LFPOST, lfpost);

        // ---- guarda anti falso-PASS: si un archivo falta o esta incompleto, $readmemh deja 'X'
        //      y 'X !== X' es falso en Verilog (compararia X contra X y daria PASS) ----
        hayx = 0;
        for (r = 0; r < N_OUT*N_IN; r = r + 1) if ((^wpre[r] === 1'bx) || (^wpost[r] === 1'bx)) hayx = 1;
        for (r = 0; r < N_OUT; r = r + 1)
            if ((^bpre[r] === 1'bx) || (^lfpre[r] === 1'bx) || (^bpost[r] === 1'bx) || (^lfpost[r] === 1'bx)) hayx = 1;
        for (r = 0; r < T*N_OUT; r = r + 1) if (^dztab[r] === 1'bx) hayx = 1;
        for (r = 0; r < T*N_IN;  r = r + 1) if (^atab[r]  === 1'bx) hayx = 1;
        if (hayx) begin
            $display("FALLO: algun archivo de vectores falta o esta incompleto (hay 'x' en las tablas)");
            $finish;
        end

        repeat (4) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // ---- carga del estado inicial (un valor por ciclo; se maneja en el flanco negativo) ----
        for (r = 0; r < N_OUT; r = r + 1)
            for (c = 0; c < N_IN; c = c + 1) begin
                @(negedge clk);
                w_we = 1; w_row = r; w_col = c; w_data = wpre[r*N_IN + c];
            end
        @(negedge clk); w_we = 0;
        for (r = 0; r < N_OUT; r = r + 1) begin
            @(negedge clk);
            b_we = 1;  b_row = r;  b_data = bpre[r];
            lf_we = 1; lf_row = r; lf_data = lfpre[r];
        end
        @(negedge clk); b_we = 0; lf_we = 0;

        // ---- T imagenes encadenadas ----
        ciclos_tot = 0;
        for (t = 0; t < T; t = t + 1) begin
            for (r = 0; r < N_OUT; r = r + 1) begin
                @(negedge clk);
                dz_we = 1; dz_row = r; dz_data = dztab[t*N_OUT + r];
            end
            @(negedge clk); dz_we = 0;
            for (c = 0; c < N_IN; c = c + 1) begin
                @(negedge clk);
                a_we = 1; a_addr = c; a_data = atab[t*N_IN + c];
            end
            @(negedge clk); a_we = 0;

            @(negedge clk); start = 1;
            @(negedge clk); start = 0;
            ciclos = 0;
            while (!done && ciclos < 400000) begin
                @(negedge clk);
                ciclos = ciclos + 1;
            end
            if (!done) begin
                $display("FALLO: timeout, 'done' nunca se activo (imagen %0d)", t);
                $finish;
            end
            ciclos_tot = ciclos_tot + ciclos;
        end

        // ---- captura y comparacion bit a bit, EN CUANTO 'done' es visible ----
        cap = 1; #1;
        eW = 0; eb = 0; el = 0;
        for (r = N_OUT - 1; r >= 0; r = r - 1) begin
            for (c = N_IN - 1; c >= 0; c = c - 1)
                if (wcap[r*N_IN + c] !== wpost[r*N_IN + c]) begin
                    if (eW < 8) $display("  W[%0d][%0d]=%h (esperado %h)", r, c, wcap[r*N_IN + c], wpost[r*N_IN + c]);
                    eW = eW + 1;
                end
            if (bcap[r]  !== bpost[r])  begin
                if (eb < 8) $display("  b[%0d]=%h (esperado %h)", r, bcap[r], bpost[r]);
                eb = eb + 1;
            end
            if (lfcap[r] !== lfpost[r]) begin
                if (el < 8) $display("  lfsr[%0d]=%h (esperado %h)", r, lfcap[r], lfpost[r]);
                el = el + 1;
            end
        end
        $display("L=%0d N_IN=%0d N_OUT=%0d T=%0d rmode=%0d  ciclos(simulados,por imagen)=%0d  errores: W=%0d b=%0d LFSR=%0d",
                 L, N_IN, N_OUT, T, RMODE, ciclos_tot / T, eW, eb, el);
        if (eW + eb + el == 0) $display("PASS: W, b y estado del LFSR coinciden bit a bit con el emulador");
        else                   $display("FAIL");
        $finish;
    end
endmodule
