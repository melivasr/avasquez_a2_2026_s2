// ============================================================
// tb_matvec_t.sv
//
// Testbench de motor_matvec_t (MATVEC_T + DERIV) contra los vectores dorados
// del emulador. Compara BIT A BIT, para cada columna k:
//     dA[k]      (Q2.22, 24 bits)   contra  F_DA
//     dz_prev[k] (Q2.22, 24 bits)   contra  F_DZP
//
// Sirve para las 2 capas y para los 2 origenes de datos cambiando parametros (-P):
//   Entrenamiento real (--vectores=PREF):
//     capa 3->2: N_OUT=10 N_IN=64   F_W=PREF_W3_pre.hex F_DZ=PREF_dz3.hex F_A=PREF_a2.hex F_DA=PREF_dA2.hex F_DZP=PREF_dz2.hex
//     capa 2->1: N_OUT=64 N_IN=128  F_W=PREF_W2_pre.hex F_DZ=PREF_dz2.hex F_A=PREF_a1.hex F_DA=PREF_dA1.hex F_DZP=PREF_dz1.hex
//   Sinteticos (--test-backward --vectores=PREF):  PREF_c3_{W,dz,a,dA,dzp}.hex y PREF_c2_*
// L (numero de lanes) tambien es parametro: el resultado debe ser el mismo para cualquier L.
//
// Esto es SIMULACION: los ciclos que imprime no son tiempo real.
// ============================================================
`timescale 1ns/1ps

module tb_matvec_t;
    parameter L     = 32;
    parameter N_IN  = 128;      // columnas de W
    parameter N_OUT = 64;       // filas de W
    parameter F_W   = "vec_W2_pre.hex";
    parameter F_DZ  = "vec_dz2.hex";
    parameter F_A   = "vec_a1.hex";
    parameter F_DA  = "vec_dA1.hex";
    parameter F_DZP = "vec_dz1.hex";

    localparam KW = $clog2(N_IN);
    localparam RW = $clog2(N_OUT);

    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 1;

    logic                 start = 0;
    wire                  done;
    logic                 w_we = 0;
    logic [RW-1:0]        w_row = 0;
    logic [KW-1:0]        w_col = 0;
    logic signed [23:0]   w_data = 0;
    logic                 dz_we = 0;
    logic [RW-1:0]        dz_row = 0;
    logic signed [23:0]   dz_data = 0;
    logic                 a_we = 0;
    logic [KW-1:0]        a_addr = 0;
    logic [15:0]          a_data = 0;
    logic [KW-1:0]        y_addr = 0;
    wire signed [23:0]    da_data, dzp_data;

    motor_matvec_t #(.L(L), .N_IN(N_IN), .N_OUT(N_OUT), .FW(20)) dut (
        .clk(clk), .rst(rst), .start(start), .done(done),
        .w_we(w_we), .w_row(w_row), .w_col(w_col), .w_data(w_data),
        .dz_we(dz_we), .dz_row(dz_row), .dz_data(dz_data),
        .a_we(a_we), .a_addr(a_addr), .a_data(a_data),
        .y_addr(y_addr), .da_data(da_data), .dzp_data(dzp_data)
    );

    // ---- BASURA en las RAM de todas las lanes antes de cargar los datos reales ----
    // Una RAM sin escribir vale 'X' en simulacion, lo que delata un error de forma
    // casual; en la placa contiene valores arbitrarios. Se rellena con aleatorios
    // (deterministas) para que una lane sin fila que NO aportara exactamente 0 falle
    // tambien en simulacion. La carga posterior sobreescribe solo las entradas reales.
    generate
        for (genvar g = 0; g < L; g = g + 1) begin : basura
            integer i;
            initial begin
                for (i = 0; i < (1 << dut.AW); i = i + 1) dut.lanes[g].u_lane.wmem[i]  = $random;
                for (i = 0; i < (1 << dut.BW); i = i + 1) dut.lanes[g].u_lane.dzmem[i] = $random;
            end
        end
    endgenerate

    // tablas doradas (un valor por linea, hex en complemento a 2)
    logic [23:0] wtab   [0:N_OUT*N_IN-1];
    logic [23:0] dztab  [0:N_OUT-1];
    logic [15:0] atab   [0:N_IN-1];
    logic [23:0] datab  [0:N_IN-1];
    logic [23:0] dzptab [0:N_IN-1];

    integer r, c, errores, ciclos, hayx;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_matvec_t.vcd");
            $dumpvars(0, tb_matvec_t);
        end
        $readmemh(F_W, wtab);
        $readmemh(F_DZ, dztab);
        $readmemh(F_A, atab);
        $readmemh(F_DA, datab);
        $readmemh(F_DZP, dzptab);

        // ---- guarda anti falso-PASS: archivo ausente/incompleto => 'X' en las tablas, y 'X !== X'
        //      es falso en Verilog (compararia X contra X y daria PASS) ----
        hayx = 0;
        for (r = 0; r < N_OUT*N_IN; r = r + 1) if (^wtab[r] === 1'bx) hayx = 1;
        for (r = 0; r < N_OUT; r = r + 1) if (^dztab[r] === 1'bx) hayx = 1;
        for (c = 0; c < N_IN; c = c + 1)
            if ((^atab[c] === 1'bx) || (^datab[c] === 1'bx) || (^dzptab[c] === 1'bx)) hayx = 1;
        if (hayx) begin
            $display("FALLO: algun archivo de vectores falta o esta incompleto (hay 'x' en las tablas)");
            $finish;
        end

        repeat (4) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // ---- carga (un valor por ciclo; se maneja en el flanco negativo) ----
        for (r = 0; r < N_OUT; r = r + 1)
            for (c = 0; c < N_IN; c = c + 1) begin
                @(negedge clk);
                w_we = 1; w_row = r; w_col = c; w_data = wtab[r*N_IN + c];
            end
        @(negedge clk); w_we = 0;

        for (r = 0; r < N_OUT; r = r + 1) begin
            @(negedge clk);
            dz_we = 1; dz_row = r; dz_data = dztab[r];
        end
        @(negedge clk); dz_we = 0;

        for (c = 0; c < N_IN; c = c + 1) begin
            @(negedge clk);
            a_we = 1; a_addr = c; a_data = atab[c];
        end
        @(negedge clk); a_we = 0;

        // ---- arranque y espera ----
        @(negedge clk); start = 1;
        @(negedge clk); start = 0;
        ciclos = 0;
        while (!done && ciclos < 200000) begin
            @(negedge clk);
            ciclos = ciclos + 1;
        end
        if (!done) begin
            $display("FALLO: timeout, 'done' nunca se activo");
            $finish;
        end

        // ---- comparacion bit a bit, EN CUANTO 'done' es visible, orden DESCENDENTE ----
        errores = 0;
        for (c = N_IN - 1; c >= 0; c = c - 1) begin
            y_addr = c;
            #1;
            if (da_data !== datab[c] || dzp_data !== dzptab[c]) begin
                if (errores < 10)
                    $display("  col %0d: dA=%h (esperado %h)  dz_prev=%h (esperado %h)",
                             c, da_data, datab[c], dzp_data, dzptab[c]);
                errores = errores + 1;
            end
        end

        $display("L=%0d N_IN=%0d N_OUT=%0d  ciclos(simulados)=%0d  columnas con error=%0d / %0d",
                 L, N_IN, N_OUT, ciclos, errores, N_IN);
        if (errores == 0) $display("PASS: dA y dz_prev coinciden bit a bit con el emulador");
        else              $display("FAIL");
        $finish;
    end
endmodule