// ============================================================
// tb_matvec.v
//
// Testbench de motor_matvec contra los vectores dorados del emulador
// (./emulador fixed ... --vectores=vec). Compara BIT A BIT, para cada
// neurona de la capa:
//     z (Q4.12)  contra  vec_zN.hex
//     a (Q0.16)  contra  vec_aN.hex
//
// Sirve para las 3 capas cambiando parametros (-P de Icarus):
//   capa 1: N_IN=784 N_OUT=128 F_W=vec_W1_pre.hex F_B=vec_b1_pre.hex F_X=vec_x.hex  F_Z=vec_z1.hex F_A=vec_a1.hex
//   capa 2: N_IN=128 N_OUT=64  F_W=vec_W2_pre.hex F_B=vec_b2_pre.hex F_X=vec_a1.hex F_Z=vec_z2.hex F_A=vec_a2.hex
//   capa 3: N_IN=64  N_OUT=10  F_W=vec_W3_pre.hex F_B=vec_b3_pre.hex F_X=vec_a2.hex F_Z=vec_z3.hex F_A=vec_a3.hex
// y L (numero de lanes) tambien es parametro: el resultado debe ser el
// mismo para cualquier L.
//
// Esto es SIMULACION: los ciclos que imprime no son tiempo real.
// ============================================================
`timescale 1ns/1ps

module tb_matvec;
    parameter L     = 32;
    parameter N_IN  = 784;
    parameter N_OUT = 128;
    parameter F_LUT = "vec_lut.hex";
    parameter F_W   = "vec_W1_pre.hex";
    parameter F_B   = "vec_b1_pre.hex";
    parameter F_X   = "vec_x.hex";
    parameter F_Z   = "vec_z1.hex";
    parameter F_A   = "vec_a1.hex";

    localparam KW = $clog2(N_IN);
    localparam RW = $clog2(N_OUT);

    // ---------------- reloj y reset ----------------
    logic clk = 0;
    always #5 clk = ~clk;          // 100 MHz simulados (valor arbitrario)
    logic rst = 1;

    // ---------------- senales hacia el DUT ----------------
    logic                  start = 0;
    wire                 done;
    logic                  x_we = 0;
    logic  [KW-1:0]        x_waddr = 0;
    logic  [15:0]          x_wdata = 0;
    logic                  w_we = 0;
    logic  [RW-1:0]        w_row = 0;
    logic  [KW-1:0]        w_col = 0;
    logic  signed [23:0]   w_data = 0;
    logic                  b_we = 0;
    logic  [RW-1:0]        b_row = 0;
    logic  signed [23:0]   b_data = 0;
    logic  [RW-1:0]        y_addr = 0;
    wire [15:0]          y_data, zy_data;

    motor_matvec #(
        .L(L), .N_IN(N_IN), .N_OUT(N_OUT), .FW(20), .LUT_FILE(F_LUT)
    ) dut (
        .clk(clk), .rst(rst), .start(start), .done(done),
        .x_we(x_we), .x_waddr(x_waddr), .x_wdata(x_wdata),
        .w_we(w_we), .w_row(w_row), .w_col(w_col), .w_data(w_data),
        .b_we(b_we), .b_row(b_row), .b_data(b_data),
        .y_addr(y_addr), .y_data(y_data), .zy_data(zy_data)
    );

    // ---------------- tablas doradas ----------------
    logic [23:0] wtab [0:N_OUT*N_IN-1];     // W en orden fila
    logic [23:0] btab [0:N_OUT-1];
    logic [15:0] xtab [0:N_IN-1];
    logic [15:0] ztab [0:N_OUT-1];
    logic [15:0] atab [0:N_OUT-1];

    integer r, c, errores, ciclos, t;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_matvec.vcd");
            $dumpvars(0, tb_matvec);
        end

        $readmemh(F_W, wtab);
        $readmemh(F_B, btab);
        $readmemh(F_X, xtab);
        $readmemh(F_Z, ztab);
        $readmemh(F_A, atab);

        if ((^wtab[0] === 1'bx) || (^wtab[N_OUT*N_IN-1] === 1'bx) ||
            (^btab[0] === 1'bx) || (^xtab[N_IN-1] === 1'bx) ||
            (^ztab[N_OUT-1] === 1'bx) || (^atab[N_OUT-1] === 1'bx)) begin
            $display("FALLO: no se cargaron los .hex (revisar nombres y carpeta)");
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
            b_we = 1; b_row = r; b_data = btab[r];
        end
        @(negedge clk); b_we = 0;

        for (c = 0; c < N_IN; c = c + 1) begin
            @(negedge clk);
            x_we = 1; x_waddr = c; x_wdata = xtab[c];
        end
        @(negedge clk); x_we = 0;

        // ---- arranque y espera ----
        @(negedge clk); start = 1;
        @(negedge clk); start = 0;
        ciclos = 0;
        // Se muestrea en el flanco negativo y se compara EN CUANTO 'done' es
        // visible, sin ciclos de gracia: asi se detecta un 'done' prematuro.
        while (!done && ciclos < 200000) begin
            @(negedge clk);
            ciclos = ciclos + 1;
        end

        if (!done) begin
            $display("FALLO: timeout, 'done' nunca se activo");
            $finish;
        end

        // ---- comparacion bit a bit ----
        // Orden DESCENDENTE: la ultima fila (N_OUT-1) es la ultima que escribe el
        // motor, asi que se verifica en cuanto 'done' es visible. Si 'done' se
        // activara antes de tiempo, esa fila aun estaria desactualizada.
        errores = 0;
        for (r = N_OUT - 1; r >= 0; r = r - 1) begin
            y_addr = r;
            #1;
            if (zy_data !== ztab[r] || y_data !== atab[r]) begin
                if (errores < 10)
                    $display("  fila %0d: z=%h (esperado %h)  a=%h (esperado %h)",
                             r, zy_data, ztab[r], y_data, atab[r]);
                errores = errores + 1;
            end
        end

        $display("L=%0d N_IN=%0d N_OUT=%0d  ciclos(simulados)=%0d  filas con error=%0d / %0d",
                 L, N_IN, N_OUT, ciclos, errores, N_OUT);
        if (errores == 0) $display("PASS: z y a coinciden bit a bit con el emulador");
        else              $display("FAIL");
        $finish;
    end
endmodule