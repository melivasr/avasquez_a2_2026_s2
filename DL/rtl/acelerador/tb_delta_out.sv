// ============================================================
// tb_delta_out.sv
//
// Prueba DELTA_OUT contra la traza del emulador
// (./emulador fixed EPOCAS --n=N --vectores=PREF  ->  PREF_delta.hex).
// Cada linea del archivo es UNA imagen (una evaluacion de DELTA_OUT) y trae
// la etiqueta, a3[0..9] y los valores esperados dz3[0..9], e2 y pred.
// Comparacion BIT A BIT de: dz3 (10 valores), e2_sum, pred y hit.
//
// Formato de la linea (112 digitos hex = 448 bits, primer campo = MSB):
//   [447:440] etiqueta  [439:436] pred  [435:400] e2
//   [399:160] dz3[0..9] (24 bits c/u, dz3[0] el mas alto)
//   [159:0]   a3[0..9]  (16 bits c/u, a3[0] el mas alto)
//
// SIMULACION: los ciclos no son tiempo real.
// ============================================================
`timescale 1ns/1ps

module tb_delta_out;
    parameter NVEC  = 200;                 // numero de lineas del archivo
    parameter F_VEC = "vd_delta.hex";

    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 1, start = 0;
    logic [3:0] label = 0;

    wire  [3:0]         rd_addr;
    logic [15:0]        rd_data;
    wire                wr_en;
    wire  [3:0]         wr_addr;
    wire signed [23:0]  wr_data;
    wire                done;
    wire  [35:0]        e2_sum;
    wire  [3:0]         pred;
    wire                hit;

    delta_out dut (.*);

    // Modelo de RAM sincrona de a3 (lectura con 1 ciclo de latencia) y
    // captura de dz3 como lo haria una RAM sincrona (al flanco positivo).
    logic [15:0]        a_mem  [0:9];
    logic signed [23:0] dz_mem [0:9];
    always @(posedge clk) rd_data <= a_mem[rd_addr];
    always @(posedge clk) if (wr_en) dz_mem[wr_addr] <= wr_data;

    logic [447:0] vec [0:NVEC-1];
    logic [447:0] w;
    integer v, k, errores, malos, ciclos, ciclos_max;
    logic [3:0]   lab_e, pred_e;
    logic [35:0]  e2_e;
    logic [23:0]  dz_e;
    logic         vec_mal;

    initial begin
        $readmemh(F_VEC, vec);
        errores = 0; malos = 0; ciclos_max = 0;
        repeat (6) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        for (v = 0; v < NVEC; v = v + 1) begin
            w      = vec[v];
            lab_e  = w[443:440];
            pred_e = w[439:436];
            e2_e   = w[435:400];
            for (k = 0; k < 10; k = k + 1) begin
                a_mem[k]  = w[159 - 16*k -: 16];
                dz_mem[k] = 'x;                       // detecta escrituras faltantes
            end

            @(negedge clk); label = lab_e; start = 1;
            @(negedge clk); start = 0;
            ciclos = 0;
            // se compara EN CUANTO 'done' es visible (sin ciclos de gracia)
            while (!done && ciclos < 200) begin
                @(negedge clk);
                ciclos = ciclos + 1;
            end
            if (ciclos > ciclos_max) ciclos_max = ciclos;

            vec_mal = (!done);
            for (k = 0; k < 10; k = k + 1) begin
                dz_e = w[399 - 24*k -: 24];
                if (dz_mem[k] !== dz_e) begin
                    vec_mal = 1;
                    if (errores < 8) $display("  vec %0d dz3[%0d]=%h esperado %h", v, k, dz_mem[k], dz_e);
                    errores = errores + 1;
                end
            end
            if (e2_sum !== e2_e) begin
                vec_mal = 1;
                if (errores < 8) $display("  vec %0d e2=%h esperado %h", v, e2_sum, e2_e);
                errores = errores + 1;
            end
            if (pred !== pred_e) begin
                vec_mal = 1;
                if (errores < 8) $display("  vec %0d pred=%0d esperado %0d", v, pred, pred_e);
                errores = errores + 1;
            end
            if (hit !== (pred_e == lab_e)) begin
                vec_mal = 1;
                if (errores < 8) $display("  vec %0d hit=%b esperado %b", v, hit, (pred_e == lab_e));
                errores = errores + 1;
            end
            if (vec_mal) malos = malos + 1;
        end

        $display("DELTA_OUT: %0d vectores, vectores con error=%0d, ciclos por imagen=%0d (simulados)",
                 NVEC, malos, ciclos_max);
        if (malos == 0) $display("PASS: dz3, e2, pred y hit coinciden bit a bit con el emulador");
        else            $display("FAIL");
        $finish;
    end
endmodule
