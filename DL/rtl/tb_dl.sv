// ============================================================
// tb_dl.sv  -  testbench del motor integrado (motor_dl). Hace de "CPU": carga el estado inicial por
// el puerto ld_*, y por cada imagen emite los comandos
//     LOADX; MATVEC 1,2,3; DELTA_OUT; MATVEC_T 3,2; UPDATE 3,2,1
// durante EPOCHS epocas de N imagenes, y lo compara con el emulador de punto fijo
// (../src/emulador fixed EPOCHS --n=N --pesos=... --redondeo=... --vectores=PRE):
//   NIVEL 1 (depuracion, imagen 0 de la epoca 0): a1, a2, a3, dz3, dz2, dz1, y W, b y estado del
//            LFSR de las 3 capas tras el UPDATE. BIT A BIT.
//   NIVEL 2: totales por epoca (suma de e^2 y aciertos) y el HASH FNV-1a de los enteros de
//            W1,b1,W2,b2,W3,b3 al final del entrenamiento (el de checksum.h / emulador).
// Los pesos/dz/a se leen por REFERENCIA JERARQUICA (solo simulacion). SIMULACION: los ciclos no son
// tiempo real. Parametros: ver abajo; los .hex los genera el emulador con --vectores=PRE.
// ============================================================
`timescale 1ns/1ps

module tb_dl;
    parameter L      = 32;
    parameter N      = 100;          // imagenes por epoca (igual que --n= del emulador)
    parameter EPOCHS = 2;
    parameter RMODE  = 1;            // 0 cercano, 1 estocastico, 2 piso (igual que --redondeo=)
    parameter CHECK0 = 1;            // 1: verificar el detalle de la imagen 0 / epoca 0
    parameter PRE    = "e6";         // prefijo de los vectores
    parameter LUTF   = "e6_lut.hex";

    // offsets canonicos (orden del hash): W1(128x784) W2(64x128) W3(10x64)
    localparam W1O = 0, W2O = 128*784, W3O = 128*784 + 64*128, WTOT = 128*784 + 64*128 + 10*64;
    localparam B1O = 0, B2O = 128, B3O = 192, BTOT = 202;

    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 1;

    // ---- DUT ----
    logic cmd_valid = 0; logic [3:0] cmd_op = 0; logic [1:0] cmd_lay = 0; logic [15:0] cmd_arg = 0;
    wire busy, done;
    logic ld_we = 0; logic [1:0] ld_kind = 0, ld_lay = 0; logic [7:0] ld_row = 0; logic [9:0] ld_col = 0;
    logic [31:0] ld_data = 0;
    wire [25:0] px_addr; logic [7:0] px_data; wire [15:0] lb_addr; logic [7:0] lb_data;
    wire [47:0] ep_e2; wire [15:0] ep_hits;
    wire [31:0] cyc_ldx, cyc_fwd, cyc_del, cyc_bwd, cyc_upd;

    motor_dl #(.L(L), .FW(20), .LUT_FILE(LUTF)) dut (
        .clk(clk), .rst(rst),
        .cmd_valid(cmd_valid), .cmd_op(cmd_op), .cmd_lay(cmd_lay), .cmd_arg(cmd_arg),
        .busy(busy), .done(done),
        .ld_we(ld_we), .ld_kind(ld_kind), .ld_lay(ld_lay), .ld_row(ld_row), .ld_col(ld_col), .ld_data(ld_data),
        .px_addr(px_addr), .px_data(px_data), .lb_addr(lb_addr), .lb_data(lb_data),
        .ep_e2(ep_e2), .ep_hits(ep_hits),
        .cyc_ldx(cyc_ldx), .cyc_fwd(cyc_fwd), .cyc_del(cyc_del), .cyc_bwd(cyc_bwd), .cyc_upd(cyc_upd),
        .hr_kind(2'd0), .hr_lay(2'd1), .hr_row(8'd0), .hr_col(10'd0), .hr_data()   // ventana del host: sin uso aqui
    );

    // ---- modelo de la SDRAM (lectura sincrona) ----
    logic [7:0] pix [0:N*784-1];
    logic [7:0] lbl [0:N-1];
    always @(posedge clk) begin
        px_data <= pix[px_addr];
        lb_data <= lbl[lb_addr];
    end

    // ---- estado inicial y esperado ----
    logic [23:0] wpre  [0:WTOT-1];   logic [23:0] bpre  [0:BTOT-1];   logic [31:0] lfpre  [0:BTOT-1];
    logic [23:0] wpost [0:WTOT-1];   logic [23:0] bpost [0:BTOT-1];   logic [31:0] lfpost [0:BTOT-1];
    logic [15:0] a1x [0:127]; logic [15:0] a2x [0:63]; logic [15:0] a3x [0:9];
    logic [23:0] dz3x [0:9];  logic [23:0] dz2x [0:63]; logic [23:0] dz1x [0:127];
    logic [63:0] epx [0:2*EPOCHS-1];
    logic [63:0] hmem [0:0];
    // ---- captura del estado de las lanes ----
    logic [23:0] wcap [0:WTOT-1];   logic [23:0] bcap [0:BTOT-1];   logic [31:0] lfcap [0:BTOT-1];
    logic [23:0] dzcap [0:BTOT-1];
    reg cap = 0;

    generate
        for (genvar g = 0; g < L; g = g + 1) begin : capg
            integer pp, kk, r, ii;
            // BASURA en las RAM de las lanes (una lane que leyera/escribiera donde no debe fallaria)
            initial begin
                for (ii = 0; ii < (1 << dut.AW); ii = ii + 1) dut.lanes[g].u_lane.wmem[ii] = $random;
                for (ii = 0; ii < (1 << dut.BW); ii = ii + 1) begin
                    dut.lanes[g].u_lane.dzmem[ii] = $random;
                    dut.lanes[g].u_lane.bmem[ii]  = $random;
                    dut.lanes[g].u_lane.lfmem[ii] = $random;
                end
            end
            always @(posedge cap) begin
                for (pp = 0; pp < dut.P1; pp = pp + 1) begin
                    r = pp*L + g;
                    if (r < 128) begin
                        for (kk = 0; kk < 784; kk = kk + 1) wcap[W1O + r*784 + kk] = dut.lanes[g].u_lane.wmem[pp*784 + kk];
                        bcap[B1O + r] = dut.lanes[g].u_lane.bmem[pp];   lfcap[B1O + r] = dut.lanes[g].u_lane.lfmem[pp];
                        dzcap[B1O + r] = dut.lanes[g].u_lane.dzmem[pp];
                    end
                end
                for (pp = 0; pp < dut.P2; pp = pp + 1) begin
                    r = pp*L + g;
                    if (r < 64) begin
                        for (kk = 0; kk < 128; kk = kk + 1) wcap[W2O + r*128 + kk] = dut.lanes[g].u_lane.wmem[dut.WB2 + pp*128 + kk];
                        bcap[B2O + r] = dut.lanes[g].u_lane.bmem[dut.P1 + pp];   lfcap[B2O + r] = dut.lanes[g].u_lane.lfmem[dut.P1 + pp];
                        dzcap[B2O + r] = dut.lanes[g].u_lane.dzmem[dut.P1 + pp];
                    end
                end
                for (pp = 0; pp < dut.P3; pp = pp + 1) begin
                    r = pp*L + g;
                    if (r < 10) begin
                        for (kk = 0; kk < 64; kk = kk + 1) wcap[W3O + r*64 + kk] = dut.lanes[g].u_lane.wmem[dut.WB3 + pp*64 + kk];
                        bcap[B3O + r] = dut.lanes[g].u_lane.bmem[dut.P1 + dut.P2 + pp];   lfcap[B3O + r] = dut.lanes[g].u_lane.lfmem[dut.P1 + dut.P2 + pp];
                        dzcap[B3O + r] = dut.lanes[g].u_lane.dzmem[dut.P1 + dut.P2 + pp];
                    end
                end
            end
        end
    endgenerate
    integer ia;
    initial for (ia = 0; ia < 986; ia = ia + 1) dut.amem[ia] = $random;      // basura tambien en amem

    // ---- comandos ----
    integer guard;
    task automatic cmd(input [3:0] op, input [1:0] lay, input [15:0] arg);
        begin
            @(negedge clk); cmd_valid = 1; cmd_op = op; cmd_lay = lay; cmd_arg = arg;
            @(negedge clk); cmd_valid = 0;
            guard = 0;
            while (!done && guard < 4000000) begin @(negedge clk); guard = guard + 1; end
            if (!done) begin $display("FALLO: timeout en el comando op=%0d lay=%0d", op, lay); $finish; end
        end
    endtask

    // ---- hash FNV-1a (checksum.h) de los enteros de W y b, en el orden W1,b1,W2,b2,W3,b3 ----
    function automatic [63:0] fnv32(input [63:0] h0, input [31:0] v);
        integer bi; reg [63:0] h;
        begin
            h = h0;
            for (bi = 0; bi < 4; bi = bi + 1) h = (h ^ {56'b0, v[8*bi +: 8]}) * 64'd1099511628211;
            fnv32 = h;
        end
    endfunction
    function automatic [63:0] hash_estado(input integer dummy);
        integer i; reg [63:0] h;
        begin
            h = 64'd1469598103934665603;
            for (i = 0; i < 128*784; i = i + 1) h = fnv32(h, {{8{wcap[W1O+i][23]}}, wcap[W1O+i]});
            for (i = 0; i < 128;     i = i + 1) h = fnv32(h, {{8{bcap[B1O+i][23]}}, bcap[B1O+i]});
            for (i = 0; i < 64*128;  i = i + 1) h = fnv32(h, {{8{wcap[W2O+i][23]}}, wcap[W2O+i]});
            for (i = 0; i < 64;      i = i + 1) h = fnv32(h, {{8{bcap[B2O+i][23]}}, bcap[B2O+i]});
            for (i = 0; i < 10*64;   i = i + 1) h = fnv32(h, {{8{wcap[W3O+i][23]}}, wcap[W3O+i]});
            for (i = 0; i < 10;      i = i + 1) h = fnv32(h, {{8{bcap[B3O+i][23]}}, bcap[B3O+i]});
            hash_estado = h;
        end
    endfunction

    integer i, j, ep, im, eA1, eA2, eA3, eD3, eD2, eD1, eW1, eW2, eW3, eB, eL, eE, hayx, nerr;
    reg [63:0] hh;
    string fn;

    initial begin
        // ---------------- lectura de archivos ----------------
        fn = $sformatf("%s_img.hex", PRE); $readmemh(fn, pix);
        fn = $sformatf("%s_lbl.hex", PRE); $readmemh(fn, lbl);
        fn = $sformatf("%s_W1_pre.hex", PRE); $readmemh(fn, wpre, W1O, W2O-1);
        fn = $sformatf("%s_W2_pre.hex", PRE); $readmemh(fn, wpre, W2O, W3O-1);
        fn = $sformatf("%s_W3_pre.hex", PRE); $readmemh(fn, wpre, W3O, WTOT-1);
        fn = $sformatf("%s_b1_pre.hex", PRE); $readmemh(fn, bpre, B1O, B2O-1);
        fn = $sformatf("%s_b2_pre.hex", PRE); $readmemh(fn, bpre, B2O, B3O-1);
        fn = $sformatf("%s_b3_pre.hex", PRE); $readmemh(fn, bpre, B3O, BTOT-1);
        fn = $sformatf("%s_lf1_pre.hex", PRE); $readmemh(fn, lfpre, B1O, B2O-1);
        fn = $sformatf("%s_lf2_pre.hex", PRE); $readmemh(fn, lfpre, B2O, B3O-1);
        fn = $sformatf("%s_lf3_pre.hex", PRE); $readmemh(fn, lfpre, B3O, BTOT-1);
        fn = $sformatf("%s_epocas.hex", PRE); $readmemh(fn, epx);
        fn = $sformatf("%s_hash.hex", PRE);   $readmemh(fn, hmem);
        if (CHECK0) begin
            fn = $sformatf("%s_W1_post.hex", PRE); $readmemh(fn, wpost, W1O, W2O-1);
            fn = $sformatf("%s_W2_post.hex", PRE); $readmemh(fn, wpost, W2O, W3O-1);
            fn = $sformatf("%s_W3_post.hex", PRE); $readmemh(fn, wpost, W3O, WTOT-1);
            fn = $sformatf("%s_b1_post.hex", PRE); $readmemh(fn, bpost, B1O, B2O-1);
            fn = $sformatf("%s_b2_post.hex", PRE); $readmemh(fn, bpost, B2O, B3O-1);
            fn = $sformatf("%s_b3_post.hex", PRE); $readmemh(fn, bpost, B3O, BTOT-1);
            fn = $sformatf("%s_lf1_post.hex", PRE); $readmemh(fn, lfpost, B1O, B2O-1);
            fn = $sformatf("%s_lf2_post.hex", PRE); $readmemh(fn, lfpost, B2O, B3O-1);
            fn = $sformatf("%s_lf3_post.hex", PRE); $readmemh(fn, lfpost, B3O, BTOT-1);
            fn = $sformatf("%s_a1.hex", PRE); $readmemh(fn, a1x);
            fn = $sformatf("%s_a2.hex", PRE); $readmemh(fn, a2x);
            fn = $sformatf("%s_a3.hex", PRE); $readmemh(fn, a3x);
            fn = $sformatf("%s_dz3.hex", PRE); $readmemh(fn, dz3x);
            fn = $sformatf("%s_dz2.hex", PRE); $readmemh(fn, dz2x);
            fn = $sformatf("%s_dz1.hex", PRE); $readmemh(fn, dz1x);
        end

        // ---- guarda anti falso-PASS: un archivo ausente deja 'X' y 'X !== X' es falso ----
        hayx = 0;
        for (i = 0; i < N*784; i = i + 1) if (^pix[i] === 1'bx) hayx = 1;
        for (i = 0; i < N; i = i + 1) if (^lbl[i] === 1'bx) hayx = 1;
        for (i = 0; i < WTOT; i = i + 1) if (^wpre[i] === 1'bx) hayx = 1;
        for (i = 0; i < BTOT; i = i + 1) if ((^bpre[i] === 1'bx) || (^lfpre[i] === 1'bx)) hayx = 1;
        for (i = 0; i < 2*EPOCHS; i = i + 1) if (^epx[i] === 1'bx) hayx = 1;
        if (^hmem[0] === 1'bx) hayx = 1;
        if (CHECK0) begin
            for (i = 0; i < WTOT; i = i + 1) if (^wpost[i] === 1'bx) hayx = 1;
            for (i = 0; i < BTOT; i = i + 1) if ((^bpost[i] === 1'bx) || (^lfpost[i] === 1'bx)) hayx = 1;
            for (i = 0; i < 128; i = i + 1) if ((^a1x[i] === 1'bx) || (^dz1x[i] === 1'bx)) hayx = 1;
            for (i = 0; i < 64; i = i + 1)  if ((^a2x[i] === 1'bx) || (^dz2x[i] === 1'bx)) hayx = 1;
            for (i = 0; i < 10; i = i + 1)  if ((^a3x[i] === 1'bx) || (^dz3x[i] === 1'bx)) hayx = 1;
        end
        if (hayx) begin $display("FALLO: algun archivo de vectores falta o esta incompleto (hay 'x')"); $finish; end

        repeat (4) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // ---------------- carga inicial por el puerto del host ----------------
        for (j = 1; j <= 3; j = j + 1) begin : carga_capa
            integer nr, nc, wo, bo;
            nr = (j == 1) ? 128 : (j == 2) ? 64 : 10;
            nc = (j == 1) ? 784 : (j == 2) ? 128 : 64;
            wo = (j == 1) ? W1O : (j == 2) ? W2O : W3O;
            bo = (j == 1) ? B1O : (j == 2) ? B2O : B3O;
            for (i = 0; i < nr; i = i + 1) begin
                for (ia = 0; ia < nc; ia = ia + 1) begin
                    @(negedge clk);
                    ld_we = 1; ld_kind = 0; ld_lay = j; ld_row = i; ld_col = ia; ld_data = {8'b0, wpre[wo + i*nc + ia]};
                end
                @(negedge clk); ld_kind = 1; ld_row = i; ld_data = {8'b0, bpre[bo + i]};
                @(negedge clk); ld_kind = 2; ld_row = i; ld_data = lfpre[bo + i];
            end
            @(negedge clk); ld_we = 0;
        end
        cmd(4'd7, 2'd0, RMODE);                       // SETMODE

        // ---------------- entrenamiento ----------------
        nerr = 0;
        for (ep = 0; ep < EPOCHS; ep = ep + 1) begin
            cmd(4'd6, 2'd0, 0);                       // CLRSTAT
            for (im = 0; im < N; im = im + 1) begin
                cmd(4'd1, 2'd0, im);                  // LOADX
                cmd(4'd2, 2'd1, 0); cmd(4'd2, 2'd2, 0); cmd(4'd2, 2'd3, 0);   // forward
                cmd(4'd3, 2'd0, 0);                   // DELTA_OUT
                cmd(4'd4, 2'd3, 0); cmd(4'd4, 2'd2, 0);                       // backward (pesos viejos)
                if (CHECK0 && ep == 0 && im == 0) begin     // detalle ANTES de actualizar: a, dz
                    eA1 = 0; eA2 = 0; eA3 = 0; eD3 = 0; eD2 = 0; eD1 = 0;
                    cap = 0; #1; cap = 1; #1;
                    for (i = 0; i < 128; i = i + 1) begin
                        if (dut.amem[784 + i] !== a1x[i]) eA1 = eA1 + 1;
                        if (dzcap[B1O + i] !== dz1x[i]) eD1 = eD1 + 1;
                    end
                    for (i = 0; i < 64; i = i + 1) begin
                        if (dut.amem[912 + i] !== a2x[i]) eA2 = eA2 + 1;
                        if (dzcap[B2O + i] !== dz2x[i]) eD2 = eD2 + 1;
                    end
                    for (i = 0; i < 10; i = i + 1) begin
                        if (dut.amem[976 + i] !== a3x[i]) eA3 = eA3 + 1;
                        if (dzcap[B3O + i] !== dz3x[i]) eD3 = eD3 + 1;
                    end
                    $display("NIVEL 1 (imagen 0, antes de UPDATE): errores a1=%0d a2=%0d a3=%0d | dz3=%0d dz2=%0d dz1=%0d",
                             eA1, eA2, eA3, eD3, eD2, eD1);
                    nerr = nerr + eA1 + eA2 + eA3 + eD3 + eD2 + eD1;
                end
                cmd(4'd5, 2'd3, 0); cmd(4'd5, 2'd2, 0); cmd(4'd5, 2'd1, 0);   // UPDATE 3,2,1
                if (CHECK0 && ep == 0 && im == 0) begin     // detalle DESPUES de actualizar: W, b, LFSR
                    eW1 = 0; eW2 = 0; eW3 = 0; eB = 0; eL = 0;
                    cap = 0; #1; cap = 1; #1;
                    for (i = 0; i < 128*784; i = i + 1) if (wcap[W1O + i] !== wpost[W1O + i]) eW1 = eW1 + 1;
                    for (i = 0; i < 64*128;  i = i + 1) if (wcap[W2O + i] !== wpost[W2O + i]) eW2 = eW2 + 1;
                    for (i = 0; i < 10*64;   i = i + 1) if (wcap[W3O + i] !== wpost[W3O + i]) eW3 = eW3 + 1;
                    for (i = 0; i < BTOT; i = i + 1) begin
                        if (bcap[i]  !== bpost[i])  eB = eB + 1;
                        if (lfcap[i] !== lfpost[i]) eL = eL + 1;
                    end
                    $display("NIVEL 1 (imagen 0, tras UPDATE): errores W1=%0d W2=%0d W3=%0d b=%0d LFSR=%0d", eW1, eW2, eW3, eB, eL);
                    nerr = nerr + eW1 + eW2 + eW3 + eB + eL;
                end
            end
            eE = ({16'b0, ep_e2} !== epx[2*ep]) || ({48'b0, ep_hits} !== epx[2*ep+1]);
            $display("epoca %0d: e2=%0d (esperado %0d)  aciertos=%0d (esperado %0d)  %s", ep, ep_e2, epx[2*ep], ep_hits, epx[2*ep+1],
                     eE ? "DIFIERE" : "ok");
            nerr = nerr + eE;
        end

        // ---------------- hash final ----------------
        cap = 0; #1; cap = 1; #1;
        hh = hash_estado(0);
        $display("hash enteros RTL = %h   emulador = %h   %s", hh, hmem[0], (hh === hmem[0]) ? "IGUAL" : "DIFIERE");
        if (hh !== hmem[0]) nerr = nerr + 1;
        $display("ciclos(simulados) totales: LOADX=%0d MATVEC=%0d DELTA=%0d MATVEC_T=%0d UPDATE=%0d  (por imagen: %0d)",
                 cyc_ldx, cyc_fwd, cyc_del, cyc_bwd, cyc_upd,
                 (cyc_ldx + cyc_fwd + cyc_del + cyc_bwd + cyc_upd) / N);
        $display("L=%0d N=%0d EPOCHS=%0d rmode=%0d : errores=%0d", L, N, EPOCHS, RMODE, nerr);
        if (nerr == 0) $display("PASS: el motor integrado coincide bit a bit con el emulador");
        else           $display("FAIL");
        $finish;
    end
endmodule
