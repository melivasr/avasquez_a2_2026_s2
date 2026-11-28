// ============================================================
// tb_dl_bus.sv  -  el motor integrado visto DESDE EL BUS. Hace de maestro Avalon-MM (lo que haran el
// CPU RV32I y el JTAG): toda la interaccion con dl_avalon es por lecturas/escrituras de registros.
//   - carga inicial: LD_ADDR + LD_DATA con autoincremento (W, b y estado del LFSR de las 3 capas)
//   - cada operacion: ARG + CMD (palabra tipo custom-0) y sondeo de STATUS hasta busy=0 y done=1
//   - estadisticas por epoca: EP_E2_LO/HI y EP_HITS ; contadores de ciclos por operacion
//   - al final, los pesos se RELEEN por la ventana RD_ADDR/RD_DATA y se calcula el hash FNV-1a de los
//     enteros (el del emulador / checksum.h) -> valida el camino de lectura del host (D3-A)
//   - CHECK0: tras la imagen 0 compara por la ventana dz (kind 3), y despues de UPDATE W, b y LFSR
//   - DIRECTED: pruebas de los errores del bus (opcode, CMD/LD_DATA/RD_DATA con el motor ocupado, W1C,
//     ID, CFG y CLRSTAT sobre CYC_BUSY)
// Vectores: ../src/emulador fixed EPOCHS --n=N ... --vectores=PRE  (igual que tb_dl.sv). SIMULACION.
// ============================================================
`timescale 1ns/1ps

module tb_dl_bus;
    parameter L        = 32;
    parameter N        = 2;
    parameter EPOCHS   = 1;
    parameter RMODE    = 1;
    parameter CHECK0   = 1;
    parameter DIRECTED = 1;
    parameter PRE      = "m2";
    parameter LUTF     = "m2_lut.hex";
    parameter USE_ROM  = 0;   // 1: el dataset lo sirve rom_dataset en vez del modelo del testbench
    wire [7:0] rom_px, rom_lb;

    localparam W1O = 0, W2O = 128*784, W3O = 128*784 + 64*128, WTOT = 128*784 + 64*128 + 10*64;
    localparam B1O = 0, B2O = 128, B3O = 192, BTOT = 202;
    localparam [4:0] A_ID = 5'h00, A_CMD = 5'h01, A_ARG = 5'h02, A_STATUS = 5'h03, A_CYC_BUSY = 5'h04,
                     A_CYC_LDX = 5'h05, A_CYC_FWD = 5'h06, A_CYC_DEL = 5'h07, A_CYC_BWD = 5'h08,
                     A_CYC_UPD = 5'h09, A_E2LO = 5'h0A, A_E2HI = 5'h0B, A_HITS = 5'h0C, A_CFG = 5'h0D,
                     A_LDADDR = 5'h10, A_LDDATA = 5'h11, A_RDADDR = 5'h12, A_RDDATA = 5'h13;

    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 1;

    logic [4:0]  avs_address = 0;
    logic        avs_read = 0, avs_write = 0;
    logic [31:0] avs_writedata = 0;
    wire  [31:0] avs_readdata;
    wire         avs_waitrequest;
    wire  [25:0] px_addr;  logic [7:0] px_data;
    wire  [15:0] lb_addr;  logic [7:0] lb_data;

    dl_avalon #(.L(L), .FW(20), .LUT_FILE(LUTF)) dut (
        .clk(clk), .rst(rst),
        .avs_address(avs_address), .avs_read(avs_read), .avs_write(avs_write),
        .avs_writedata(avs_writedata), .avs_readdata(avs_readdata), .avs_waitrequest(avs_waitrequest),
        .px_addr(px_addr), .px_data(USE_ROM ? rom_px : px_data), .lb_addr(lb_addr), .lb_data(USE_ROM ? rom_lb : lb_data)
    );

    rom_dataset #(.NIMG(N), .IMG_FILE({PRE, "_img.hex"}), .LBL_FILE({PRE, "_lbl.hex"})) u_rom (
        .clk(clk), .px_addr(px_addr), .px_data(rom_px), .lb_addr(lb_addr), .lb_data(rom_lb));
    // ---- modelo de la SDRAM / ROM (lectura sincrona) ----
    logic [7:0] pix [0:N*784-1];
    logic [7:0] lbl [0:N-1];
    always @(posedge clk) begin px_data <= pix[px_addr]; lb_data <= lbl[lb_addr]; end

    // ---- tablas ----
    logic [23:0] wpre  [0:WTOT-1];  logic [23:0] bpre  [0:BTOT-1];  logic [31:0] lfpre  [0:BTOT-1];
    logic [23:0] wpost [0:WTOT-1];  logic [23:0] bpost [0:BTOT-1];  logic [31:0] lfpost [0:BTOT-1];
    logic [15:0] a1x [0:127]; logic [15:0] a2x [0:63]; logic [15:0] a3x [0:9];
    logic [23:0] dz3x [0:9];  logic [23:0] dz2x [0:63]; logic [23:0] dz1x [0:127];
    logic [63:0] epx [0:2*EPOCHS-1];
    logic [63:0] hmem [0:0];

    // ---- BASURA en las RAM de las lanes y en amem (se sobreescribe lo real al cargar) ----
    generate
        for (genvar g = 0; g < L; g = g + 1) begin : basura
            integer ii;
            initial begin
                for (ii = 0; ii < (1 << dut.u_motor.AW); ii = ii + 1) dut.u_motor.lanes[g].u_lane.wmem[ii] = $random;
                for (ii = 0; ii < (1 << dut.u_motor.BW); ii = ii + 1) begin
                    dut.u_motor.lanes[g].u_lane.dzmem[ii] = $random;
                    dut.u_motor.lanes[g].u_lane.bmem[ii]  = $random;
                    dut.u_motor.lanes[g].u_lane.lfmem[ii] = $random;
                end
            end
        end
    endgenerate
    integer ia;
    initial for (ia = 0; ia < 986; ia = ia + 1) dut.u_motor.amem[ia] = $random;

    // ---- transacciones de bus (maestro Avalon-MM) ----
    task automatic bus_wr(input [4:0] a, input [31:0] d);
        begin
            @(negedge clk); avs_address = a; avs_writedata = d; avs_write = 1;
            @(negedge clk); avs_write = 0;
        end
    endtask
    task automatic bus_rd(input [4:0] a, output [31:0] d);
        begin
            @(negedge clk); avs_address = a; avs_read = 1;
            @(posedge clk);
            while (avs_waitrequest) @(posedge clk);      // aceptada en este flanco
            @(negedge clk); d = avs_readdata; avs_read = 0;   // readLatency = 1
        end
    endtask
    function automatic [31:0] palabra_cmd(input [6:0] op, input [2:0] lay);
        palabra_cmd = {op, 5'd0, 5'd0, lay, 5'd0, 7'b0001011};        // formato custom-0
    endfunction
    integer guard;
    task automatic esperar_listo;
        reg [31:0] st;
        begin
            st = 32'd1; guard = 0;
            while ((st[0] || !st[1]) && guard < 4000000) begin bus_rd(A_STATUS, st); guard = guard + 1; end
            if (guard >= 4000000) begin $display("FALLO: timeout esperando STATUS.done"); $finish; end
        end
    endtask
    // CMD y lectura de STATUS en CICLOS CONSECUTIVOS (lo que puede hacer un CPU segmentado con sw seguido de lw):
    // el periferico debe mostrar busy=1 y done=0 ya en la primera lectura, aunque el motor aun no vea el comando.
    task automatic cmd_pegado(input [6:0] op, input [2:0] lay, input [15:0] arg, output [31:0] st1);
        begin
            bus_wr(A_ARG, {16'b0, arg});
            @(negedge clk); avs_address = A_CMD; avs_writedata = palabra_cmd(op, lay); avs_write = 1;
            @(negedge clk); avs_write = 0; avs_address = A_STATUS; avs_read = 1;     // lectura en el ciclo siguiente
            @(posedge clk);
            @(negedge clk); st1 = avs_readdata; avs_read = 0;
            esperar_listo;
        end
    endtask
    task automatic cmd(input [6:0] op, input [2:0] lay, input [15:0] arg);
        begin bus_wr(A_ARG, {16'b0, arg}); bus_wr(A_CMD, palabra_cmd(op, lay)); esperar_listo; end
    endtask

    // ---- hash FNV-1a (checksum.h) ----
    function automatic [63:0] fnv32(input [63:0] h0, input [31:0] v);
        integer bi; reg [63:0] h;
        begin
            h = h0;
            for (bi = 0; bi < 4; bi = bi + 1) h = (h ^ {56'b0, v[8*bi +: 8]}) * 64'd1099511628211;
            fnv32 = h;
        end
    endfunction

    // ---- valores esperados para comparar la ventana ----
    function automatic integer off_w(input [1:0] lay); off_w = (lay == 1) ? W1O : (lay == 2) ? W2O : W3O; endfunction
    function automatic integer off_b(input [1:0] lay); off_b = (lay == 1) ? B1O : (lay == 2) ? B2O : B3O; endfunction
    function automatic [31:0] esperado(input [1:0] kind, input [1:0] lay, input integer idx);
        reg [23:0] x;
        begin
            case (kind)
                2'd0: begin x = wpost[off_w(lay) + idx]; esperado = {{8{x[23]}}, x}; end
                2'd1: begin x = bpost[off_b(lay) + idx]; esperado = {{8{x[23]}}, x}; end
                2'd2: esperado = lfpost[off_b(lay) + idx];
                default: begin
                    x = (lay == 1) ? dz1x[idx] : (lay == 2) ? dz2x[idx] : dz3x[idx];
                    esperado = {{8{x[23]}}, x};
                end
            endcase
        end
    endfunction
    task automatic chk_ventana(input [1:0] kind, input [1:0] lay, input integer n, output integer errs);
        integer i3; reg [31:0] dd;
        begin
            errs = 0;
            bus_wr(A_RDADDR, {kind, lay, 8'd0, 10'd0});
            for (i3 = 0; i3 < n; i3 = i3 + 1) begin
                bus_rd(A_RDDATA, dd);
                if (dd !== esperado(kind, lay, i3)) errs = errs + 1;
            end
        end
    endtask

    task automatic cargar_capa(input integer j);
        integer nr, nc, wo, bo, i3;
        begin
            nr = (j == 1) ? 128 : (j == 2) ? 64 : 10;
            nc = (j == 1) ? 784 : (j == 2) ? 128 : 64;
            wo = (j == 1) ? W1O : (j == 2) ? W2O : W3O;
            bo = (j == 1) ? B1O : (j == 2) ? B2O : B3O;
            bus_wr(A_LDADDR, {2'd0, j[1:0], 8'd0, 10'd0});
            for (i3 = 0; i3 < nr*nc; i3 = i3 + 1) bus_wr(A_LDDATA, {8'b0, wpre[wo + i3]});
            bus_wr(A_LDADDR, {2'd1, j[1:0], 8'd0, 10'd0});
            for (i3 = 0; i3 < nr; i3 = i3 + 1) bus_wr(A_LDDATA, {8'b0, bpre[bo + i3]});
            bus_wr(A_LDADDR, {2'd2, j[1:0], 8'd0, 10'd0});
            for (i3 = 0; i3 < nr; i3 = i3 + 1) bus_wr(A_LDDATA, lfpre[bo + i3]);
        end
    endtask

    integer i, ep, im, nerr, hayx, e1, e2, e3, eb, el, eA1, eA2, eA3, ntest;
    reg [31:0] d, st;
    reg [63:0] hh;
    string fn;
    task automatic ck(input integer ok, input string que);
        begin
            ntest = ntest + 1;
            if (!ok) begin nerr = nerr + 1; $display("BUS: FALLA  %s", que); end
        end
    endtask

    initial begin
        nerr = 0; ntest = 0;
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
        // ---- guarda anti falso-PASS ('X !== X' es falso en Verilog) ----
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

        // ---------------- pruebas dirigidas del bus (motor en reposo) ----------------
        if (DIRECTED) begin
            bus_rd(A_ID, d);   ck(d === 32'h444C_0100, "ID");
            bus_rd(A_CFG, d);  ck(d === {8'd0, 8'd20, 16'(L)}, "CFG = {FW, L}");
            bus_rd(A_STATUS, st); ck(st === 32'd0, "STATUS tras reset = 0");

            // --- operacion 0 (invalida por cmd_valido); no aisla opcode ni funct7 ---
            bus_wr(A_CMD, 32'h0000_0001);
            bus_rd(A_STATUS, st); ck(st[2] === 1'b1, "CMD con operacion 0 pone err");
            bus_wr(A_STATUS, 32'h4);                            // W1C
            bus_rd(A_STATUS, st); ck(st[2] === 1'b0, "err se limpia escribiendo 1 (W1C)");
            bus_wr(A_CMD, {7'd16, 5'd0, 5'd0, 3'd0, 5'd0, 7'b0001011});   // tambien operacion 0
            bus_rd(A_STATUS, st); ck(st[2] === 1'b1, "CMD con operacion 0 (funct7=16) pone err");
            bus_wr(A_STATUS, 32'h4);

            // --- B1: operacion VALIDA (CLRSTAT) con opcode != 0001011: solo la comprobacion de opcode puede rechazarla ---
            bus_wr(A_CMD, {7'd6, 5'd0, 5'd0, 3'd0, 5'd0, 7'b0001111});
            bus_rd(A_STATUS, st);
            ck(st[2] === 1'b1 && st[0] === 1'b0, "operacion valida con opcode != 0001011 rechazada (err, sin busy)");
            bus_wr(A_STATUS, 32'h4);

            // --- B2: operacion VALIDA (CLRSTAT) con funct7[6:4] != 0 (funct7 = 22 = 001_0110): solo esa comprobacion la rechaza ---
            bus_wr(A_CMD, palabra_cmd(7'd22, 3'd0));
            bus_rd(A_STATUS, st);
            ck(st[2] === 1'b1 && st[0] === 1'b0, "operacion valida con funct7[6:4] != 0 rechazada (err, sin busy)");
            bus_wr(A_STATUS, 32'h4);
            // comandos mal formados: no deben llegar al motor (con una capa invalida el motor no terminaria nunca)
            begin : cmds_malos
                integer q; reg [31:0] pal;
                for (q = 0; q < 5; q = q + 1) begin
                    case (q)
                        0: pal = palabra_cmd(7'd2, 3'd0);                         // MATVEC con capa 0
                        1: pal = palabra_cmd(7'd4, 3'd1);                         // MATVEC_T con capa 1 (no hay dA0)
                        2: pal = palabra_cmd(7'd0, 3'd0);                         // operacion 0
                        3: pal = palabra_cmd(7'd8, 3'd0);                         // operacion 8
                        default: pal = palabra_cmd(7'd5, 3'd5);                   // UPDATE con funct3 = 5
                    endcase
                    bus_wr(A_CMD, pal);
                    bus_rd(A_STATUS, st);
                    ck(st[2] === 1'b1 && st[0] === 1'b0, "comando mal formado rechazado con err y sin ponerse busy");
                    bus_wr(A_STATUS, 32'h4);
                end
            end
            bus_wr(A_ARG, 32'h0000_BEEF);
            bus_rd(A_ARG, d);  ck(d === 32'h0000_BEEF, "ARG se lee como se escribio");
            bus_wr(A_LDADDR, {2'd3, 2'd1, 8'd0, 10'd0});            // kind 3 = dz: solo lectura
            bus_wr(A_LDDATA, 32'h0000_0001);
            bus_rd(A_STATUS, st); ck(st[2] === 1'b1, "LD_DATA con kind 3 (solo lectura) pone err");
            bus_wr(A_STATUS, 32'h4);
        end

        // ---------------- carga inicial por el bus ----------------
        cargar_capa(1); cargar_capa(2); cargar_capa(3);
        if (DIRECTED) begin
            bus_wr(A_LDADDR, {2'd0, 2'd2, 8'd5, 10'd7});
            bus_rd(A_LDADDR, d); ck(d[21:0] === {2'd0, 2'd2, 8'd5, 10'd7}, "LD_ADDR se lee como se escribio");
        end
        if (DIRECTED) begin                                      // lecturas de RD_DATA SIN esperar (maestro rapido)
            reg [31:0] d1, d2;
            bus_wr(A_RDADDR, {2'd0, 2'd1, 8'd3, 10'd10});        // W1[3][10]
            avs_address = A_RDDATA; avs_read = 1;                // lectura en el ciclo siguiente: el dato aun no esta listo
            @(posedge clk); while (avs_waitrequest) @(posedge clk);
            @(negedge clk); d1 = avs_readdata;                   // la lectura sigue activa: 2.a lectura pegada (W1[3][11])
            @(posedge clk); while (avs_waitrequest) @(posedge clk);
            @(negedge clk); d2 = avs_readdata; avs_read = 0;
            ck(d1 === {{8{wpre[W1O + 3*784 + 10][23]}}, wpre[W1O + 3*784 + 10]}, "RD_DATA pegada a RD_ADDR espera el dato (waitrequest)");
            ck(d2 === {{8{wpre[W1O + 3*784 + 11][23]}}, wpre[W1O + 3*784 + 11]}, "2.a lectura consecutiva de RD_DATA (autoincremento + waitrequest)");
        end
        if (DIRECTED) begin
            cmd(7'd7, 3'd0, RMODE);                              // el motor ya tiene un 'done' viejo
            cmd_pegado(7'd7, 3'd0, RMODE, st);                   // SETMODE con STATUS leido justo despues
            ck(st[0] === 1'b1, "STATUS.busy=1 en el ciclo siguiente a CMD (comando en vuelo)");
            ck(st[1] === 1'b0, "STATUS.done=0 en el ciclo siguiente a CMD (no se ve el done viejo)");
        end else
            cmd(7'd7, 3'd0, RMODE);                              // SETMODE

        // ---------------- entrenamiento ----------------
        if (DIRECTED) begin                                      // CYC_BUSY acumula y CLRSTAT lo limpia
            cmd(7'd1, 3'd0, 0);                                  // LOADX de calentamiento (~787 ciclos)
            bus_rd(A_CYC_BUSY, d); ck(d > 32'd700, "CYC_BUSY acumula los ciclos ocupados");
        end
        for (ep = 0; ep < EPOCHS; ep = ep + 1) begin
            cmd(7'd6, 3'd0, 0);                                 // CLRSTAT
            if (DIRECTED) begin bus_rd(A_CYC_BUSY, d); ck(d < 32'd16, "CLRSTAT pone CYC_BUSY a ~0"); end
            for (im = 0; im < N; im = im + 1) begin
                if (DIRECTED && ep == 0 && im == 0) begin
                    // LOADX con interferencias mientras el motor esta ocupado
                    bus_wr(A_ARG, 0);  bus_wr(A_CMD, palabra_cmd(7'd1, 3'd0));
                    bus_wr(A_CMD, palabra_cmd(7'd1, 3'd0));           // CMD con el motor ocupado
                    bus_rd(A_STATUS, st);
                    ck(st[2] === 1'b1, "CMD con el motor ocupado pone err");
                    ck(st[0] === 1'b1, "STATUS.busy durante LOADX");
                    bus_wr(A_STATUS, 32'h4);
                    bus_rd(A_RDDATA, d);                               // lectura con el motor ocupado
                    ck(d === 32'hBAD0_BAD0, "RD_DATA con el motor ocupado devuelve BAD0BAD0");
                    bus_rd(A_STATUS, st); ck(st[2] === 1'b1, "RD_DATA con el motor ocupado pone err");
                    bus_wr(A_STATUS, 32'h4);
                    bus_wr(A_LDDATA, 32'h1234_5678);                   // carga con el motor ocupado
                    bus_rd(A_STATUS, st); ck(st[2] === 1'b1, "LD_DATA con el motor ocupado pone err");
                    bus_wr(A_STATUS, 32'h4);
                    bus_rd(A_STATUS, st); ck(st[2] === 1'b0, "err limpio tras W1C");
                    esperar_listo;
                end else
                    cmd(7'd1, 3'd0, im);                              // LOADX
                cmd(7'd2, 3'd1, 0); cmd(7'd2, 3'd2, 0); cmd(7'd2, 3'd3, 0);     // forward
                cmd(7'd3, 3'd0, 0);                                 // DELTA_OUT
                cmd(7'd4, 3'd3, 0); cmd(7'd4, 3'd2, 0);             // backward (pesos viejos)
                if (CHECK0 && ep == 0 && im == 0) begin
                    eA1 = 0; eA2 = 0; eA3 = 0;
                    for (i = 0; i < 128; i = i + 1) if (dut.u_motor.amem[784 + i] !== a1x[i]) eA1 = eA1 + 1;
                    for (i = 0; i < 64;  i = i + 1) if (dut.u_motor.amem[912 + i] !== a2x[i]) eA2 = eA2 + 1;
                    for (i = 0; i < 10;  i = i + 1) if (dut.u_motor.amem[976 + i] !== a3x[i]) eA3 = eA3 + 1;
                    chk_ventana(2'd3, 2'd1, 128, e1);  chk_ventana(2'd3, 2'd2, 64, e2);  chk_ventana(2'd3, 2'd3, 10, e3);
                    $display("NIVEL 1 (imagen 0, antes de UPDATE): errores a1=%0d a2=%0d a3=%0d | dz1=%0d dz2=%0d dz3=%0d (dz leido por la ventana)",
                             eA1, eA2, eA3, e1, e2, e3);
                    nerr = nerr + eA1 + eA2 + eA3 + e1 + e2 + e3;
                end
                cmd(7'd5, 3'd3, 0); cmd(7'd5, 3'd2, 0); cmd(7'd5, 3'd1, 0);     // UPDATE 3,2,1
                if (CHECK0 && ep == 0 && im == 0) begin
                    chk_ventana(2'd0, 2'd1, 128*784, e1);  chk_ventana(2'd0, 2'd2, 64*128, e2);  chk_ventana(2'd0, 2'd3, 10*64, e3);
                    eb = 0; el = 0;
                    for (i = 1; i <= 3; i = i + 1) begin : chk_bl
                        integer nrr, ee;
                        nrr = (i == 1) ? 128 : (i == 2) ? 64 : 10;
                        chk_ventana(2'd1, i[1:0], nrr, ee);  eb = eb + ee;
                        chk_ventana(2'd2, i[1:0], nrr, ee);  el = el + ee;
                    end
                    $display("NIVEL 1 (imagen 0, tras UPDATE, leido por la ventana): errores W1=%0d W2=%0d W3=%0d b=%0d LFSR=%0d",
                             e1, e2, e3, eb, el);
                    nerr = nerr + e1 + e2 + e3 + eb + el;
                end
            end
            bus_rd(A_E2LO, d);  hh = {32'b0, d};
            bus_rd(A_E2HI, d);  hh = hh | ({32'b0, d} << 32);
            bus_rd(A_HITS, d);
            ck((hh === epx[2*ep]) && ({32'b0, d} === epx[2*ep+1]), "totales de la epoca (por registros EP_*)");
            $display("epoca %0d: e2=%0d (esperado %0d)  aciertos=%0d (esperado %0d)  %s", ep, hh, epx[2*ep], d, epx[2*ep+1],
                     ((hh === epx[2*ep]) && ({32'b0, d} === epx[2*ep+1])) ? "ok" : "DIFIERE");
        end

        // ---------------- hash final leyendo los pesos por la ventana ----------------
        hh = 64'd1469598103934665603;
        for (i = 1; i <= 3; i = i + 1) begin : hash_capa
            integer nrr, ncc, k3;
            nrr = (i == 1) ? 128 : (i == 2) ? 64 : 10;
            ncc = (i == 1) ? 784 : (i == 2) ? 128 : 64;
            bus_wr(A_RDADDR, {2'd0, i[1:0], 8'd0, 10'd0});
            for (k3 = 0; k3 < nrr*ncc; k3 = k3 + 1) begin bus_rd(A_RDDATA, d); hh = fnv32(hh, d); end
            bus_wr(A_RDADDR, {2'd1, i[1:0], 8'd0, 10'd0});
            for (k3 = 0; k3 < nrr; k3 = k3 + 1) begin bus_rd(A_RDDATA, d); hh = fnv32(hh, d); end
        end
        $display("hash enteros RTL (por el bus) = %h   emulador = %h   %s", hh, hmem[0], (hh === hmem[0]) ? "IGUAL" : "DIFIERE");
        if (hh !== hmem[0]) nerr = nerr + 1;
        begin : ciclos
            reg [31:0] c1, c2, c3, c4, c5, cb;
            bus_rd(A_CYC_LDX, c1); bus_rd(A_CYC_FWD, c2); bus_rd(A_CYC_DEL, c3); bus_rd(A_CYC_BWD, c4); bus_rd(A_CYC_UPD, c5);
            bus_rd(A_CYC_BUSY, cb);
            $display("ciclos(simulados) totales de la ultima epoca: LOADX=%0d MATVEC=%0d DELTA=%0d MATVEC_T=%0d UPDATE=%0d CYC_BUSY=%0d  (por imagen: %0d)",
                     c1, c2, c3, c4, c5, cb, (c1 + c2 + c3 + c4 + c5) / N);
        end
        $display("BUS: %0d comprobaciones dirigidas", ntest);
        $display("L=%0d N=%0d EPOCHS=%0d rmode=%0d : errores=%0d", L, N, EPOCHS, RMODE, nerr);
        if (nerr == 0) $display("PASS: el motor, visto desde el bus Avalon, coincide bit a bit con el emulador");
        else           $display("FAIL");
        $finish;
    end
endmodule
