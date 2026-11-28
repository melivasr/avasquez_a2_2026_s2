`timescale 1ns/1ps
module tb_data_ram #(parameter RAM_ADDR_WIDTH = 16);
    localparam CAPACITY = 1 << RAM_ADDR_WIDTH;
    localparam [31:0] ULTIMA_PALABRA = CAPACITY - 4;
    localparam [31:0] ALIAS_RAM = 32'h0002_0000;
    localparam PALABRA = 1'b1, BYTE = 1'b0;
    logic clk = 0, reset = 1;
    always #5 clk = ~clk;

    logic store = 0, size = 1;
    logic [1:0] source = 0;
    logic [31:0] addr = 0, wdata = 0, rdata;
    logic stall, local_store, av_read, av_write;

    memory #(.ENABLE_MMIO(1), .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH)) dut (
        .clk(clk), .reset(reset), .we_mem(store), .size(size), .we_reg(1'b0),
        .result_src(source), .alu_result(addr), .rd2(wdata), .pc_plus4(32'b0), .instrD(5'b0),
        .rMemData(rdata), .stall_mem(stall), .local_store(local_store),
        .av_read(av_read), .av_write(av_write), .av_readdata(32'b0), .av_waitrequest(1'b0)
    );

    task automatic probar_escritura(input logic [31:0] direccion, dato,
                                input logic es_palabra, aceptado);
        @(negedge clk);
        addr = direccion;
        wdata = dato;
        size = es_palabra;
        store = 1;
        #1;
        if (local_store !== aceptado || stall !== 1'b0 ||
            av_read !== 1'b0 || av_write !== 1'b0)
            $fatal(1, "Controles de SW/SB incorrectos en %h", addr);
        @(negedge clk);
        store = 0;
    endtask

    task automatic probar_lectura(input logic [31:0] direccion, esperado,
                               input logic es_palabra, aceptado);
        @(negedge clk);
        addr = direccion;
        size = es_palabra;
        source = 2'b01;
        #1;
        if (dut.ram_request !== aceptado || stall !== aceptado ||
            av_read !== 1'b0 || av_write !== 1'b0)
            $fatal(1, "Controles de LW/LB incorrectos en %h", addr);
        // La respuesta a un acceso inválido es inmediata.
        if (!aceptado && rdata !== esperado)
            $fatal(1, "Acceso rechazado en %h devolvió %h", addr, rdata);
        @(negedge clk);
        if (stall !== 1'b0 || rdata !== esperado)
            $fatal(1, "Respuesta en %h: %h, esperado %h", addr, rdata, esperado);
        source = 0;
    endtask

    initial begin
        @(negedge clk);
        reset = 0;
        // La imagen de arranque carga la primera y la última palabra. Los huecos quedan en cero.
        probar_lectura(0, 32'h44332211, PALABRA, 1'b1);
        probar_lectura(ULTIMA_PALABRA, 32'hddccbbaa, PALABRA, 1'b1);
        probar_lectura(4, 0, PALABRA, 1'b1);

        // Las direcciones altas fuera de MMIO apuntan a las mismas palabras locales.
        probar_escritura(0, 32'h12345678, PALABRA, 1'b1);
        probar_escritura(ULTIMA_PALABRA, 32'h87654321, PALABRA, 1'b1);
        probar_lectura(ALIAS_RAM, 32'h12345678, PALABRA, 1'b1);
        probar_lectura(ALIAS_RAM + ULTIMA_PALABRA, 32'h87654321, PALABRA, 1'b1);

        // Prueba accesos por byte y los tres offsets que dejan una palabra desalineada.
        probar_escritura(0, 32'h80, BYTE, 1'b0);
        probar_lectura(0, 0, BYTE, 1'b0);
        for (integer offset = 1; offset <= 3; offset++) begin
            probar_escritura(offset, 32'hffffffff, PALABRA, 1'b0);
            probar_lectura(offset, 0, PALABRA, 1'b0);
        end
        probar_escritura(CAPACITY - 1, 32'hffffffff, PALABRA, 1'b0);
        probar_lectura(CAPACITY - 1, 0, PALABRA, 1'b0);

        // Comprueba que los stores rechazados no cambiaron la RAM.
        probar_lectura(0, 32'h12345678, PALABRA, 1'b1);
        probar_lectura(4, 0, PALABRA, 1'b1);
        probar_lectura(ULTIMA_PALABRA, 32'h87654321, PALABRA, 1'b1);

        // Durante reset se bloquea la escritura y se conserva el dato anterior.
        reset = 1;
        probar_escritura(0, 32'hffffffff, PALABRA, 1'b0);
        reset = 0;
        probar_lectura(0, 32'h12345678, PALABRA, 1'b1);
        $display("PASS RAM por palabras: %0d bytes", CAPACITY);
        $finish;
    end
endmodule
