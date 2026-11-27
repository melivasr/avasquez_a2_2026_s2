`timescale 1ns/1ps

module tb_instr_rom;

// =====================================================
//  DUT ports
// =====================================================
logic        clk   = 0;
logic [31:0] addr;
logic [31:0] instr;

// =====================================================
//  Clock  (10 ns period)
// =====================================================
always #5 clk = ~clk;

// =====================================================
//  DUT
// =====================================================
instr_rom #(.DEPTH(16)) dut (
    .clk   (clk),
    .addr  (addr),
    .instr (instr)
);

// =====================================================
//  Helpers
// =====================================================
task automatic tick;
    @(negedge clk); #1;
endtask

task automatic check;
    input [31:0] got, expected;
    input [63:0] label; // just a tag value, printed in hex
    if (got === expected)
        $display("[PASS] addr=%08h  instr=%08h", label[31:0], got);
    else
        $fatal(1, "[FAIL] addr=%08h  got=%08h  expected=%08h",
                 label[31:0], got, expected);
endtask

// =====================================================
//  Test sequence
// =====================================================
initial begin
    $dumpfile("sim/waves/tb_instr_rom.vcd");
    $dumpvars(0, tb_instr_rom);

    #1;
    for (int i = 0; i < 8; i++)
        dut.memory[i] = 32'h12340000 + i;

    $display("============================================");
    $display("  tb_instr_rom — ROM síncrona");
    $display("============================================");

    // --------------------------------------------------
    // TEST 1: lectura al flanco negativo
    // --------------------------------------------------
    $display("\n--- TEST 1: lectura al flanco negativo ---");

    addr = 32'h0000_0000;   // word 0 → memory[0]
    tick;                   // flanco: ROM registra addr=0
    // instr ya tiene la salida de memory[0]
    check(instr, dut.memory[0], 64'h0);

    addr = 32'h0000_0004;   // word 1 → memory[1]
    tick;
    check(instr, dut.memory[1], 64'h4);

    addr = 32'h0000_0008;   // word 2 → memory[2]
    tick;
    check(instr, dut.memory[2], 64'h8);

    // --------------------------------------------------
    // TEST 2: dirección fuera de rango → NOP
    // --------------------------------------------------
    $display("\n--- TEST 2: dirección fuera de rango ---");

    addr = 32'hFFFF_FFFC;   // muy por arriba de DEPTH=16
    tick;
    check(instr, 32'h0058_0000, 64'hFFFFFFFC); // NOP en Craft21

    // --------------------------------------------------
    // TEST 3: acceso secuencial completo (words 0..7)
    // --------------------------------------------------
    $display("\n--- TEST 3: barrido secuencial words 0-7 ---");

    begin : sweep
        integer i;
        for (i = 0; i < 8; i = i + 1) begin
            addr = i * 4;
            tick;
            check(instr, dut.memory[i], i * 4);
        end
    end

    $display("\n============================================");
    $display("  Fin de tb_instr_rom");
    $display("============================================");
    $finish;
end

endmodule