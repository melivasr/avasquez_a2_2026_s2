`timescale 1ns/1ps
module tb_craft21_dl;
    logic clk = 0, reset = 1;
    always #5 clk = ~clk;
    craft21_dl_top #(.NIMG(1)) dut (.clk(clk), .reset(reset));

    integer reads = 0, writes = 0;
    logic smoke, readback, saw_jal = 0;
    integer wait_cycles = 0;

    always @(posedge clk) begin
        if (!reset && readback && dut.read && dut.waitrequest)
            wait_cycles++;
    end

    always @(posedge clk) begin
        if (!reset && !dut.waitrequest) begin
            if (dut.read) begin
                if (dut.address !== (readback ? 5'h13 : (reads == 2 ? 5'h0d : 5'h00)))
                    $fatal(1, "unexpected read %0d at %h", reads, dut.address);
                reads++;
            end
            if (dut.write) begin
                if (readback) begin
                    case (writes)
                        0: if (dut.address !== 5'h10) $fatal(1, "Falta LD_ADDR");
                        1, 2, 3: if (dut.address !== 5'h11) $fatal(1, "Falta LD_DATA");
                        4: if (dut.address !== 5'h12) $fatal(1, "Falta RD_ADDR");
                        default: $fatal(1, "Store Avalon duplicado");
                    endcase
                end else if (dut.address !== 5'h02 || dut.writedata !== 32'd7)
                    $fatal(1, "Store inesperado %h = %h", dut.address, dut.writedata);
                writes++;
            end
        end
        if (!reset && dut.cpu.we_reg_mem && dut.cpu.result_srcMEM == 2'b10) begin
            saw_jal <= 1;
            if (dut.cpu.ex_mem_hz !== dut.cpu.pcPlus4MEM)
                $fatal(1, "JAL forwarded an incorrect link");
        end
    end

    task automatic check_word(input integer addr, input logic [31:0] expected);
        logic [31:0] actual;
        actual = dut.cpu.Memory.Ram.mem[addr >> 2];
        if (actual !== expected) $fatal(1, "RAM[%h]=%h expected %h", addr, actual, expected);
    endtask

    initial begin
        smoke = $test$plusargs("smoke");
        readback = $test$plusargs("readback");
        repeat (3) @(negedge clk);
        #1 reset = 0;
        repeat (100) @(negedge clk);
        if (readback) begin
            check_word('h100, 'h00001234);
            check_word('h104, 'hfffffffe);
            check_word('h108, 'h00005678);
            if (reads != 3 || writes != 5 || wait_cycles == 0)
                $fatal(1, "RD_DATA: lecturas=%0d escrituras=%0d espera=%0d", reads, writes, wait_cycles);
            $display("PASS RD_DATA: %0d lecturas, %0d escrituras, %0d flancos de espera",
                     reads, writes, wait_cycles);
            $finish;
        end
        check_word('h100, 'h444c0100);
        check_word('h008, 'h44332211);
        if (dut.cpu.Memory.Ram.mem[3] !== 32'h55443322)
            $fatal(1, "invalid store changed local RAM");
        if (smoke) begin
            if (reads != 1 || writes != 0) $fatal(1, "expected exactly one accepted ID read");
            $display("PASS ID=444c0100 in RAM[0x100], accepted reads=1");
        end else begin
            check_word('h104, 'h444c0100);
            check_word('h108, 'h00140021);
            check_word('h080, 7);
            check_word('h07c, 0);
            check_word('h10c, 80);  // JAL en PC=76 guarda PC+4 como dirección de retorno.
            if (reads != 3 || writes != 1 || !saw_jal ||
                dut.accelerator.u_avs.arg_reg !== 16'd7 ||
                dut.cpu.Decode.RegBank.regs[4] !== 32'h00140020 ||
                dut.cpu.Decode.RegBank.regs[7] !== 32'd1 ||
                dut.cpu.Decode.RegBank.regs[8] !== 32'd0 ||
                dut.cpu.Decode.RegBank.regs[9] !== 32'd0 ||
                dut.cpu.Decode.RegBank.regs[11] !== 32'd0)
                $fatal(1, "CFG, invalid accesses, x0 or transaction count");
            $display("PASS dl_periph: CFG=00140020, consecutive LW, load-use, SW isolation, invalid MMIO, x0 and FREEZE");
        end
        $finish;
    end
endmodule
