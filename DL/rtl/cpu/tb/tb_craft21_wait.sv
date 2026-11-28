`timescale 1ns/1ps
module tb_craft21_wait;
    logic clk = 0, reset = 1;
    always #5 clk = ~clk;
    logic [4:0] address;
    logic read, write, waitrequest;
    logic [31:0] writedata, readdata;
    top #(.ENABLE_MMIO(1)) dut (
        .clk(clk), .reset(reset), .av_address(address),
        .av_read(read), .av_write(write), .av_writedata(writedata),
        .av_readdata(readdata), .av_waitrequest(waitrequest)
    );

    integer wait_cycles = 0, reads = 0, writes = 0, commits_x6 = 0;
    logic held = 0;
    logic [38:0] previous_request;
    logic saw_branch_behind_mem = 0, saw_load_forward = 0;
    assign waitrequest = (read || write) && wait_cycles < 3;

    always @(posedge clk) begin
        if (reset) begin
            wait_cycles <= 0;
            reads <= 0;
            writes <= 0;
            commits_x6 <= 0;
            held <= 0;
            readdata <= 'x;
            saw_branch_behind_mem <= 0;
            saw_load_forward <= 0;
            if (read || write) $fatal(1, "request asserted during reset");
        end else begin
            if (held && {read, write, address, writedata} !== previous_request)
                $fatal(1, "request changed before acceptance");
            held <= waitrequest;
            previous_request <= {read, write, address, writedata};

            if (dut.Decode.RegBank.we_reg && dut.instrDWB == 6)
                commits_x6 <= commits_x6 + 1;
            if (dut.pc_srcEX && dut.stall_mem) begin
                saw_branch_behind_mem <= 1;
                if (dut.flushD || dut.flushE) $fatal(1, "flush while MEM holds EX");
            end
            if (dut.bneEX && (dut.forwardA !== 2'b01 || dut.Exec.srcA !== 32'd33))
                $fatal(1, "held branch lost the WB forwarding operand");
            if (dut.result_srcMEM == 2'b01) begin
                if (dut.stall_mem && dut.we_reg_mem)
                    $fatal(1, "load forwarded before response");
                if (!dut.stall_mem && dut.we_reg_mem) begin
                    saw_load_forward <= 1;
                    if (dut.ex_mem_hz !== 32'h444c0100)
                        $fatal(1, "incorrect architectural load forwarding");
                end
            end

            if (waitrequest)
                wait_cycles <= wait_cycles + 1;
            else
                wait_cycles <= 0;
            if (read && !waitrequest) begin
                if (address !== 0 || reads != 0) $fatal(1, "unexpected read");
                reads <= reads + 1;
                readdata <= 32'h444c0100;
            end else begin
                readdata <= 'x;
            end
            if (write && !waitrequest) begin
                if (address !== 5'h02 || writedata !== 32'h444c0100 || writes != 0)
                    $fatal(1, "unexpected store");
                writes <= writes + 1;
            end
        end
    end

    initial begin
        repeat (3) @(negedge clk);
        #1 reset = 0;
        wait (read && waitrequest);
        @(negedge clk);
        #1 reset = 1;
        #1;
        if (read || write || reads != 0) $fatal(1, "reset did not cancel stalled request");
        // El reset del CPU conserva los registros. Se pone x6 en cero para que
        // la segunda ejecución dependa del nuevo valor reenviado.
        repeat (3) @(negedge clk);
        dut.Decode.RegBank.regs[6] = 0;
        #1 reset = 0;
        repeat (100) @(negedge clk);
        if (reads != 1 || writes != 1 || commits_x6 != 1 ||
            !saw_branch_behind_mem || !saw_load_forward ||
            dut.Issue.addr_aux !== 32 ||
            dut.Memory.Ram.mem['h110 >> 2] !== 32'h0 ||
            dut.Memory.Ram.mem['h100 >> 2] !== 32'h444c0100)
            $fatal(1, "wait/branch/reset results: reads=%0d writes=%0d WB=%0d branch=%b pc=%h",
                   reads, writes, commits_x6, saw_branch_behind_mem, dut.Issue.addr_aux);
        $display("PASS Avalon wait: stable read/write, single WB, retained branch, load forwarding and reset");
        $finish;
    end

    initial begin
        #5000;
        $fatal(1, "timed out waiting for a request");
    end
endmodule
