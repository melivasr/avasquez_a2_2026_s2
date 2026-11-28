module tb_craft21_encoding;
    logic [31:0] instr, imm;
    logic [4:0] rd, rs1, rs2, alu_control;
    logic we_reg, we_mem, jump, bne, alu_src;
    logic [1:0] result_src;

    decode dut (
        .clk(1'b0), .instr(instr), .pc_plus4(32'b0), .pc_act(32'b0),
        .we_regWB(1'b0), .wd(32'b0), .instrDIN(5'b0),
        .imm(imm), .instrDOUT(rd), .rs1DE(rs1), .rs2DE(rs2),
        .we_reg(we_reg), .we_mem(we_mem), .result_src(result_src),
        .jump(jump), .bne(bne), .alu_src(alu_src), .alu_control(alu_control)
    );

    integer fd, parsed, count;
    integer exp_rd, exp_rs1, exp_rs2, exp_reg, exp_mem, exp_result;
    logic [31:0] exp_imm;

    initial begin
        fd = $fopen("decode.txt", "r");
        if (!fd) $fatal(1, "decode.txt missing");
        count = 0;
        parsed = $fscanf(fd, "%h %d %d %d %d %d %d %h",
                        instr, exp_rd, exp_rs1, exp_rs2, exp_reg, exp_mem, exp_result, exp_imm);
        while (parsed == 8) begin
            #1;
            if (rd !== exp_rd[4:0] || rs1 !== exp_rs1[4:0] ||
                rs2 !== exp_rs2[4:0] || we_reg !== exp_reg[0] ||
                we_mem !== exp_mem[0] || result_src !== exp_result[1:0] || imm !== exp_imm ||
                bne !== (instr[3:0] == 6) || jump !== (instr[3:0] == 4) ||
                alu_src !== (instr[3:0] != 6 && instr[3:0] != 3) || alu_control !== {4'b0, bne})
                $fatal(1, "encoding mismatch for %h (imm=%h)", instr, imm);
            count++;
            parsed = $fscanf(fd, "%h %d %d %d %d %d %d %h",
                            instr, exp_rd, exp_rs1, exp_rs2, exp_reg, exp_mem, exp_result, exp_imm);
        end
        if (count != 8 || parsed > 0) $fatal(1, "incomplete encoding vectors: %0d", count);
        $fclose(fd);
        $display("PASS Craft21 encoding (8 vectors)");
        $finish;
    end
endmodule
