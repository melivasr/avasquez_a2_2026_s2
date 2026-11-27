module id_ex_pipe (
    input  logic clk,
    input  logic reset,

    input  logic jump, blt, bge, bne, beq, 
    input logic [1:0] result_src,
    input logic alu_src,
    input logic we_mem, size,
    input logic [4:0] alu_control,
    input logic we_reg,
    input logic [31:0] rd1, rd2, imm, pc_act, pcplus4,
    input logic [4:0] instrD, rs1DIN, rs2DIN,

    // Hazard Unit
    input logic flushE,
    input logic stallE,

    output  logic jumpOUT, bltOUT, bgeOUT, bneOUT, beqOUT, 
    output logic [1:0] result_srcOUT,
    output logic alu_srcOUT,
    output logic we_memOUT, sizeOUT,
    output logic [4:0] alu_controlOUT,
    output logic we_regOUT,
    output logic [31:0] rd1OUT, rd2OUT, immOUT, pc_actOUT, pcplus4OUT,
    output logic [4:0] instrDOUT, rs1DOUT, rs2DOUT
);

task automatic clear_pipeline;
    jumpOUT              <= 1'b0;
    bltOUT               <= 1'b0;
    bgeOUT               <= 1'b0;
    bneOUT               <= 1'b0;
    beqOUT               <= 1'b0;
    result_srcOUT        <= 2'b0;
    alu_srcOUT           <= 1'b0;
    we_memOUT            <= 1'b0;
    sizeOUT              <= 1'b0;
    alu_controlOUT       <= 5'b0;
    we_regOUT            <= 1'b0;
    rd1OUT               <= 32'b0;
    rd2OUT               <= 32'b0;
    immOUT               <= 32'b0;
    pc_actOUT            <= 32'b0;
    pcplus4OUT           <= 32'b0;
    instrDOUT            <= 5'b0;
    rs1DOUT              <= 5'b0;
    rs2DOUT              <= 5'b0;
endtask

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        clear_pipeline();
    end else if (flushE) begin
        clear_pipeline();
    end else if (stallE) begin
        // hold — mantener valores actuales
    end else begin
        jumpOUT              <= jump;
        bltOUT               <= blt;
        bgeOUT               <= bge;
        bneOUT               <= bne;
        beqOUT               <= beq;
        result_srcOUT        <= result_src;
        alu_srcOUT           <= alu_src;
        we_memOUT            <= we_mem;
        sizeOUT              <= size;
        alu_controlOUT       <= alu_control;
        we_regOUT            <= we_reg;
        rd1OUT               <= rd1;
        rd2OUT               <= rd2;
        immOUT               <= imm;
        pc_actOUT            <= pc_act;
        pcplus4OUT           <= pcplus4;
        instrDOUT            <= instrD;
        rs1DOUT              <= rs1DIN;
        rs2DOUT              <= rs2DIN;
    end
end

endmodule
