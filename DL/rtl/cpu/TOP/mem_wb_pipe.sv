module mem_wb_pipe(
    input  logic clk,
    input  logic reset,

    input logic [1:0] result_src,
    input logic we_reg,
    input logic [31:0] rMemData, alu_result, pcPlus4,
    input logic [4:0] instrD,

    // Hazard Unit
    input logic stallW,

    output logic [1:0] result_srcOUT,
    output logic we_regOUT,
    output logic [31:0] rMemDataOUT, alu_resultOUT, pcPlus4OUT,
    output logic [4:0] instrDOUT
);

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        result_srcOUT       <= 2'b0;
        we_regOUT           <= 1'b0;
        rMemDataOUT         <= 32'b0;
        alu_resultOUT       <= 32'b0;
        pcPlus4OUT          <= 32'b0;
        instrDOUT           <= 5'b0;
    end else if (stallW) begin
        // hold — mantener valores actuales
    end else begin
        result_srcOUT       <= result_src;
        we_regOUT           <= we_reg;
        rMemDataOUT         <= rMemData;
        alu_resultOUT        <= alu_result;
        pcPlus4OUT          <= pcPlus4;
        instrDOUT           <= instrD;
    end
end

endmodule
