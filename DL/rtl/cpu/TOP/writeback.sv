module writeback(
    input logic [1:0] result_src,
    input logic we_reg,

    input logic [31:0] rMemData, alu_result, pc_plus4,

    input logic pc_srcEx,
    input logic [31:0] pc_plus4F, pc_target_EX,
    input logic [4:0] instrD,

    output logic we_regOUT,
    output logic [31:0] wdOUT,

    output logic [31:0] new_addr,
    output logic [4:0] instrDOUT, instrWbOUT_hz,

    output logic we_reg_wb_hz
);

// ==========================================================
//                       RESULT_SRC [1:0]
// ==========================================================

always_comb begin
    case (result_src)
        2'b01: wdOUT = rMemData;
        2'b10: wdOUT = pc_plus4;
        default: wdOUT = alu_result;
    endcase
end

// ==========================================================
//                       NEW PC MUX
// ==========================================================

mux31 new_pc_mux(
    .in1(pc_plus4F),
    .in2(pc_target_EX),
    .src(pc_srcEx),
    .out(new_addr)
);

// ==========================================================
//                       OTRAS SEÑALES
// ==========================================================

assign instrDOUT = instrD;
assign we_regOUT = we_reg;

// Hazard Unit
assign instrWbOUT_hz = instrD;
assign we_reg_wb_hz = we_reg;

// mem_wb = wdOUT

endmodule
