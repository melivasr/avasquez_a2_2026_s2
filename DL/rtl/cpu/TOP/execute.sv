module execute(
    input logic jump, blt, bne, beq, bge,
    input logic [1:0] result_src,
    input logic we_mem, size,
    input logic [4:0] alu_control,
    input logic alu_src,
    input logic we_reg,
    input logic [31:0] rd1E, rd2E, immE, pc_actE, pc_plus4E,
    input logic [4:0] instrD, rs1EIN, rs2EIN,

    // Hazard Unit
    input logic [1:0] forwardA, forwardB,
    // forward
    input logic [31:0] ex_mem, mem_wb,

    output logic pc_srcEx,
    output logic [1:0] result_srcOUT, 
    output logic we_memOUT,
    output logic sizeOUT, we_regOUT,

    output logic [31:0] alu_result, rd2OUT, pc_plus4OUT,
    output logic [31:0] pc_targetOUT,
    output logic [4:0] instrDOUT, instrDOUT_hz, rs1EOUT, rs2EOUT,

    // Hazard Unit
    output logic result_src_0

);

// ==========================================================
//                       INSTANCIA PCTARG_SUM
// ==========================================================

logic [31:0] pc_target;
logic [31:0] pc_relative_target;
logic [31:0] jalr_target;
logic is_jalr;
logic [31:0] srcA;

sum31b pcTargetSum(
    .in1(immE),
    .in2(pc_actE),
    .out(pc_relative_target)
);

sum31b jalrTargetSum(
    .in1(immE),
    .in2(srcA),
    .out(jalr_target)
);

assign is_jalr = jump && (result_src == 2'b00);
assign pc_target = is_jalr ? jalr_target : pc_relative_target;

// ==========================================================
//                       INSTANCIA FORWARD A MUX
// ==========================================================

logic [31:0] not_used;
assign not_used = 31'b0;

mux31_2 fwa_mux(
    .in1(rd1E),
    .in2(mem_wb),
    .in3(ex_mem),
    .in4(not_used),
    .src(forwardA),
    .out(srcA)
);

// ==========================================================
//                       INSTANCIA FORWARD B MUX
// ==========================================================

logic [31:0] srcB_hz;

mux31_2 fwb_mux(
    .in1(rd2E),
    .in2(mem_wb),
    .in3(ex_mem),
    .in4(not_used),
    .src(forwardB),
    .out(srcB_hz)
);

// ==========================================================
//                       INSTANCIA ALU_SRC MUX31_2
// ==========================================================

logic [31:0] srcB;

mux31 alu_mux(
    .in1(srcB_hz),
    .in2(immE),
    .src(alu_src),
    .out(srcB)
);

/*
always @(*) begin
    if (!alu_src) begin
        $display("[EXEC] alu_src=%b rd1=%h srcB=%h immE=%h result=%h",
                  alu_src, rd1E, srcB, immE, alu_result);
    end
end
*/

// ==========================================================
//                       INSTANCIA ALU
// ==========================================================

logic v_flagUNUSED;
logic z_aux;
logic n_aux;

alu ALU(
    .srcA(srcA),
    .srcB(srcB),
    .alu_control(alu_control),
    .alu_result(alu_result),
    .z_flag(z_aux),
    .n_flag(n_aux),
    .v_flag(v_flagUNUSED)
);

// ==========================================================
//                       LÓGICA DE BRANCHES Y JUMP
// ==========================================================

logic notz;
logic notn;

assign notz = ~z_aux;
assign notn = ~n_aux;

// Compuertas AND

logic beqAND;
logic bneAND;

logic bgeOR;
logic bgeAND;

logic bltAND;

assign beqAND = beq & z_aux;
assign bneAND = bne & notz;

assign bgeOR = z_aux | notn;
assign bgeAND = bgeOR & bge;

assign bltAND = blt & n_aux;

// ==========================================================
//                       INSTANCIA OR5GATE
// ==========================================================

orgate5 TypeJ_OR(
    .in1(jump),
    .in2(beqAND),
    .in3(bneAND),
    .in4(bgeAND),
    .in5(bltAND),
    .out(pc_srcEx)
);

// ==========================================================
//                       OTRAS SEÑALES
// ==========================================================

assign result_srcOUT = result_src;
assign we_memOUT = we_mem;
assign sizeOUT = size;


assign we_regOUT = we_reg;

// NON CONTROL SIGNALS

assign rd2OUT = srcB_hz;

assign instrDOUT = instrD;
assign pc_plus4OUT = pc_plus4E;

assign pc_targetOUT = pc_target;

// Hazard Unit 5 salidas
// pcSrcEx en top
assign rs1EOUT = rs1EIN;
assign rs2EOUT = rs2EIN;

assign instrDOUT_hz = instrD;
assign result_src_0 = (result_src == 2'b01);

/*
always @(*) begin
    $display("[EX] srcA=%h srcB=%h alu_ctrl=%b alu_src=%b immE=%h result=%h",
        srcA, srcB, alu_control, alu_src, immE, alu_result);
end
*/

endmodule
