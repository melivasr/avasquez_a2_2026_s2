module memory(
    input logic clk,
    input logic reset,

    input logic we_mem, size,
    input logic we_reg,

    input logic [31:0] alu_result, rd2, pc_plus4,
    input logic [4:0] instrD,

    output logic [31:0] rMemData, alu_resultOUT, pc_plus4OUT,
    output logic [4:0] instrDOUT, instrMOUT_hz, 
    output logic we_reg_MEM_hz, // hazard unit

    // forward
    output logic [31:0] ex_mem,
    output logic stall_mem // hazard unit


);
// La RAM lee combinacionalmente durante MEM y escribe en el flanco
// que avanza EX/MEM. MEM/WB captura la lectura en ese mismo flanco.
assign stall_mem = 1'b0;

data_ram NormalRam(
    .clk(clk),
    .we_mem(we_mem && !reset),
    .size({size, 1'b0}),
    .addr(alu_result),
    .wdata(rd2),
    .rdata(rMemData)
);

// ==========================================================
//                       OTRAS SEÑALES
// ==========================================================



// NON CONTROL SIGNALS

assign alu_resultOUT = alu_result;
assign pc_plus4OUT = pc_plus4;
assign instrDOUT = instrD;

// Hazard Unit
assign instrMOUT_hz = instrD;
assign we_reg_MEM_hz = we_reg;
assign ex_mem = alu_result;


endmodule
