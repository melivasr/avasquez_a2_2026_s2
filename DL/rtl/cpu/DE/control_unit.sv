module control_unit(
    input logic [3:0] opcode, // opcode 3:0
    input logic [3:0] func23, // func 23:20
    input logic func19, // func 19
    input logic [1:0] func15, // func 15:14

    output logic [4:0] alu_control,
    output logic we_reg, size,
    output logic imm_src2,
    output logic [1:0] imm_src, 
    output logic we_mem,
    output logic beq, bne, blt, bge, jump, 
    output logic [1:0] result_src,
    output logic alu_src
    // pc_enable = (-) stall es logica en ISSUE (decodificador)
);

// ========================================================================
//                          1 ALU_CONTROL [4:0]
// ========================================================================
assign alu_control  = (opcode == 4'b0000) ? {func23, func19} // TypeR
                    : (opcode == 4'b0001) ? {5'b00000} // TypeI
                    : (opcode == 4'b0010) ? {5'b00000} // Type SW/LW
                    : (opcode == 4'b0011) ? {5'b00000} // Type SB/LB
                    // 0100 Type J es x, no importa
                    // 0101 no usada
                    : (opcode == 4'b0110) ? {5'b00001} // Type B
                    // 0111 no usada
                    // 1000 no usada
                    // 1001 no usada
                    // 1101 no usada
                    : 5'b0; // el resto no importa

// ========================================================================
//                         2 WE_REG [0]
// ========================================================================
assign we_reg       = (opcode == 4'b0000 && func23 == 4'b0110 && func19 == 1'b0) ? 1'b0 // FREEZE
                    : (opcode == 4'b0000 && func23 == 4'b0101 && func19 == 1'b1) ? 1'b0 // SLEEP ... casos excepción primero
                    : (opcode == 4'b0000) ? 1'b1 // TypeR
                    : (opcode == 4'b0001) ? 1'b1 // TypeI
                    : (opcode == 4'b0010 && func19 == 1'b0) ? 1'b0 // Type SW
                    : (opcode == 4'b0010 && func19 == 1'b1) ? 1'b1 // Type LW
                    : (opcode == 4'b0011 && func19 == 1'b0) ? 1'b0 // Type SB
                    : (opcode == 4'b0011 && func19 == 1'b1) ? 1'b1 // Type LB
                    : (opcode == 4'b0100 && func19 == 1'b0) ? 1'b1 // Type JAL
                    : (opcode == 4'b0100 && func19 == 1'b1) ? 1'b0 // Type JALR
                    // 0101 no usada
                    : (opcode == 4'b0110) ? 1'b0 // Type B
                    // 0111 no usada
                    // 1000 no usada
                    // 1001 no usada
                    // 1101 no usada
                    : 1'b0; // el resto no importa

// ========================================================================
//                          3 SIZE [0]
// ========================================================================

assign size         = (opcode == 4'b0010) ? 1'b1 // STORE o LOAD WORD
                    : (opcode == 4'b0011) ? 1'b0 // STORE o LOAD BYTE
                    : 1'b1; // el resto no importa

// ========================================================================
//                         4 imm_src2 [2]
// ========================================================================

assign imm_src2     = (opcode == 4'b0001 && func15 == 2'b01) ? 1'b0 // addiHIGH
                    : (opcode == 4'b0001) ? 1'b1 // resto de Type I
                    : (opcode == 4'b0010 && func19 == 1'b0) ? 1'b0 // SW
                    : (opcode == 4'b0010 && func19 == 1'b1) ? 1'b1 // LW
                    : (opcode == 4'b0011 && func19 == 1'b0) ? 1'b0 // SB
                    : (opcode == 4'b0011 && func19 == 1'b1) ? 1'b1 // LB
                    : (opcode == 4'b0100 && func19 == 1'b0) ? 1'b0 // JAL
                    : (opcode == 4'b0100 && func19 == 1'b1) ? 1'b1 // JALR
                    : (opcode == 4'b0110) ? 1'b0 // Type B

                    : 1'b0; // el resto no importa

// ========================================================================
//                         5 imm_src [1:0]
// ========================================================================

assign imm_src      = (opcode == 4'b0001 && func15 == 2'b00) ? 2'b10 // addi
                    : (opcode == 4'b0001) ? 2'b11 // resto de Type I
                    : (opcode == 4'b0010 && func19 == 1'b0) ? 2'b01 // SW
                    : (opcode == 4'b0010 && func19 == 1'b1) ? 2'b00 // LW
                    : (opcode == 4'b0011 && func19 == 1'b0) ? 2'b01 // SB
                    : (opcode == 4'b0011 && func19 == 1'b1) ? 2'b00 // LB
                    : (opcode == 4'b0100 && func19 == 1'b0) ? 2'b00// JAL
                    : (opcode == 4'b0100 && func19 == 1'b1) ? 2'b01 // JALR
                    : (opcode == 4'b0110) ? 2'b10 // Type B

                    : 2'b00; // el resto no importa

// ========================================================================
//                         6 WE_MEM [0]
// ========================================================================

assign we_mem       = (opcode == 4'b0010 && func19 == 1'b0) ? 1'b1 // SW
                    : (opcode == 4'b0010 && func19 == 1'b1) ? 1'b0 // LW
                    : (opcode == 4'b0011 && func19 == 1'b0) ? 1'b1 // SB
                    : (opcode == 4'b0011 && func19 == 1'b1) ? 1'b0 // LB
                    : 1'b0; // el resto NO ESCRIBE (0)

// ========================================================================
//                        7,8,9,10  Branches [0] y 11 JUMP [0]
// ========================================================================

assign beq  = (opcode == 4'b0110 && func23 == 4'b0000 && func19 == 1'b0) ? 1'b1 // beq
            : 1'b0;

assign bne  = (opcode == 4'b0110 && func23 == 4'b0000 && func19 == 1'b1) ? 1'b1 // bne
            : 1'b0;

assign blt  = (opcode == 4'b0110 && func23 == 4'b0001 && func19 == 1'b0) ? 1'b1 // blt
            : 1'b0;

assign bge  = (opcode == 4'b0110 && func23 == 4'b0001 && func19 == 1'b1) ? 1'b1 // bge
            : 1'b0;
            
assign jump = (opcode == 4'b0100) ? 1'b1 // jal y jalr
            : 1'b0;

// ========================================================================
//                        14  RESULT_SRC [1:0]
// ========================================================================

assign result_src   = (opcode == 4'b0010 && func19 == 1'b1) ? 2'b01 // LW
                    : (opcode == 4'b0011 && func19 == 1'b1) ? 2'b01 // LB
                    : (opcode == 4'b0100 && func19 == 1'b0) ? 2'b10 // jal usa PC+4
                    : (opcode == 4'b0100 && func19 == 1'b1) ? 2'b00 // jalr no escribe registro
                    : 2'b00; // el resto es 00 o no importa

// ========================================================================
//                         15  ALU_SRC [0]
// ========================================================================

assign alu_src  = (opcode == 4'b0001) ? 1'b1 // Type Imm
                : (opcode == 4'b0010) ? 1'b1 // Type SW/LW
                : (opcode == 4'b0011) ? 1'b1 // Type SB/LB
                : (opcode == 4'b0100) ? 1'b1 // Type J usa inmediato
                : (opcode == 4'b0110) ? 1'b0 // Type BRANCH
                : 1'b0; // operando de registro
    
endmodule
