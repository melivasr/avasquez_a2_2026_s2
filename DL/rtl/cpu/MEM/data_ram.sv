module data_ram #(
    parameter ADDR_WIDTH = 16  // Ancho de la dirección local en bytes.
) (
    input  logic        clk,
    input  logic        re_mem, we_mem,
    input  logic [31:0] addr,
    input  logic [31:0] wdata,
    output logic [31:0] rdata
);
    localparam WORDS = 1 << (ADDR_WIDTH - 2);
    wire [ADDR_WIDTH-3:0] index = addr[ADDR_WIDTH-1:2];
    (* ramstyle = "M10K" *) logic [31:0] mem [0:WORDS-1];

    initial $readmemh("programs/data_words.hex", mem);

    // MEM valida el tamaño y el alineamiento antes de habilitar el puerto.
    always @(posedge clk) begin
        if (we_mem)
            mem[index] <= wdata;
        else if (re_mem)
            rdata <= mem[index];
    end
endmodule
