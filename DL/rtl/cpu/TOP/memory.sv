module memory #(
    parameter ENABLE_MMIO = 0,
    parameter RAM_ADDR_WIDTH = 16
)(
    input logic clk, reset,
    input logic we_mem, we_reg,
    input logic [1:0] result_src,
    input logic [31:0] alu_result, rd2, pc_plus4,
    input logic [4:0] instrD,

    output logic [31:0] rMemData, alu_resultOUT, pc_plus4OUT,
    output logic [4:0] instrDOUT, instrMOUT_hz,
    output logic we_reg_MEM_hz,
    output logic [31:0] ex_mem,
    output logic stall_mem,

    output logic [4:0] av_address,
    output logic av_read, av_write,
    output logic [31:0] av_writedata,
    input logic [31:0] av_readdata,
    input logic av_waitrequest,
    output logic local_store
);
    localparam logic [31:0] MMIO_BASE = 32'h0001_0000;
    logic mmio_selected, mmio_word, mapped_word, aligned_word, ram_word;
    logic [4:0] word_index;
    logic [31:0] ram_data, mmio_data;
    logic ram_request, ram_response, avalon_stall;

    // Selecciona la ventana con la dirección completa, antes de indexar.
    assign mmio_selected = ENABLE_MMIO && (alu_result[31:7] == MMIO_BASE[31:7]);
    assign word_index = alu_result[6:2];
    // dl_avalon define registros en 00–0D y 10–13.
    assign mapped_word = (word_index <= 5'h0d) ||
                         (word_index >= 5'h10 && word_index <= 5'h13);
    assign aligned_word = alu_result[1:0] == 2'b00;
    assign mmio_word = mmio_selected && mapped_word && aligned_word;
    assign ram_word = !mmio_selected && aligned_word;

    // La carga ocupa MEM hasta que los bancos registran su salida.
    assign ram_request = !reset && ram_word && result_src == 2'b01 && !ram_response;
    assign local_store = !reset && ram_word && we_mem;
    assign stall_mem = ram_request || avalon_stall;

    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            ram_response <= 1'b0;
        else
            ram_response <= ram_request;
    end

    data_ram #(.ADDR_WIDTH(RAM_ADDR_WIDTH)) Ram(
        .clk(clk),
        .re_mem(ram_request), .we_mem(local_store),
        .addr(alu_result),
        .wdata(rd2),
        .rdata(ram_data)
    );

    avalon_master Master(
        .clk(clk), .reset(reset),
        .load(mmio_word && result_src == 2'b01),
        .store(mmio_word && we_mem),
        .address(word_index), .store_data(rd2),
        .stall(avalon_stall),
        .av_address(av_address), .av_read(av_read), .av_write(av_write),
        .av_writedata(av_writedata), .av_readdata(av_readdata),
        .av_waitrequest(av_waitrequest), .load_data(mmio_data)
    );

    // Los accesos inválidos devuelven cero y carecen de efectos.
    assign rMemData = mmio_word ? mmio_data : ram_word ? ram_data : 32'b0;
    assign alu_resultOUT = alu_result;
    assign pc_plus4OUT = pc_plus4;
    assign instrDOUT = instrD;
    assign instrMOUT_hz = instrD;

    // Reenvía el resultado arquitectónico cuando MEM puede avanzar.
    assign we_reg_MEM_hz = we_reg && !stall_mem;
    assign ex_mem = (result_src == 2'b01) ? rMemData :
                    (result_src == 2'b10) ? pc_plus4 : alu_result;
endmodule
