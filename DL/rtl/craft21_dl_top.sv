// Craft21 y el acelerador comparten el reloj y el reset activo en alto.
module craft21_dl_top #(
    parameter L = 32,
    parameter RAM_ADDR_WIDTH = 16,
    parameter INIT_LANES = 0,       // Precarga W, b y LFSR desde archivos por lane.
    parameter FW = 20,
    parameter LUT_FILE = "lut.hex",
    parameter NIMG = 100,
    parameter IMG_FILE = "ds_img.hex",
    parameter LBL_FILE = "ds_lbl.hex"
)(
    input logic clk,
    input logic reset,
    output logic command_accepted, local_store,
    output logic [31:0] local_addr, local_wdata
);
    logic [4:0] address;
    logic read, write, waitrequest;
    logic [31:0] writedata, readdata;

    // La placa usa el primer CMD aceptado para indicar ejecución.
    assign command_accepted = !reset && write && !waitrequest && address == 5'd1;

    top #(.ENABLE_MMIO(1), .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH)) cpu (
        .clk(clk), .reset(reset),
        .av_address(address), .av_read(read), .av_write(write),
        .av_writedata(writedata), .av_readdata(readdata),
        .av_waitrequest(waitrequest),
        .local_store(local_store), .local_addr(local_addr), .local_wdata(local_wdata)
    );

    dl_periph #(
        .L(L), .INIT_LANES(INIT_LANES), .FW(FW), .LUT_FILE(LUT_FILE), .NIMG(NIMG),
        .IMG_FILE(IMG_FILE), .LBL_FILE(LBL_FILE)
    ) accelerator (
        .clk(clk), .rst(reset),
        .avs_address(address), .avs_read(read), .avs_write(write),
        .avs_writedata(writedata), .avs_readdata(readdata),
        .avs_waitrequest(waitrequest)
    );
endmodule
