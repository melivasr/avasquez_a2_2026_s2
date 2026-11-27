// Craft21 y el acelerador comparten el reloj y el reset activo en alto.
module craft21_dl_top #(
    parameter L = 32,
    parameter FW = 20,
    parameter LUT_FILE = "lut.hex",
    parameter NIMG = 100,
    parameter IMG_FILE = "ds_img.hex",
    parameter LBL_FILE = "ds_lbl.hex"
)(
    input logic clk,
    input logic reset
);
    logic [4:0] address;
    logic read, write, waitrequest;
    logic [31:0] writedata, readdata;

    top #(.ENABLE_MMIO(1)) cpu (
        .clk(clk), .reset(reset),
        .av_address(address), .av_read(read), .av_write(write),
        .av_writedata(writedata), .av_readdata(readdata),
        .av_waitrequest(waitrequest)
    );

    dl_periph #(
        .L(L), .FW(FW), .LUT_FILE(LUT_FILE), .NIMG(NIMG),
        .IMG_FILE(IMG_FILE), .LBL_FILE(LBL_FILE)
    ) accelerator (
        .clk(clk), .rst(reset),
        .avs_address(address), .avs_read(read), .avs_write(write),
        .avs_writedata(writedata), .avs_readdata(readdata),
        .avs_waitrequest(waitrequest)
    );
endmodule
