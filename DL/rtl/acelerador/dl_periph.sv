// ============================================================
// dl_periph.sv - periferico completo para Platform Designer: dl_avalon + rom_dataset.
// Unico acceso desde fuera: clk, rst y el esclavo Avalon-MM (mapa de registros en dl_avalon.sv).
// rst es SINCRONO y ACTIVO EN ALTO (declararlo asi en el editor de componentes).
// SIMULACION: nada de esto esta sintetizado [VERIFICAR].
// ============================================================
module dl_periph #(
    parameter L        = 32,           // sintetizar solo con L potencia de 2 (32 o 64)
    parameter INIT_LANES = 0,       // Precarga W, b y LFSR desde archivos por lane.
    parameter FW       = 20,
    parameter LUT_FILE = "lut.hex",
    parameter NIMG     = 100,
    parameter IMG_FILE = "ds_img.hex",
    parameter LBL_FILE = "ds_lbl.hex"
) (
    input  logic        clk,
    input  logic        rst,
    input  logic [4:0]  avs_address,
    input  logic        avs_read,
    input  logic        avs_write,
    input  logic [31:0] avs_writedata,
    output logic [31:0] avs_readdata,
    output logic        avs_waitrequest
);
    wire [25:0] px_addr;  wire [7:0] px_data;
    wire [15:0] lb_addr;  wire [7:0] lb_data;

    dl_avalon #(.L(L), .INIT_LANES(INIT_LANES), .FW(FW), .LUT_FILE(LUT_FILE)) u_avs (
        .clk(clk), .rst(rst),
        .avs_address(avs_address), .avs_read(avs_read), .avs_write(avs_write),
        .avs_writedata(avs_writedata), .avs_readdata(avs_readdata), .avs_waitrequest(avs_waitrequest),
        .px_addr(px_addr), .px_data(px_data), .lb_addr(lb_addr), .lb_data(lb_data)
    );

    rom_dataset #(.NIMG(NIMG), .IMG_FILE(IMG_FILE), .LBL_FILE(LBL_FILE)) u_rom (
        .clk(clk), .px_addr(px_addr), .px_data(px_data), .lb_addr(lb_addr), .lb_data(lb_data)
    );
endmodule
