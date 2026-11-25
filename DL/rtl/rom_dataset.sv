// ============================================================
// rom_dataset.sv - ROM de imagenes (uint8) y etiquetas para LOADX.
// Interfaz IDENTICA al modelo de SDRAM de tb_dl_bus.sv: lectura SINCRONA con
// 1 ciclo de latencia (px_data / lb_data valen 1 ciclo despues de px_addr / lb_addr).
// Contenido: ds_img.hex (NIMG*784 lineas, 2 digitos hex) y ds_lbl.hex (NIMG lineas),
// los que escribe el emulador con --vectores=PREF (PREF_img.hex / PREF_lbl.hex).
// Con NIMG=100 son 78400 bytes: ~62 bloques M10K [VERIFICAR en Quartus].
// SystemVerilog (Icarus -g2012 y Quartus). Nada de esto esta sintetizado.
// ============================================================
module rom_dataset #(
    parameter NIMG     = 100,
    parameter IMG_FILE = "ds_img.hex",
    parameter LBL_FILE = "ds_lbl.hex",
    // ---- derivados: NO se sobreescriben desde fuera ----
    parameter IW = $clog2(NIMG * 784),
    parameter LW = (NIMG > 1) ? $clog2(NIMG) : 1
) (
    input  logic        clk,
    input  logic [25:0] px_addr,     // imagen*784 + pixel
    output logic [7:0]  px_data,
    input  logic [15:0] lb_addr,     // indice de imagen
    output logic [7:0]  lb_data
);
    (* ramstyle = "M10K" *) logic [7:0] img [0:NIMG*784-1];
    logic [7:0] lbl [0:NIMG-1];

    initial begin
        $readmemh(IMG_FILE, img);
        $readmemh(LBL_FILE, lbl);
    end

    always_ff @(posedge clk) begin
        px_data <= img[px_addr[IW-1:0]];
        lb_data <= lbl[lb_addr[LW-1:0]];
    end
endmodule
