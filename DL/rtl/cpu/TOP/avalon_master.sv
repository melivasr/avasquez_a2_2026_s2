// Se admite un acceso pendiente. EX/MEM mantiene la solicitud mientras stall esté activo.
module avalon_master (
    input logic clk, reset,
    input logic load, store,
    input logic [4:0] address,
    input logic [31:0] store_data,
    output logic stall,
    output logic [4:0]  av_address,
    output logic av_read, av_write,
    output logic [31:0] av_writedata,
    input logic [31:0] av_readdata,
    input logic  av_waitrequest,
    output logic [31:0] load_data
);
    logic read_response;

    // El esclavo actualiza readdata después de aceptar la lectura. MEM/WB captura
    // el dato al final del ciclo de respuesta. Durante ese ciclo no se emite ninguna solicitud.
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            read_response <= 1'b0;
        else
            read_response <= av_read && !av_waitrequest;
    end

    assign av_address = address;
    assign av_writedata = store_data;
    assign av_read = load && !read_response && !reset;
    assign av_write = store && !read_response && !reset;
    assign stall = av_read || (av_write && av_waitrequest);
    assign load_data = av_readdata;
endmodule
