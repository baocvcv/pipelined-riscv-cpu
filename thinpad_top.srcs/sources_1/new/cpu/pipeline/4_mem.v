`include "../../common.vh"

module mem (
    input wire rst,

    input wire[`DataBus] mem_addr_i,

    output wire[`DataBus] mem_addr_o
);

assign mem_addr_o = (rst == `RstEnable) ? `ZeroWord : mem_addr_i;

endmodule
