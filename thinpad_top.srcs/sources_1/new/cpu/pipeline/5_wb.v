`include "../../common.vh"

module wb (
    input wire rst,

    input wire[`DataBus] mem_data_out,
    input wire[`DataBus] alu_out,
    input wire reg_write_sel,

    output wire[`DataBus] wdata,

    output wire exception_req,
    output wire[`DataBus] cause
);

assign wdata = (rst == `RstEnable) ? `ZeroWord :
               (reg_write_sel == `RegWriteSrcAlu) ? alu_out :
               (reg_write_sel == `RegWriteSrcMem) ? mem_data_out :
               `ZeroWord;

assign exception_req = `False_v;
assign cause = `ZeroWord;

endmodule
