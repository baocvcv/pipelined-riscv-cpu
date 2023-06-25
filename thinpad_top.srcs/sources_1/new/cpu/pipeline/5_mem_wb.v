`include "../../common.vh"

module mem_wb (
    input wire clk,
    input wire rst,
    input wire stall,
    input wire bubble,

    input wire[`DataBus] mem_alu_out,
    input wire[`DataBus] mem_data_out,
    input wire[`RegAddrBus] mem_rd,
    input wire mem_reg_write,
    input wire mem_reg_write_sel,

    output reg[`DataBus] wb_alu_out,
    output reg[`DataBus] wb_data_out,
    output reg[`RegAddrBus] wb_rd,
    output reg wb_reg_write,
    output reg wb_reg_write_sel
);

always @ (posedge clk or posedge rst) begin
    if (rst == `RstEnable) begin
        wb_alu_out <= `ZeroWord;
        wb_data_out <= `ZeroWord;
        wb_rd <= `RegX0;
        wb_reg_write <= `WriteDisable;
        wb_reg_write_sel <= `RegWriteSrcAlu;
    end else if (~stall) begin
        if (bubble) begin
            wb_reg_write <= `WriteDisable;
        end else begin
            wb_alu_out <= mem_alu_out;
            wb_data_out <= mem_data_out;
            wb_rd <= mem_rd;
            wb_reg_write <= mem_reg_write;
            wb_reg_write_sel <= mem_reg_write_sel;
        end
    end
end
endmodule
