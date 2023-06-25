`include "../../common.vh"

module reg_file (
    input wire clk,
    input wire rst,

    input wire we_n,
    input wire[`RegAddrBus] rd,
    input wire[`DataBus] wdata,

    // input wire re1,
    input wire[`RegAddrBus] rs1,
    output wire[`DataBus] rdata1,

    // input wire re2,
    input wire[`RegAddrBus] rs2,
    output wire[`DataBus] rdata2
);

reg[`DataBus] regs[1:(`RegNum-1)];

integer i;
always @ (posedge rst or posedge clk) begin
    if (rst == `RstEnable) begin
        for (i = 1; i < `RegNum; i = i + 1) begin
            regs[i] <= `ZeroWord;
        end
    end else begin
        if (we_n == `WriteEnable && rd != `RegX0)
            regs[rd] <= wdata;
    end
end

assign rdata1 = (rst == `RstEnable || rs1 == `RegX0) ? `ZeroWord :
                (we_n == `WriteEnable && rs1 == rd) ? wdata : regs[rs1];

assign rdata2 = (rst == `RstEnable || rs2 == `RegX0) ? `ZeroWord :
                (we_n == `WriteEnable && rs2 == rd) ? wdata : regs[rs2];

endmodule
