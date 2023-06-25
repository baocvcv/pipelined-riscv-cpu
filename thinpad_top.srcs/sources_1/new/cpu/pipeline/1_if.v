`include "../../common.vh"
`include "id.vh"

module if_ (
    input wire clk,
    input wire rst,

    input wire stall,
    input wire bubble,

    input wire exc_flag,
    input wire[`DataBus] exc_target_address,

    input wire branch_flag,
    input wire[`DataBus] branch_target_address,

    input wire[`DataBus] inst_i,
    output reg[`DataBus] pc,
    output wire[`DataBus] inst_o,

    // exception from mem
    input wire exception_req_i,
    output wire exception_req_o,

    output wire update_tlb_req
);

wire[`DataBus] pc_in;
assign pc_in = (rst == `RstEnable) ? `StartInstAddr :
               exc_flag ? exc_target_address :
               (branch_flag == `BranchTaken) ? branch_target_address :
               pc + 4;

assign update_tlb_req = (rst == `RstEnable || bubble) ? `False_v :
    (inst_i[`ID_OPCODE] == `OP_SYSTEM && inst_i[`ID_FUNC3] == `FUNC3_PRIV && inst_i[`ID_FUNC7] == `FUNC7_SFENCE) ? `True_v : `False_v;

assign exception_req_o = bubble ? `False_v : exception_req_i;

assign inst_o = inst_i;

always @ (posedge clk or posedge rst) begin
    if (rst == `RstEnable) begin
        pc <= `StartInstAddr;
    end else if (~stall) begin
        pc <= pc_in;
    end
end

endmodule
