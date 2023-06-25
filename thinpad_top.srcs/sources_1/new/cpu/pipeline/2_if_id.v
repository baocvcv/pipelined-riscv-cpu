`include "../../common.vh"

module if_id (
    input wire clk,
    input wire rst,

    input wire stall,
    input wire bubble,

    input wire[`PrivLvlBus] if_priv_lvl,
    input wire[`DataBus] if_inst,
    input wire[`DataBus] if_pc,

    output reg[`PrivLvlBus] id_priv_lvl,
    output reg[`DataBus] id_inst,
    output reg[`DataBus] id_pc
);

always @ (posedge rst or posedge clk) begin
    if (rst == `RstEnable) begin
        {id_pc, id_inst} <= {`ZeroWord, `NOP};
        id_priv_lvl <= `M_MODE;
    end else if (~stall) begin
        if (bubble) begin
            id_inst <= `NOP;
            id_pc <= `StartInstAddr;
        end else begin
            {id_pc, id_inst} <= {if_pc, if_inst};
            id_priv_lvl <= if_priv_lvl;
        end
    end
end

endmodule
