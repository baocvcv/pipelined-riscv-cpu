`include "../../common.vh"
`include "id.vh"

module id_ex (
    input wire clk,
    input wire rst,
    input wire stall,
    input wire bubble,

    input wire[`PrivLvlBus] id_priv_lvl,
    input wire[`DataBus] id_pc,
    input wire[`AluOpBus] id_alu_op,
    input wire[`DataBus] id_alu_a,
    input wire[`DataBus] id_alu_b,

    input wire[`DataBus] id_data_rs2,
    input wire[`RegAddrBus] id_rd,
    input wire id_reg_write,
    input wire id_reg_write_sel,

    input wire id_mem_read,
    input wire id_mem_write,
    input wire[`Func3Bus] id_mem_option,

    input wire id_csr_we,
    input wire[`CSRAddrBus] id_csr_rd,
    input wire[`DataBus] id_csr_wdata,

    output reg[`PrivLvlBus] ex_priv_lvl,
    output reg[`DataBus] ex_pc,
    output reg[`AluOpBus] ex_alu_op,
    output reg[`DataBus] ex_alu_a,
    output reg[`DataBus] ex_alu_b,

    output reg[`DataBus] ex_data_rs2,
    output reg[`RegAddrBus] ex_rd,
    output reg ex_reg_write,
    output reg ex_reg_write_sel,

    output reg ex_mem_read,
    output reg ex_mem_write,
    output reg[`Func3Bus] ex_mem_option,
    
    output reg ex_csr_we,
    output reg[`CSRAddrBus] ex_csr_rd,
    output reg[`DataBus] ex_csr_wdata
);

always @ (posedge clk or posedge rst) begin
    if (rst == `RstEnable) begin
        ex_priv_lvl <= `M_MODE;
        ex_pc <= `StartInstAddr;
        ex_alu_op <= `ALU_ADD;
        ex_alu_a <= `ZeroWord;
        ex_alu_b <= `ZeroWord;

        ex_data_rs2 <= `ZeroWord;
        ex_rd <= `RegX0;
        ex_reg_write <= `WriteDisable;
        ex_reg_write_sel <= `RegWriteSrcAlu;

        ex_mem_read <= `ReadDisable;
        ex_mem_write <= `WriteDisable;
        ex_mem_option <= `FUNC3_WORD;

        ex_csr_we <= `WriteDisable;
        ex_csr_rd <= `ZeroWord;
        ex_csr_wdata <= `ZeroWord;
    end else if (~stall) begin
        if (bubble) begin
            ex_reg_write <= `WriteDisable;
            ex_mem_read <= `ReadDisable;
            ex_mem_write <= `WriteDisable;
            ex_mem_option <= `FUNC3_WORD;
            ex_csr_we <= `WriteDisable;
        end else begin
            ex_priv_lvl <= id_priv_lvl;
            ex_pc <= id_pc;
            ex_alu_op <= id_alu_op;
            ex_alu_a <= id_alu_a;
            ex_alu_b <= id_alu_b;

            ex_data_rs2 <= id_data_rs2;
            ex_rd <= id_rd;
            ex_reg_write <= id_reg_write;
            ex_reg_write_sel <= id_reg_write_sel;

            ex_mem_read <= id_mem_read;
            ex_mem_write <= id_mem_write;
            ex_mem_option <= id_mem_option;

            ex_csr_we <= id_csr_we;
            ex_csr_rd <= id_csr_rd;
            ex_csr_wdata <= id_csr_wdata;
        end
    end
end

endmodule
