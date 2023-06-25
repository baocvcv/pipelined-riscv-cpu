`include "../../common.vh"
`include "id.vh"

module ex_mem (
    input wire clk,
    input wire rst,
    input wire stall,
    input wire bubble,

    input wire[`PrivLvlBus] ex_priv_lvl,
    input wire[`DataBus] ex_pc,
    input wire[`DataBus] ex_alu_out,
    input wire[`DataBus] ex_data_rs2,
    input wire[`RegAddrBus] ex_rd,
    input wire ex_reg_write,
    input wire ex_reg_write_sel,

    input wire ex_mem_read,
    input wire ex_mem_write,
    input wire[`Func3Bus] ex_mem_option,

    input wire ex_csr_we,
    input wire[`CSRAddrBus] ex_csr_rd,
    input wire[`DataBus] ex_csr_wdata,

    output reg[`PrivLvlBus] mem_priv_lvl,
    output reg[`DataBus] mem_pc,
    output reg[`DataBus] mem_alu_out,
    output reg[`DataBus] mem_data_rs2,
    output reg[`RegAddrBus] mem_rd,
    output reg mem_reg_write,
    output reg mem_reg_write_sel,

    output reg mem_mem_read,
    output reg mem_mem_write,
    output reg[`Func3Bus] mem_mem_option,
    
    output reg mem_csr_we,
    output reg[`CSRAddrBus] mem_csr_rd,
    output reg[`DataBus] mem_csr_wdata
);

always @ (posedge clk or posedge rst) begin
    if (rst == `RstEnable) begin
        mem_priv_lvl <= `M_MODE;
        mem_pc <= `ZeroWord;
        mem_alu_out <= `ZeroWord;
        mem_data_rs2 <= `ZeroWord;
        mem_rd <= `RegX0;
        mem_reg_write <= `WriteDisable;
        mem_reg_write_sel <= `RegWriteSrcAlu;

        mem_mem_read <= `ReadDisable;
        mem_mem_write <= `WriteDisable;
        mem_mem_option <= `FUNC3_WORD;

        mem_csr_we <= `WriteDisable;
        mem_csr_rd <= `ZeroWord;
        mem_csr_wdata <= `ZeroWord;
    end else if (~stall) begin
        if (bubble) begin
            mem_reg_write <= `WriteDisable;
            mem_mem_read <= `ReadDisable;
            mem_mem_write <= `WriteDisable;
            mem_mem_option <= `FUNC3_WORD;
            mem_csr_we <= `WriteDisable;
        end else begin
            mem_priv_lvl <= ex_priv_lvl;
            mem_pc <= ex_pc;
            mem_alu_out <= ex_alu_out;
            mem_data_rs2 <= ex_data_rs2;
            mem_rd <= ex_rd;
            mem_reg_write <= ex_reg_write;
            mem_reg_write_sel <= ex_reg_write_sel;

            mem_mem_read <= ex_mem_read;
            mem_mem_write <= ex_mem_write;
            mem_mem_option <= ex_mem_option;

            mem_csr_we <= ex_csr_we;
            mem_csr_rd <= ex_csr_rd;
            mem_csr_wdata <= ex_csr_wdata;
        end
    end
end

endmodule
