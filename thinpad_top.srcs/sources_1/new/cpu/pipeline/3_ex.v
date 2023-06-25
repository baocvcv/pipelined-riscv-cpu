`include "../../common.vh"

module ex (
    input wire rst,

    input wire[`AluOpBus] alu_op,
    input wire[`DataBus] alu_a,
    input wire[`DataBus] alu_b,

    output wire[`DataBus] alu_out,

    output wire exception_req,
    output wire[`DataBus] cause
);

wire[4:0] shamt;
assign shamt = alu_b & 5'b11111;

assign alu_out = (rst == `RstEnable) ? `ZeroWord :
                 (alu_op == `ALU_ADD) ? (alu_a + alu_b) :
                 (alu_op == `ALU_SUB) ? (alu_a - alu_b) :
                 (alu_op == `ALU_SLL) ? (alu_a << shamt) :
                 (alu_op == `ALU_SLT) ? (alu_a[`XLEN-1] > alu_b[`XLEN-1] || (alu_a[`XLEN-1] == alu_b[`XLEN-1] && alu_a < alu_b) ? `OneWord : `ZeroWord) :
                 (alu_op == `ALU_SLTU) ? (alu_a < alu_b ? `OneWord : `ZeroWord) :
                 (alu_op == `ALU_XOR) ? (alu_a ^ alu_b) :
                 (alu_op == `ALU_SRL) ? (alu_a >> shamt) :
                 (alu_op == `ALU_SRA) ? (alu_a[`XLEN-1] ? ((alu_a >> shamt) | (`OneMoreWord - (`OneMoreWord >> shamt))) : (alu_a >> shamt)) :
                 (alu_op == `ALU_OR) ? (alu_a | alu_b) :
                 (alu_op == `ALU_AND) ? (alu_a & alu_b) :
                 (alu_op == `ALU_MIN) ? (alu_a[`XLEN-1] > alu_b[`XLEN-1] || (alu_a[`XLEN-1] == alu_b[`XLEN-1] && alu_a < alu_b) ? alu_a : alu_b) :
                 (alu_op == `ALU_SBSET) ? (alu_a | (`OneWord << (alu_b & (`XLEN - 1)))) :
                 (alu_op == `ALU_CTZ) ? (
                     (alu_a & 32'hffffffff) == 0 ? 32 :
                     (alu_a & 32'h7fffffff) == 0 ? 31 :
                     (alu_a & 32'h3fffffff) == 0 ? 30 :
                     (alu_a & 32'h1fffffff) == 0 ? 29 :
                     (alu_a & 32'h0fffffff) == 0 ? 28 :
                     (alu_a & 32'h07ffffff) == 0 ? 27 :
                     (alu_a & 32'h03ffffff) == 0 ? 26 :
                     (alu_a & 32'h01ffffff) == 0 ? 25 :
                     (alu_a & 32'h00ffffff) == 0 ? 24 :
                     (alu_a & 32'h007fffff) == 0 ? 23 :
                     (alu_a & 32'h003fffff) == 0 ? 22 :
                     (alu_a & 32'h001fffff) == 0 ? 21 :
                     (alu_a & 32'h000fffff) == 0 ? 20 :
                     (alu_a & 32'h0007ffff) == 0 ? 19 :
                     (alu_a & 32'h0003ffff) == 0 ? 18 :
                     (alu_a & 32'h0001ffff) == 0 ? 17 :
                     (alu_a & 32'h0000ffff) == 0 ? 16 :
                     (alu_a & 32'h00007fff) == 0 ? 15 :
                     (alu_a & 32'h00003fff) == 0 ? 14 :
                     (alu_a & 32'h00001fff) == 0 ? 13 :
                     (alu_a & 32'h00000fff) == 0 ? 12 :
                     (alu_a & 32'h000007ff) == 0 ? 11 :
                     (alu_a & 32'h000003ff) == 0 ? 10 :
                     (alu_a & 32'h000001ff) == 0 ? 9 :
                     (alu_a & 32'h000000ff) == 0 ? 8 :
                     (alu_a & 32'h0000007f) == 0 ? 7 :
                     (alu_a & 32'h0000003f) == 0 ? 6 :
                     (alu_a & 32'h0000001f) == 0 ? 5 :
                     (alu_a & 32'h0000000f) == 0 ? 4 :
                     (alu_a & 32'h00000007) == 0 ? 3 :
                     (alu_a & 32'h00000003) == 0 ? 2 :
                     (alu_a & 32'h00000001) == 0 ? 1 : 0
                 ) :
                 `ZeroWord;

assign exception_req = `False_v;
assign cause = `ZeroWord;

endmodule
