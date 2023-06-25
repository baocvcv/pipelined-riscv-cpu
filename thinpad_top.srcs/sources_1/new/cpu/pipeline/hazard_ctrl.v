`include "../../common.vh"
`include "id.vh"

module hazard_ctrl (
    input wire clk,
    input wire rst,

    // from if
    input wire if_update_tlb,
    input wire if_is_store,

    // from id
    input wire[`OpcodeBus] id_opcode,
    input wire[`Func3Bus] id_func3,
    input wire[`RegAddrBus] id_rs1,
    input wire[`RegAddrBus] id_rs2,
    input wire id_branch_flag,

    // from exe
    input wire ex_reg_write,
    input wire[`RegAddrBus] ex_rd,
    input wire ex_mem_read,
    input wire ex_mem_write,

    // from mem
    input wire mem_reg_write,
    input wire[`RegAddrBus] mem_rd,
    input wire mem_mem_read,
    input wire mem_mem_write,

    // from mmu
    input wire mem_controller_busy,
    input wire tlb_update_done,

    // from exception control
    input wire[`StageBus] flush_up_to,

    // csr
    input wire[`CSRAddrBus] id_csr,
    input wire ex_csr_we,
    input wire[`CSRAddrBus] ex_csr_rd,
    // input wire mem_csr_we,
    // input wire[`CSRAddrBus] mem_csr_rd,
    output wire[`ForwardSelBus] csr_forward,

    // to if
    output wire[`ForwardSelBus] forward1,
    output wire[`ForwardSelBus] forward2,   

    // bubble and stall
    output wire[`StallBus] bubble,
    output wire[`StallBus] stall,

    // to tlb
    output wire update_tlb
);

wire id_rs1_read, id_rs2_read;
assign id_rs1_read = (id_opcode == `OP_LUI || id_opcode == `OP_AUIPC || id_opcode == `OP_JAL) ? `ReadDisable : `ReadEnable;
assign id_rs2_read = (id_opcode == `OP_BRANCH || id_opcode == `OP_STORE || id_opcode == `OP_ALU) ? `ReadEnable : `ReadDisable;

assign forward1 = (ex_reg_write == `WriteEnable && ex_mem_read == `ReadDisable && id_rs1 != `RegX0 && ex_rd == id_rs1) ? `FOR_ALU : // RAW
                  (mem_reg_write == `WriteEnable && mem_mem_read == `ReadDisable && id_rs1 != `RegX0 && mem_rd == id_rs1) ? `FOR_MEM_ALU : // RAW
                  (mem_mem_read == `ReadEnable && mem_rd == id_rs1) ? `FOR_MEM : // load-use
                  `FOR_REG;
assign forward2 = (ex_reg_write == `WriteEnable && ex_mem_read == `ReadDisable && id_rs2 != `RegX0 && ex_rd == id_rs2) ? `FOR_ALU : // RAW
                  (mem_reg_write == `WriteEnable && mem_mem_read == `ReadDisable && id_rs2 != `RegX0 && mem_rd == id_rs2) ? `FOR_MEM_ALU : // RAW
                  (mem_mem_read == `ReadEnable && mem_rd == id_rs2) ? `FOR_MEM : // load-use
                  `FOR_REG;

assign csr_forward =
    (ex_csr_we == `WriteEnable && ex_csr_rd == id_csr) ? `FOR_ALU :
    // (mem_csr_we == `WriteEnable && mem_csr_rd == id_csr) ? `FOR_MEM :
    `FOR_REG;

assign stall[5] = 1'b0;

wire stall_4;
assign stall_4 = mem_controller_busy;
assign stall[4] = stall_4;

assign stall[3] = stall_4 || mem_mem_read == `ReadEnable || mem_mem_write == `WriteEnable;

wire stall_2;
assign stall_2 = (ex_reg_write == `WriteEnable && ex_mem_read == `ReadEnable && ((id_rs1 != `RegX0 && id_rs1 == ex_rd) || (id_rs2 != `RegX0 && id_rs2 == ex_rd))) || // load-use
                 mem_mem_read == `ReadEnable || mem_mem_write == `WriteEnable || mem_controller_busy; // memory busy
assign stall[2] = stall_2;

wire stall_1;
assign stall_1 = stall_2 || (if_update_tlb && (~tlb_update_done));
assign stall[1] = stall_1;

// assign bubble = (rst == `RstEnable) ? `BubbleEnable :
//                 ((id_opcode == `OP_BRANCH && id_branch_flag == `BranchTaken) || id_opcode == `OP_JAL || id_opcode == `OP_JALR) ?
//                     ((stall[`Stall2] == `Stop) ? `BubbleDisable : `BubbleEnable) : // if id is stalled do not insert bubble
//                 (id_opcode == `OP_SYSTEM && id_func3 == `FUNC3_PRIV && id_branch_flag == `BranchTaken) ?
//                     ((stall[`Stall2] == `Stop) ? `BubbleDisable : `BubbleEnable) :
//                 `BubbleDisable;

//TODO
assign bubble[5] = stall[4] || (flush_up_to >= 5);
assign bubble[4] = stall[3] || (flush_up_to >= 4);
assign bubble[3] = stall[2] || (flush_up_to >= 3);
assign bubble[2] = stall[1] || (id_branch_flag == `BranchTaken) || (flush_up_to >= 2);
assign bubble[1] = (flush_up_to >= 1);

assign update_tlb = (if_update_tlb == `True_v &&
                    id_opcode != `OP_LOAD && id_opcode != `OP_STORE &&
                    ex_mem_read == `ReadDisable && ex_mem_write == `WriteDisable &&
                    mem_mem_read == `ReadDisable && mem_mem_write == `WriteDisable &&
                    (~mem_controller_busy)) ? `True_v : `False_v;

endmodule
