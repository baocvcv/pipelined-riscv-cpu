`include "../../common.vh"
`include "id.vh"

//TODO: raise exception, handle new instructions rdtime[h], sret, ...
module id (
    input wire rst,
    input wire clk,
    // input wire stall,

    input wire[`PrivLvlBus] priv_lvl,
    input wire[`DataBus] pc,
    input wire[`DataBus] inst,

    // data forwarding
    input wire[`DataBus] ex_alu_out,
    input wire[`DataBus] mem_data_out,
    input wire[`DataBus] mem_alu_out,

    // from wb
    input wire wb_reg_write,
    input wire[`RegAddrBus] wb_rd,
    input wire[`DataBus] wb_wdata,

    // from hazard_control
    input wire[`ForwardSelBus] forward1,
    input wire[`ForwardSelBus] forward2,

    // to exe
    output wire[`AluOpBus] alu_op,
    output wire[`DataBus] alu_a,
    output wire[`DataBus] alu_b,

    output wire[`DataBus] data_rs2,
    output wire[`RegAddrBus] rd,

    output wire reg_write,
    output wire reg_write_sel,
    output wire mem_read,
    output wire mem_write,
    output wire[`Func3Bus] mem_option,

    // to if
    output wire branch_flag,
    output wire[`DataBus] branch_target_addr,

    // to ctrl
    output wire[`OpcodeBus] opcode,
    output wire[`Func3Bus] func3,
    output wire[`RegAddrBus] rs1,
    output wire[`RegAddrBus] rs2,

    // to csr reg
    output wire[`CSRAddrBus] csr,
    input wire[`DataBus] csr_rdata_buf,
    output wire csr_we,
    output wire[`CSRAddrBus] csr_rd,
    output wire[`DataBus] csr_wdata,

    // csr data forward
    input wire[`ForwardSelBus] csr_forward,
    input wire[`DataBus] ex_csr_wdata,
    // input wire[`DataBus] mem_csr_wdata,

    // exception request
    output wire exception_req,
    output wire[`DataBus] cause
);

// instruction
wire[`Func7Bus] func7;
wire[`Func12Bus] func12;
assign opcode = inst[`ID_OPCODE];
assign func3 = inst[`ID_FUNC3];
assign func7 = inst[`ID_FUNC7];
assign func12 = inst[`ID_FUNC12];

assign rs1 = inst[`ID_RS1];
assign rs2 = inst[`ID_RS2];
assign rd = inst[`ID_RD];

// assign csr = (csr_sel == `CSR_MRET) ? `CSRMepc :
//              (csr_sel == `CSR_JUMP) ? `CSRMtvec :
//              inst[`ID_CSR];
// assign csr_rd = (csr_sel == `CSR_MRET) ? `CSRMstatus :
//                 (csr_sel == `CSR_JUMP) ? `CSRMcause :
//                 inst[`ID_CSR];

// register file
wire[`DataBus] rdata1, rdata2;
reg_file _reg_file (
    .clk(clk),
    .rst(rst),
    .we_n(wb_reg_write),
    .rd(wb_rd),
    .wdata(wb_wdata),
    .rs1(rs1),
    .rdata1(rdata1),
    .rs2(rs2),
    .rdata2(rdata2)
);

// local control
wire[`AluSelBus] alu_a_src_sel, alu_b_src_sel;
wire[`BranchAddrOpBus] branch_addr_op;
wire[`ImmTypeBus] imm_type;
wire priv_lvl_sel;

wire _is_csr_op;
assign _is_csr_op = (opcode == `OP_SYSTEM && (func3 == `FUNC3_CSRRW || func3 == `FUNC3_CSRRS || func3 == `FUNC3_CSRRC));

// lui: reg[rd] <- 0 + imm
// jal, jalr: reg[rd] <- pc + 4
// auipc: reg[rd] <- pc + imm
assign alu_a_src_sel = (opcode == `OP_LUI || _is_csr_op) ? `ALU_A_0 :
                       (opcode == `OP_AUIPC || opcode == `OP_JAL || opcode == `OP_JALR) ? `ALU_A_PC :
                       `ALU_A_REG;
assign alu_b_src_sel = (opcode == `OP_JAL || opcode == `OP_JALR) ? `ALU_B_4 :
                       (opcode == `OP_ALU) ? `ALU_B_REG :
                       _is_csr_op ? `ALU_B_CSR :
                       `ALU_B_IMM;
assign alu_op = (opcode == `OP_ALU || opcode == `OP_ALU_IMM) ? (
                    (func3 == `FUNC3_ADD) ?
                        opcode == `OP_ALU && func7 == `FUNC7_MOD ? `ALU_SUB : `ALU_ADD :
                    (func3 == `FUNC3_SLL) ?
                        ((func7 == `FUNC7_SBSET) ? `ALU_SBSET :
                        (func7 == `FUNC7_CTZ) ? `ALU_CTZ :
                        `ALU_SLL) :
                    (func3 == `FUNC3_SLT) ? `ALU_SLT :
                    (func3 == `FUNC3_SLTU) ? `ALU_SLTU :
                    (func3 == `FUNC3_XOR) ? ((func7 == `FUNC7_MIN) ? `ALU_MIN : `ALU_XOR) :
                    (func3 == `FUNC3_SRL) ?
                        func7 == `FUNC7_MOD ? `ALU_SRA : `ALU_SRL :
                    (func3 == `FUNC3_OR) ? `ALU_OR :
                    (func3 == `FUNC3_AND) ? `ALU_AND :
                    `ALU_ADD) :
                `ALU_ADD;
assign reg_write = (opcode == `OP_BRANCH || opcode == `OP_STORE) ? `WriteDisable :
                   (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV) ? `WriteDisable :
                   `WriteEnable;
assign reg_write_sel = (opcode == `OP_LOAD) ? `RegWriteSrcMem : `RegWriteSrcAlu;

assign branch_addr_op = (opcode == `OP_JALR) ? `BRANCH_ADDR_REG :
                        (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV) ? `BRANCH_ADDR_CSR :
                        `BRANCH_ADDR_PC;

assign mem_read = (opcode != `OP_LOAD) ? `ReadDisable : `ReadEnable;
assign mem_write =  (opcode != `OP_STORE) ? `WriteDisable : `WriteEnable;
assign mem_option = (opcode == `OP_LOAD || opcode == `OP_STORE) ? func3 : `FUNC3_WORD;

assign imm_type = (opcode == `OP_STORE) ? `IMM_S :
                  (opcode == `OP_BRANCH) ? `IMM_B :
                  (opcode == `OP_LUI || opcode == `OP_AUIPC) ? `IMM_U :
                  (opcode == `OP_JAL) ? `IMM_J :
                  (opcode == `OP_ALU_IMM) ? (
                    (func3 == `FUNC3_SLL || func3 == `FUNC3_SRL) ? `IMM_SHAMT : `IMM_I) :
                  `IMM_I;

// assign csr_sel = (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV) ?
//                     (func12 == `FUNC12_MRET) ? `CSR_MRET :
//                     (func12 == `FUNC12_ECALL || func12 == `FUNC12_EBREAK) ? `CSR_JUMP :
//                     `CSR_CSRRW :
//                  `CSR_CSRRW; 
// assign csr_wdata_sel = (func3 == `FUNC3_CSRRW) ? `CSR_WDATA_SEL_RW :
//                        (func3 == `FUNC3_CSRRS) ? `CSR_WDATA_SEL_RS :
//                        (func3 == `FUNC3_CSRRC) ? `CSR_WDATA_SEL_RC :
//                        (func3 == `FUNC3_PRIV) ?
//                            (func12 == `FUNC12_MRET) ? `CSR_WDATA_SEL_MSTATUS : `CSR_WDATA_SEL_MCAUSE :
//                        `CSR_WDATA_SEL_RW;
// assign mepc_we = (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV && (func12 == `FUNC12_EBREAK || func12 == `FUNC12_ECALL)) ? `WriteEnable : `WriteDisable;

// assign priv_lvl_sel = (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV && func12 == `FUNC12_MRET) ? `PRIV_LVL_SRC_MPP : `PRIV_LVL_SRC_MMODE;
// assign priv_lvl_we = (stall == `Stop) ? `WriteDisable :
//     (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV && (func12 == `FUNC12_MRET || func12 == `FUNC12_ECALL || func12 == `FUNC12_EBREAK)) ? `WriteEnable :
//     `WriteDisable;

// data forwarding
wire[`DataBus] data_rs1;
assign data_rs1 = (forward1 == `FOR_ALU) ? ex_alu_out :
                  (forward1 == `FOR_MEM) ? mem_data_out :
                  (forward1 == `FOR_MEM_ALU) ? mem_alu_out :
                  rdata1;
assign data_rs2 = (forward2 == `FOR_ALU) ? ex_alu_out :
                  (forward2 == `FOR_MEM) ? mem_data_out :
                  (forward2 == `FOR_MEM_ALU) ? mem_alu_out :
                  rdata2;

// alu src sel
assign alu_a = (alu_a_src_sel == `ALU_A_REG) ? data_rs1 :
               (alu_a_src_sel == `ALU_A_PC) ? pc : `ZeroWord;
assign alu_b = (alu_b_src_sel == `ALU_B_REG) ? data_rs2 :
               (alu_b_src_sel == `ALU_B_4) ? 4 :
               (alu_b_src_sel == `ALU_B_CSR) ? csr_rdata :
               imm;

// immediate
wire[`DataBus] imm;
assign imm = (imm_type == `IMM_I) ? (inst[31] ? {20'hfffff, inst[31:20]} : {20'h00000, inst[31:20]}) :
             (imm_type == `IMM_S) ? (inst[31] ? {20'hfffff, inst[31:25], inst[11:7]} : {20'h00000, inst[31:25], inst[11:7]}) :
             (imm_type == `IMM_B) ? (inst[31] ? {20'hfffff, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0} : {20'h00000, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}) :
             (imm_type == `IMM_U) ? (inst[31] ? {inst[31:12], 12'h000} : {inst[31:12], 12'h000}) :
             (imm_type == `IMM_J) ? (inst[31] ? {12'hfff, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0} : {12'h000, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}) :
             (imm_type == `IMM_SHAMT) ? {27'h0000000, inst[24:20]} :
             `ZeroWord;

// branch alu
wire signed[`DataBus] data_rs1_s, data_rs2_s; // signed version
assign data_rs1_s = data_rs1;
assign data_rs2_s = data_rs2;
// assign branch_flag = (stall == `Stop) ? `BranchNotTaken :
assign branch_flag = (opcode == `OP_BRANCH) ? (
                        (func3 == `FUNC3_BEQ && data_rs1 == data_rs2) ? `BranchTaken : 
                        (func3 == `FUNC3_BNE && data_rs1 != data_rs2) ? `BranchTaken : 
                        (func3 == `FUNC3_BLT && data_rs1_s < data_rs2_s) ? `BranchTaken : 
                        (func3 == `FUNC3_BGE && data_rs1_s >= data_rs2_s) ? `BranchTaken :
                        (func3 == `FUNC3_BLTU && data_rs1 < data_rs2) ? `BranchTaken : 
                        (func3 == `FUNC3_BGEU && data_rs1 >= data_rs2) ? `BranchTaken :
                        `BranchNotTaken) :
                     (opcode == `OP_JAL || opcode == `OP_JALR) ? `BranchTaken :
                    //  (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV) ?
                    //     (func7 == `FUNC7_SFENCE ? `BranchNotTaken : `BranchTaken) :
                     `BranchNotTaken;

assign branch_target_addr = (branch_addr_op == `BRANCH_ADDR_PC) ? (pc + imm) :
                            (branch_addr_op == `BRANCH_ADDR_REG) ? (data_rs1 + imm) :
                            // (branch_addr_op == `BRANCH_ADDR_CSR) ? csr_rdata :
                            `ZeroWord;

// csr
wire[`DataBus] csr_rdata;
assign csr = inst[`ID_CSR];
assign csr_rd = inst[`ID_CSR];
assign csr_rdata =
    csr_forward == `FOR_ALU ? ex_csr_wdata :
    // csr_forward == `FOR_MEM ? mem_csr_wdata :
    csr_rdata_buf;
assign csr_we = (opcode != `OP_SYSTEM) ? `WriteDisable :
                (func3 == `FUNC3_CSRRW || func3 == `FUNC3_CSRRWI) ? `WriteEnable :
                ((func3 == `FUNC3_CSRRS || func3 == `FUNC3_CSRRC || func3 == `FUNC3_CSRRSI || func3 == `FUNC3_CSRRCI) && (rs1 != `RegX0)) ? `WriteEnable :
                // (func3 == `FUNC3_PRIV) ? `WriteEnable :
                `WriteDisable;
assign csr_wdata = (func3 == `FUNC3_CSRRW) ? data_rs1 :                
                   (func3 == `FUNC3_CSRRS) ? (data_rs1 | csr_rdata) :                
                   (func3 == `FUNC3_CSRRC) ? ((~data_rs1) & csr_rdata) :                
                   (func3 == `FUNC3_CSRRWI) ? {27'h0000000, rs1} :                
                   (func3 == `FUNC3_CSRRSI) ? (rs1 | csr_rdata) :                
                   (func3 == `FUNC3_CSRRCI) ? (rs1 & csr_rdata) :                
                //    (csr_wdata_sel == `CSR_WDATA_SEL_MSTATUS) ? `ZeroWord :
                //    (csr_wdata_sel == `CSR_WDATA_SEL_MCAUSE) ? (
                //     func12 == `FUNC12_ECALL ? {1'b0, `ExcEcallU} :
                //     func12 == `FUNC12_EBREAK ? {1'b0, `ExcBreak} :
                //     `ZeroWord) :
                  `ZeroWord;

// exception
wire invalid_inst;
assign invalid_inst = 
    opcode == `OP_LUI || opcode == `OP_AUIPC || opcode == `OP_JAL ? `False_v :
    opcode == `OP_JALR && func3 == 3'b000 ? `False_v :
    opcode == `OP_BRANCH ? `False_v :
    opcode == `OP_LOAD || opcode == `OP_STORE ? `False_v :
    opcode == `OP_ALU || opcode == `OP_ALU_IMM ? `False_v :
    opcode == `OP_SYSTEM && func3 != 3'b100 ? `False_v :
    opcode == `OP_FENCE ? `False_v :
    `True_v;

assign exception_req = (opcode == `OP_SYSTEM && func3 == `FUNC3_PRIV && func7 != `FUNC7_SFENCE);
assign cause = {
    `ExcException,
    func12 == `FUNC12_SRET ? `ExcSret :
    func12 == `FUNC12_MRET ? `ExcMret :
    func12 == `FUNC12_ECALL && priv_lvl == `U_MODE ? `ExcEcallU :
    func12 == `FUNC12_ECALL && priv_lvl == `S_MODE ? `ExcEcallS :
    func12 == `FUNC12_EBREAK ? `ExcBreak :
    `ExcIllegalInst
};

endmodule