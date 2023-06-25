`include "../../common.vh"
`include "id.vh"

module exception_handler (
    input wire rst,
    input wire clk,

    // status of each stage
    input wire[9:0] priv_lvls_1d,
    input wire[`StallBus] stalls,

    // csr write ops
    input wire ex_csr_we,
    input wire mem_csr_we,

    // exception requests from all stages
    input wire[`StallBus] exc_req,
    input wire[159:0] exc_causes_1d,

    // to if
    output wire exc_flag,
    output wire[`DataBus] exc_target_address,

    // to hazard control
    output wire[`StageBus] flush_up_to,

    /* to and from csr_reg_file */
    output wire[`PrivLvlBus] exc_priv_lvl,

    output wire xepc_we,
    output wire[`StageBus] pc_sel,
    input wire[`DataBus] xepc,

    output wire xcause_we,
    output wire[`DataBus] xcause_o,

    output wire xstatus_we,
    output wire[`DataBus] xstatus_o,
    input wire[`DataBus] xstatus_i,

    input wire[`DataBus] xip,
    input wire[`DataBus] xie,

    input wire[`DataBus] medeleg,
    input wire[`DataBus] mideleg,
    input wire[`DataBus] xtvec,

    output wire priv_lvl_we,
    output wire[`PrivLvlBus] priv_lvl_o
);

// reshape
wire[`PrivLvlBus] priv_lvls[`StallBus];
assign priv_lvls[`IF] = priv_lvls_1d[9:8];
assign priv_lvls[`ID] = priv_lvls_1d[7:6];
assign priv_lvls[`EXE] = priv_lvls_1d[5:4];
assign priv_lvls[`MEM] = priv_lvls_1d[3:2];
assign priv_lvls[`WB] = priv_lvls_1d[1:0];
wire[`DataBus] exc_causes[`StallBus];
assign exc_causes[`IF] = exc_causes_1d[159:128];
assign exc_causes[`ID] = exc_causes_1d[127:96];
assign exc_causes[`EXE] = exc_causes_1d[95:64];
assign exc_causes[`MEM] = exc_causes_1d[63:32];
assign exc_causes[`WB] = exc_causes_1d[31:0];


// check timing interrupt
localparam timer_stage = 3;
wire[`PrivLvlBus] timer_priv;
assign timer_priv = priv_lvls[timer_stage];

wire has_m_timing_int, has_s_timing_int;
assign has_m_timing_int = xip[`MTIx] && xie[`MTIx] && (xstatus_i[`MSTATUS_MIE] || timer_priv < `M_MODE);
assign has_s_timing_int = xip[`STIx] && xie[`STIx] && ((xstatus_i[`MSTATUS_SIE] && timer_priv == `S_MODE) || timer_priv < `S_MODE);

// select an exception to handle
wire[`StageBus] exc_stage;
assign exc_stage = 
    // exc_req[5] ? 5 :
    exc_req[4] ? 4 :
    (has_m_timing_int || has_s_timing_int) ? timer_stage :
    // exc_req[3] ? 3 :
    exc_req[2] ? 2 :
    // exc_req[1] ? 1 :
    0;

wire has_exception;
assign has_exception = exc_stage > 0;

// exc_cause: timer int, page fault, ecall/ebreak, (mret/sret)
wire[`DataBus] exc_cause;
assign exc_cause =
    has_exception ?
        (exc_stage != timer_stage) ? exc_causes[exc_stage] :
        has_m_timing_int ? {`ExcInterrupt, `ExcMTimeInt} :
        has_s_timing_int ? {`ExcInterrupt, `ExcSTimeInt} :
        `ZeroWord :
    `ZeroWord;

// set priv lvl to handle exception
assign exc_priv_lvl = (exc_cause[`MCAUSE_CODE] == `ExcMret) ? `M_MODE :
    (exc_cause[`MCAUSE_CODE] == `ExcSret) ? `S_MODE :
    (exc_cause[`MCAUSE_TYPE] == `MCAUSE_INT && mideleg[exc_cause[`MCAUSE_CODE]]) ? `S_MODE :
    (exc_cause[`MCAUSE_TYPE] == `MCAUSE_TRAP && medeleg[exc_cause[`MCAUSE_CODE]]) ? `S_MODE :
    `M_MODE;

// wait for respective stalls to finish
// if interrupt, wait until priv lvls are consistent
wire wait_for_stall;
assign wait_for_stall = (exc_stage == 3 && stalls[3]) || (exc_stage == 2 && stalls[2]) ||
    (exc_stage == 3 && timer_priv != priv_lvls[`IF]) ||
    (exc_stage == 3 && mem_csr_we == `WriteEnable) ||
    (exc_stage == 2 && (ex_csr_we == `WriteEnable || mem_csr_we == `WriteEnable));

wire is_ret;
assign is_ret = exc_cause[`MCAUSE_CODE] == `ExcMret || exc_cause[`MCAUSE_CODE] == `ExcSret;

// change pc
assign exc_flag = has_exception && ~wait_for_stall;
assign exc_target_address =
    is_ret ? xepc :
    (exc_cause[`MCAUSE_TYPE] == `MCAUSE_INT) ?
        (xtvec[`MTVEC_MODE] == `MTVEC_DIRECT) ? {xtvec[`MTVEC_BASE], 2'b00} : {xtvec[`MTVEC_BASE] + exc_cause[`MCAUSE_CODE], 2'b00} :
    {xtvec[`MTVEC_BASE], 2'b00};

// flush
assign flush_up_to = exc_flag ? exc_stage : 0;

// change csr register values
assign xepc_we = exc_flag && ~is_ret ? `WriteEnable : `WriteDisable;
assign pc_sel = exc_stage;

wire[`PrivLvlBus] cur_priv;
assign cur_priv = exc_stage > 0 ? priv_lvls[exc_stage] : `U_MODE;

assign xcause_we = exc_flag && ~is_ret ? `WriteEnable : `WriteDisable;
assign xcause_o = exc_cause;

assign xstatus_we = exc_flag ? `WriteEnable : `WriteDisable;
assign xstatus_o =
    (exc_cause[`MCAUSE_CODE] == `ExcMret) ? {xstatus_i[31:13], `U_MODE, xstatus_i[10:8], 1'b1, xstatus_i[6:4], xstatus_i[`MSTATUS_MPIE], xstatus_i[2:0]} :
    (exc_cause[`MCAUSE_CODE] == `ExcSret) ? {xstatus_i[31:9], 1'b0, xstatus_i[7:6], 1'b1, xstatus_i[4:2], xstatus_i[`MSTATUS_SPIE], xstatus_i[0]} :
    (exc_priv_lvl == `M_MODE) ? {xstatus_i[31:13], cur_priv, xstatus_i[10:8], xstatus_i[`MSTATUS_MIE], xstatus_i[6:4], 1'b0, xstatus_i[2:0]} :
    (exc_priv_lvl == `S_MODE) ? {xstatus_i[31:9], cur_priv[0], xstatus_i[7:6], xstatus_i[`MSTATUS_SIE], xstatus_i[4:2], 1'b0, xstatus_i[0]} :
    xstatus_i;

assign priv_lvl_we = exc_flag ? `WriteEnable : `WriteDisable;
assign priv_lvl_o =
    (exc_cause[`MCAUSE_CODE] == `ExcMret) ? xstatus_i[`MSTATUS_MPP] :
    (exc_cause[`MCAUSE_CODE] == `ExcSret) ? {1'b0, xstatus_i[`MSTATUS_SPP]} :
    exc_priv_lvl;

endmodule