`include "../../common.vh"

module csr_reg_file (
    input wire clk,
    input wire rst,

    // for regular use in id stage
    input wire[`PrivLvlBus] priv_lvl_cur, // the current privilege level

    input wire[`CSRAddrBus] csr,
    output wire[`DataBus] csr_rdata,
    output wire[`DataBus] satp_rdata,
    output wire[`DataBus] xstatus_rdata,

    input wire csr_we,
    input wire[`CSRAddrBus] csr_rd,
    input wire[`DataBus] csr_wdata,

    // for use with exception_handler
    // exception priv_lvl
    input wire[`PrivLvlBus] exc_priv_lvl,

    // epc
    input wire xepc_we,
    input wire[`StageBus] pc_sel,
    input wire[159:0] pc_i_1d,
    output wire[`DataBus] xepc_o,

    // cause
    input wire xcause_we,
    input wire[`DataBus] cause_i,
    // output wire[`DataBus] xcause_o,

    // status
    input wire xstatus_we,
    input wire[`DataBus] xstatus_i,
    output wire[`DataBus] xstatus_o,

    // mie/mip
    output wire[`DataBus] xie_o,
    output wire[`DataBus] xip_o,

    // deleg
    output wire[`DataBus] medeleg_o,
    output wire[`DataBus] mideleg_o,

    output wire[`DataBus] xtvec_o,

    // priv lvl 
    input wire priv_lvl_we,
    input wire[`PrivLvlBus] priv_lvl_i,
    output wire[`PrivLvlBus] priv_lvl_o,

    // time for MMIO
    input wire time_we,
    input wire[1:0] time_sel,
    input wire[`DataBus] time_i,
    output wire[`DataBus] time_o
);

reg[`DataBus] csr_regs[0:(`CSRRegNum-1)];
reg[`PrivLvlBus] priv_lvl;

localparam mstatus = `CSRRegNumLog2'd0;
localparam medeleg = `CSRRegNumLog2'd1;
localparam mideleg = `CSRRegNumLog2'd2;
localparam mie = `CSRRegNumLog2'd3;
localparam mtvec = `CSRRegNumLog2'd4;
localparam mscratch = `CSRRegNumLog2'd5;
localparam mepc = `CSRRegNumLog2'd6;
localparam mcause = `CSRRegNumLog2'd7;
localparam mip = `CSRRegNumLog2'd8;

localparam stvec = `CSRRegNumLog2'd9;
localparam sscratch = `CSRRegNumLog2'd10;
localparam sepc = `CSRRegNumLog2'd11;
localparam scause = `CSRRegNumLog2'd12;
localparam stval = `CSRRegNumLog2'd13;
localparam satp = `CSRRegNumLog2'd14;

localparam mtimel = `CSRRegNumLog2'd15;
localparam mtimeh = `CSRRegNumLog2'd16;
localparam mtimecmpl = `CSRRegNumLog2'd17;
localparam mtimecmph = `CSRRegNumLog2'd18;

localparam none = `CSRRegNumLog2'd31;

wire[`CSRRegNumBus] _csr, _csr_rd;
assign _csr = priv_lvl_cur >= `M_MODE ?
        (csr == `CSRMstatus) ? mstatus :
        (csr == `CSRMedeleg) ? medeleg :
        (csr == `CSRMideleg) ? mideleg :
        (csr == `CSRMie) ? mie :
        (csr == `CSRMtvec) ? mtvec :
        (csr == `CSRMscratch) ? mscratch :
        (csr == `CSRMepc) ? mepc :
        (csr == `CSRMcause) ? mcause :
        (csr == `CSRMip) ? mip :
        (csr == `CSRSstatus) ? mstatus :
        (csr == `CSRSie) ? mie :
        (csr == `CSRStvec) ? stvec :
        (csr == `CSRSscratch) ? sscratch :
        (csr == `CSRSepc) ? sepc :
        (csr == `CSRScause) ? scause :
        (csr == `CSRStval) ? stval :
        (csr == `CSRSip) ? mip :
        (csr == `CSRSatp) ? satp :
        (csr == `CSRTime) ? mtimel :
        (csr == `CSRTimeh) ? mtimeh :
        none :
    priv_lvl_cur >= `S_MODE ?
        (csr == `CSRSstatus) ? mstatus :
        (csr == `CSRSie) ? mie :
        (csr == `CSRStvec) ? stvec :
        (csr == `CSRSscratch) ? sscratch :
        (csr == `CSRSepc) ? sepc :
        (csr == `CSRScause) ? scause :
        (csr == `CSRStval) ? stval :
        (csr == `CSRSip) ? mip :
        (csr == `CSRSatp) ? satp :
        (csr == `CSRTime) ? mtimel :
        (csr == `CSRTimeh) ? mtimeh :
        none :
    (csr == `CSRTime) ? mtimel :
    (csr == `CSRTimeh) ? mtimeh :
    none;
    
assign _csr_rd = priv_lvl_cur >= `M_MODE ?
        (csr_rd== `CSRMstatus || csr_rd== `CSRSstatus) ? mstatus :
        (csr_rd== `CSRMedeleg) ? medeleg :
        (csr_rd== `CSRMideleg) ? mideleg :
        (csr_rd== `CSRMie || csr_rd== `CSRSie) ? mie :
        (csr_rd== `CSRMtvec) ? mtvec :
        (csr_rd== `CSRMscratch) ? mscratch :
        (csr_rd== `CSRMepc) ? mepc :
        (csr_rd== `CSRMcause) ? mcause :
        (csr_rd== `CSRMip || csr_rd== `CSRSip) ? mip :
        (csr_rd== `CSRStvec) ? stvec :
        (csr_rd== `CSRSscratch) ? sscratch :
        (csr_rd== `CSRSepc) ? sepc :
        (csr_rd== `CSRScause) ? scause :
        (csr_rd== `CSRStval) ? stval :
        (csr_rd== `CSRSatp) ? satp :
        none :
    priv_lvl_cur >= `S_MODE ?
        (csr_rd== `CSRSstatus) ? mstatus :
        (csr_rd== `CSRSie) ? mie :
        (csr_rd== `CSRStvec) ? stvec :
        (csr_rd== `CSRSscratch) ? sscratch :
        (csr_rd== `CSRSepc) ? sepc :
        (csr_rd== `CSRScause) ? scause :
        (csr_rd== `CSRStval) ? stval :
        (csr_rd== `CSRSatp) ? satp :
        (csr_rd== `CSRSip) ? mip :
        none :
    none;

// time
wire[`DwordBus] mtime, mtimecmp;
assign mtime = {csr_regs[mtimeh], csr_regs[mtimel]};
assign mtimecmp = {csr_regs[mtimecmph], csr_regs[mtimecmpl]};

wire[`CSRRegNumBus] csr_time;
assign csr_time = (time_sel == `MMIOTimeL) ? mtimel :
    (time_sel == `MMIOTimeH) ? mtimeh :
    (time_sel == `MMIOTimeCmpL) ? mtimecmpl :
    (time_sel == `MMIOTimeCmpH) ? mtimecmph :
    mtimel;

// csrs
wire[`DataBus] pc_i[`StallBus];
assign pc_i[`IF] = pc_i_1d[159:128];
assign pc_i[`ID] = pc_i_1d[127:96];
assign pc_i[`EXE] = pc_i_1d[95:64];
assign pc_i[`MEM] = pc_i_1d[63:32];
assign pc_i[`WB] = pc_i_1d[31:0];

localparam MIP_MASK = 32'h00000a2a;
localparam SSTATUS_MASK = 32'h800de162;
localparam SIP_MASK = 32'h00000222;
localparam SIE_MASK = 32'h00000222;

wire[`DataBus] mstatus_i_buf, mie_i_buf, mip_i_buf;
assign mstatus_i_buf = (priv_lvl_cur == `M_MODE) ? csr_wdata :
    (csr_regs[mstatus] & ~SSTATUS_MASK) | (csr_wdata & SSTATUS_MASK);
assign mie_i_buf = (priv_lvl_cur == `M_MODE) ? csr_wdata :
    (csr_regs[mie] & ~SIE_MASK) | (csr_wdata & SIE_MASK);
assign mip_i_buf = (priv_lvl_cur == `M_MODE) ?
    (csr_regs[mip] & ~MIP_MASK) | (csr_wdata & MIP_MASK) | ((mtime >= mtimecmp) << `MTIx) :
    (csr_regs[mip] & ~SIP_MASK) | (csr_wdata & SIP_MASK) | ((mtime >= mtimecmp) << `MTIx);

wire write_conflict;
assign write_conflict = (csr_we == `WriteDisable) ? `False_v :
    (xepc_we == `WriteEnable && ((exc_priv_lvl == `M_MODE && _csr_rd == mepc) || (exc_priv_lvl == `S_MODE && _csr_rd == sepc))) ||
    (xcause_we == `WriteEnable && ((exc_priv_lvl == `M_MODE && _csr_rd == mcause) || (exc_priv_lvl == `S_MODE && _csr_rd == scause))) ||
    (xstatus_we == `WriteEnable && _csr_rd == mcause);

integer i;
always @ (posedge rst or posedge clk) begin
    if (rst == `RstEnable) begin
        for (i = 0; i < `CSRRegNum; i = i + 1) begin
            csr_regs[i] <= `ZeroWord;
        end
        priv_lvl <= `M_MODE;
    end else begin
        // time counters
        if (time_we == `WriteEnable && (csr_time == mtimeh || csr_time == mtimel))
            csr_regs[csr_time] <= time_i;
        else begin
            {csr_regs[mtimeh], csr_regs[mtimel]} <= mtime + 1;
            if (time_we == `WriteEnable) // mtimecmpl or mtimecmph
                csr_regs[csr_time] <= time_i;
        end

        // other csrs
        if (xepc_we == `WriteEnable) begin
            if (exc_priv_lvl == `M_MODE)
                csr_regs[mepc] <= pc_i[pc_sel];
            else if (exc_priv_lvl == `S_MODE)
                csr_regs[sepc] <= pc_i[pc_sel];
        end
        if (xcause_we == `WriteEnable) begin
            if (exc_priv_lvl == `M_MODE)
                csr_regs[mcause] <= cause_i;
            else if (exc_priv_lvl == `S_MODE)
                csr_regs[scause] <= cause_i;
        end
        if (xstatus_we == `WriteEnable) begin
            if (exc_priv_lvl == `M_MODE)
                csr_regs[mstatus] <= xstatus_i;
            else if (exc_priv_lvl == `S_MODE)
                csr_regs[mstatus] <= (csr_regs[mstatus] & ~SSTATUS_MASK) | (xstatus_i & SSTATUS_MASK);
        end
        if (~write_conflict && csr_we == `WriteEnable) begin
            if (_csr_rd == mstatus)
                csr_regs[_csr_rd] <= mstatus_i_buf;
            else if (_csr_rd == mie)
                csr_regs[_csr_rd] <= mie_i_buf;
            else if (_csr_rd == mip)
                csr_regs[_csr_rd] <= mip_i_buf;
            else
                csr_regs[_csr_rd] <= csr_wdata;
        end else begin // write to mip anyways
            csr_regs[mip][`MTIx] <= mtime >= mtimecmp;
        end

        // priv lvl
        if (priv_lvl_we == `WriteEnable)
            priv_lvl <= priv_lvl_i;
    end
end

assign csr_rdata = (csr_we == `WriteEnable && _csr_rd == _csr) ? csr_wdata : csr_regs[_csr];

assign xepc_o = (exc_priv_lvl == `M_MODE) ?
    (csr_we == `WriteEnable && _csr_rd == mepc) ? csr_wdata : csr_regs[mepc] :
    (csr_we == `WriteEnable && _csr_rd == sepc) ? csr_wdata : csr_regs[sepc];
assign xstatus_o = (csr_we == `WriteEnable && _csr_rd == mstatus) ? mstatus_i_buf : csr_regs[mstatus];
assign xie_o = (csr_we == `WriteEnable && _csr_rd == mie) ? mie_i_buf : csr_regs[mie];
assign xip_o = (csr_we == `WriteEnable && _csr_rd == mip) ? mip_i_buf : csr_regs[mip];
assign medeleg_o = (csr_we == `WriteEnable && _csr_rd == medeleg) ? csr_wdata : csr_regs[medeleg];
assign mideleg_o = (csr_we == `WriteEnable && _csr_rd == mideleg) ? csr_wdata : csr_regs[mideleg];
// assign xstatus_o = csr_regs[mstatus];
// assign xie_o = csr_regs[mie];
// assign xip_o = csr_regs[mip];
// assign medeleg_o = csr_regs[medeleg];
// assign mideleg_o = csr_regs[mideleg];
assign xtvec_o = (exc_priv_lvl == `M_MODE) ?
    (csr_we == `WriteEnable && _csr_rd == mtvec) ? csr_wdata : csr_regs[mtvec] :
    (csr_we == `WriteEnable && _csr_rd == stvec) ? csr_wdata : csr_regs[stvec];

assign time_o = csr_regs[csr_time];

assign satp_rdata = (csr_we == `WriteEnable && _csr_rd == satp) ? csr_wdata : csr_regs[satp];
assign priv_lvl_o = priv_lvl;
endmodule