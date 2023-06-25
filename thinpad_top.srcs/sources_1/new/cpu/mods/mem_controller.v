`default_nettype none
`include "../../common.vh"
`include "../pipeline/id.vh"

`define StateLen 3
`define StateBus (`StateLen-1):0

module mem_controller(
    input wire clk,
    input wire rst,

    output wire busy,

    input wire sum,
    input wire[`DataBus] pc,
    input wire[`PrivLvlBus] pc_priv_lvl,
    input wire[`DataBus] mem_addr,
    input wire[`PrivLvlBus] mem_priv_lvl,

    input wire[`Func3Bus] mem_option,
    input wire[`DataBus] data_in,
    output wire[`DataBus] data_out,

    input wire oe_n,
    input wire we_n,
    
    input wire[`DataBus] satp,

    input wire update_tlb,
    output reg update_done,

    output wire if_exception_req,
    output wire[`DataBus] if_cause,
    output wire mem_exception_req,
    output wire[`DataBus] mem_cause,

    // csr regs
    output wire time_we,
    output wire[1:0] time_sel,
    output wire[`DataBus] time_wdata,
    input wire[`DataBus] time_i,

    // cpld uart
    output reg uart_rdn,
    output reg uart_wrn,
    input wire uart_dataready,
    input wire uart_tbre,
    input wire uart_tsre,

    // base ram
    inout wire[`DataBus] base_ram_data,
    output wire[`RamDataBus] base_ram_addr,
    output wire[`ByteEnBus] base_ram_be_n,
    output wire base_ram_ce_n,
    output wire base_ram_oe_n,
    output wire base_ram_we_n,

    // ext ram
    inout wire[`DataBus] ext_ram_data,
    output wire[`RamDataBus] ext_ram_addr,
    output wire[`ByteEnBus] ext_ram_be_n,
    output wire ext_ram_ce_n,
    output wire ext_ram_oe_n,
    output wire ext_ram_we_n
);

// states
reg[`StateBus] uart_state;
localparam uart_state_begin = `StateLen'b000;
localparam uart_read_done = `StateLen'b001;
localparam uart_write_stall = `StateLen'b010;
localparam uart_write_start = `StateLen'b011;
localparam uart_write_done = `StateLen'b100;

reg mem_write_state;
localparam mem_write_begin = 1'b0;
localparam mem_write_done = 1'b1;

// width flags
wire read_write_byte, read_write_hw, read_write_word;
assign read_write_byte = (mem_option == `FUNC3_BYTE || mem_option == `FUNC3_BYTE_U);
assign read_write_hw = (mem_option == `FUNC3_HALF_WORD || mem_option == `FUNC3_HALF_WORD_U);
assign read_write_word = mem_option == `FUNC3_WORD;

wire read_write_signed;
assign read_write_signed = (mem_option == `FUNC3_BYTE || mem_option == `FUNC3_HALF_WORD);

wire tlb_miss;
assign busy = (uart_state == uart_state_begin && mem_addr == UART_DATA_ADDR && read_write_byte && (oe_n == `ReadEnable || we_n == `WriteEnable)) ||
              uart_state == uart_write_stall || uart_state == uart_write_start || (uart_state == uart_write_done && (~uart_tsre)) ||
              (mem_write_state == mem_write_begin && (~accessing_MMIO) && (~tlb_miss) && we_n == `WriteEnable) ||
              tlb_miss;

// MMIO
localparam UART_DATA_ADDR = `XLEN'h10000000;
localparam UART_STATUS_ADDR = `XLEN'h10000005;
localparam TIMEL_ADDR = `XLEN'h0200bff8;
localparam TIMEH_ADDR = `XLEN'h0200bffc;
localparam TIMECMPL_ADDR = `XLEN'h02004000;
localparam TIMECMPH_ADDR = `XLEN'h02004004;

localparam EXT_EN = 22; // if addr[EXT_EN] == 1, then r/w ext_ram

// MMIO flags
wire is_uart; // whether mem_addr is uart
assign is_uart = mem_addr[`XLEN-1:`XLEN-4] == 4'h1;

wire accessing_uart; // whether we are accessing uart
assign accessing_uart = (uart_state != uart_state_begin) ||
                        ((oe_n == `ReadEnable || we_n == `WriteEnable) && read_write_byte && is_uart);

wire accessing_time;
assign accessing_time = (oe_n == `ReadEnable || we_n == `WriteEnable) &&
    (mem_addr == TIMEL_ADDR || mem_addr == TIMEH_ADDR || mem_addr == TIMECMPL_ADDR || mem_addr == TIMECMPH_ADDR);

wire accessing_MMIO;
assign accessing_MMIO = accessing_uart || accessing_time;

// addr for ram
wire[`DataBus] physical_addr; // addr translated by tlb

// mem flags
wire is_ext;
assign is_ext = physical_addr[EXT_EN] == 1'b1;

// stall for mem write
always @ (posedge clk or posedge rst) begin
    if (rst) begin
        mem_write_state <= mem_write_begin;
    end else begin
        case (mem_write_state) 
            mem_write_begin: begin
                if (~accessing_MMIO && ~tlb_miss && we_n == `WriteEnable) begin
                    mem_write_state <= mem_write_done;
                end
            end
            mem_write_done: begin
                mem_write_state <= mem_write_begin;
            end
        endcase
    end
end

wire mem_we_n;
assign mem_we_n = (mem_write_state == mem_write_begin) ? we_n : `WriteDisable;

// base ram r/w flag
assign base_ram_ce_n = 1'b0;
assign base_ram_oe_n = (accessing_MMIO || is_ext) ? `ReadDisable :
                       (tlb_miss) ? `ReadEnable :
                       (we_n == `WriteEnable) ? `ReadDisable : `ReadEnable;
assign base_ram_we_n = (accessing_MMIO || is_ext) ? `WriteDisable :
                       (tlb_miss) ? `WriteDisable : mem_we_n;

// ext ram r/w flag
assign ext_ram_ce_n = 1'b0;
assign ext_ram_oe_n = (accessing_MMIO || ~is_ext) ? `ReadDisable :
                      (tlb_miss) ? `ReadEnable :
                      (we_n == `WriteEnable) ? `ReadDisable : `ReadEnable;
assign ext_ram_we_n = (accessing_MMIO || ~is_ext) ? `WriteDisable :
                      (tlb_miss) ? `WriteDisable : mem_we_n;

// time
assign time_we = (accessing_time && we_n == `WriteEnable) ? `WriteEnable : `WriteDisable;
assign time_sel = mem_addr == TIMEL_ADDR ? `MMIOTimeL :
    mem_addr == TIMEH_ADDR ? `MMIOTimeH :
    mem_addr == TIMECMPL_ADDR ? `MMIOTimeCmpL :
    mem_addr == TIMECMPH_ADDR ? `MMIOTimeCmpH :
    `MMIOTimeL;
assign time_wdata = data_in;

// byte enable
wire[1:0] addr_mod_4;
assign addr_mod_4 = accessing_MMIO ? (mem_addr % 4) : (physical_addr % 4);
wire[`ByteEnBus] be_n;
assign be_n = read_write_byte ?
                (addr_mod_4 == 2'b00) ? `ByteEnable0 :
                (addr_mod_4 == 2'b01) ? `ByteEnable1 :
                (addr_mod_4 == 2'b10) ? `ByteEnable2 :
                (addr_mod_4 == 2'b11) ? `ByteEnable3 :
                `ByteEnableAll :
              read_write_hw ?
                (addr_mod_4 == 2'b00) ? (`ByteEnable0 & `ByteEnable1) :
                (addr_mod_4 == 2'b10) ? (`ByteEnable2 & `ByteEnable3) :
                `ByteEnableAll :
              `ByteEnableAll;
assign base_ram_be_n = be_n;
assign ext_ram_be_n = be_n;

// bus control
assign base_ram_addr = physical_addr[21:2];
assign ext_ram_addr = physical_addr[21:2];

wire[`DataBus] data_in_buf;
assign data_in_buf = read_write_byte ?
                        (addr_mod_4 == 2'b00) ? {24'h000000, data_in[`Byte]} :
                        (addr_mod_4 == 2'b01) ? {16'h0000, data_in[`Byte], 8'h00} :
                        (addr_mod_4 == 2'b10) ? {8'h00, data_in[`Byte], 16'h00} :
                        (addr_mod_4 == 2'b11) ? {data_in[`Byte], 24'h000000} :
                        data_in :
                     read_write_hw ?
                        (addr_mod_4 == 2'b00) ? {16'h0000, data_in[15:0]} :
                        (addr_mod_4 == 2'b10) ? {data_in[15:0], 16'h0000} :
                        data_in :
                     data_in;

wire data_z;
assign data_z = accessing_uart ?
                    ((we_n == `WriteEnable || uart_state == uart_write_start || uart_state == uart_write_stall || uart_state == uart_write_done) ? 1'b0 : 1'b1) :
                    tlb_miss ? 1'b1: (we_n == `WriteEnable ? 1'b0 : 1'b1);
assign base_ram_data = (data_z || accessing_time) ? 32'bz : data_in_buf;
assign ext_ram_data = (data_z || accessing_MMIO) ? 32'bz : data_in_buf;

wire[`Byte] uart_status;
assign uart_status = {2'b00, uart_state == uart_state_begin ? 1'b1 : 1'b0, 4'b0000, uart_dataready};

wire[`DataBus] ram_data; // select ram data
assign ram_data = (is_ext == `True_v) ? ext_ram_data : base_ram_data;

wire[`DataBus] data_out_buf; // holds the value before sign extension
assign data_out_buf = accessing_uart ? {24'h000000, (mem_addr == UART_STATUS_ADDR) ? uart_status : base_ram_data[`Byte]}: // UART
                      accessing_time ? time_i :
                      read_write_byte ?
                        (addr_mod_4 == 2'b00) ? {24'h000000, ram_data[7:0]} :
                        (addr_mod_4 == 2'b01) ? {24'h000000, ram_data[15:8]} :
                        (addr_mod_4 == 2'b10) ? {24'h000000, ram_data[23:16]} :
                        (addr_mod_4 == 2'b11) ? {24'h000000, ram_data[31:24]} :
                        ram_data :
                      read_write_hw ?
                        (addr_mod_4 == 2'b00) ? {16'h0000, ram_data[15:0]} :
                        (addr_mod_4 == 2'b10) ? {16'h0000, ram_data[31:16]} :
                        ram_data :
                      ram_data;

// sign extend
assign data_out = read_write_signed ?
                    (mem_option == `FUNC3_BYTE && data_out_buf[7] === 1'b1) ? {24'hffffff, data_out_buf[7:0]} :
                    (mem_option == `FUNC3_HALF_WORD && data_out_buf[15] === 1'b1) ? {16'hffff, data_out_buf[15:0]} :
                    data_out_buf :
                  data_out_buf;

always@(posedge clk or posedge rst) begin
    if (rst) begin
        uart_state <= uart_state_begin;
        {uart_rdn, uart_wrn} <= {`ReadDisable, `WriteDisable};
    end
    else begin
        case (uart_state)
            uart_state_begin: begin
                if (mem_addr == UART_DATA_ADDR && read_write_byte) begin // r/w uart
                    if (oe_n == `ReadEnable && uart_dataready) begin
                        uart_rdn <= `ReadEnable;
                        uart_state <= uart_read_done;
                    end else if (we_n == `WriteEnable) begin
                        uart_wrn <= `WriteEnable;
                        uart_state <= uart_write_stall;
                    end
                end
            end
            uart_read_done: begin
                uart_rdn <= `ReadDisable;
                uart_state <= uart_state_begin;
            end
            uart_write_stall: begin
                uart_wrn <= `WriteDisable;
                uart_state <= uart_write_start;
            end
            uart_write_start: begin
                if (uart_tbre)
                    uart_state <= uart_write_done;
            end
            uart_write_done: begin
                if (uart_tsre) begin
                    uart_state <= uart_state_begin;
                end
            end
        endcase
    end
end


/* tlb */
reg[`StateBus] tlb_state;
localparam tlb_state_begin = `StateLen'h0;
localparam tlb_state_fetch = `StateLen'h1;
localparam tlb_state_write = `StateLen'h2;

reg[`TlbEntryBus] tlb_table[0:`TlbEntryNum-1];

wire is_data;
assign is_data = (oe_n == `ReadEnable || we_n == `WriteEnable);

// addr & priv
wire[`DataBus] virtual_addr;
assign virtual_addr = is_data ? mem_addr : pc;
wire[`PrivLvlBus] priv_lvl;
assign priv_lvl = is_data ? mem_priv_lvl : pc_priv_lvl;

// address decoding
wire[`TlbIndexBus] index;
assign index = virtual_addr[`VPN_INDEX];

wire[`TlbEntryBus] entry;
assign entry = tlb_table[index];

wire tlb_entry_found;
assign tlb_entry_found = (entry[`TLBE_VPN] == virtual_addr[`VPN_REST] && entry[`PTE_V]);

wire[`DataBus] tlb_result;
assign tlb_result = {entry[`PTE_PPN], virtual_addr[`PageOffset]};

// physical address assignment
wire need_translate;
assign need_translate = ~(accessing_MMIO || priv_lvl == `M_MODE || satp[`SatpMode] == `SATP_BARE);

assign tlb_miss = (need_translate && ~tlb_entry_found);

reg[`DataBus] lookup_addr; // used for walking the page table
assign physical_addr = need_translate ? 
                        (tlb_miss ? lookup_addr : tlb_result) :
                       virtual_addr;


// control bits
wire[`PTE_REST] control_bits;
assign control_bits = (tlb_state == tlb_state_fetch) ? ram_data[`PTE_REST] : entry[`PTE_REST];

// exceptions generated
assign if_exception_req = (is_data || ~need_translate) ? `False_v :
    (tlb_miss && tlb_state != tlb_state_fetch) ? `False_v :
    (priv_lvl == `U_MODE && ~control_bits[`PTE_U] && ~sum) ? `True_v :
    // (~control_bits[`PTE_A]) ? `True_v :
    (~control_bits[`PTE_X]) ? `True_v :
    `False_v;
assign if_cause = {`ExcException, `ExcInstPageFault};

assign mem_exception_req = (~is_data || ~need_translate) ? `False_v :
    (tlb_miss && tlb_state != tlb_state_fetch) ? `False_v :
    (priv_lvl == `U_MODE && ~control_bits[`PTE_U] && ~sum) ? `True_v :
    // (~control_bits[`PTE_A]) ? `True_v :
    (oe_n && ~control_bits[`PTE_R]) ? `True_v :
    (we_n && ~control_bits[`PTE_W]) ? `True_v :
    // (we_n && ~control_bits[`PTE_D]) ? `True_v :
    `False_v;
assign mem_cause = oe_n ? {`ExcException, `ExcLoadPageFault} : {`ExcException, `ExcStorePageFault};

integer i;
always @ (posedge clk or posedge rst) begin
    if (rst == `RstEnable) begin
        tlb_state <= tlb_state_begin;
        for (i = 0; i < `TlbEntryNum; i = i + 1)
            tlb_table[i] <= 0;
        update_done <= `False_v;
        lookup_addr <= `ZeroWord;
    end else begin
        case (tlb_state)
            tlb_state_begin: begin
                if (update_tlb == `True_v) begin // update all entries
                    update_done <= `True_v;
                    for (i = 0; i < `TlbEntryNum; i = i + 1)
                        tlb_table[i] <= 0;
                end else begin
                    update_done <= `False_v; 
                    if (tlb_entry_found == `False_v && need_translate == `True_v) begin
                        // tlb miss
                        lookup_addr <= {satp[`SatpPPN], virtual_addr[`VPN1], 2'b00};
                        tlb_state <= tlb_state_fetch;
                    end
                end
            end
            tlb_state_fetch: begin // fetch level 2 entry
                update_done <= `False_v;
                lookup_addr <= {ram_data[`PTE_PPN], virtual_addr[`VPN2], 2'b00};
                if (ram_data[`PTE_V])
                    tlb_state <= tlb_state_write;
                else // level 1 entry invalid
                    tlb_state <= tlb_state_begin;
            end
            tlb_state_write: begin // update tlb entry
                update_done <= `False_v;
                tlb_table[index] <= {virtual_addr[`VPN_REST], ram_data};
                tlb_state <= tlb_state_begin;
            end
        endcase
    end
end

endmodule
