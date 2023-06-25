`default_nettype none
`include "common.vh"
`include "cpu/pipeline/id.vh"

module thinpad_top(
    input wire clk_50M,           //50MHz 时钟输入
    input wire clk_11M0592,       //11.0592MHz 时钟输入（备用，可不用）

    input wire clock_btn,         //BTN5手动时钟按钮�????????关，带消抖电路，按下时为1
    input wire reset_btn,         //BTN6手动复位按钮�????????关，带消抖电路，按下时为1

    input  wire[3:0]  touch_btn,  //BTN1~BTN4，按钮开关，按下时为1
    input  wire[31:0] dip_sw,     //32位拨码开关，拨到“ON”时�????????1
    output wire[15:0] leds,       //16位LED，输出时1点亮
    output wire[7:0]  dpy0,       //数码管低位信号，包括小数点，输出1点亮
    output wire[7:0]  dpy1,       //数码管高位信号，包括小数点，输出1点亮

    //CPLD串口控制器信�????????
    output wire uart_rdn,         //读串口信号，低有�????????
    output wire uart_wrn,         //写串口信号，低有�????????
    input wire uart_dataready,    //串口数据准备�????????
    input wire uart_tbre,         //发�?�数据标�????????
    input wire uart_tsre,         //数据发�?�完毕标�????????

    //BaseRAM信号
    inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共�????????
    output wire[19:0] base_ram_addr, //BaseRAM地址
    output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持�????????0
    output wire base_ram_ce_n,       //BaseRAM片�?�，低有�????????
    output wire base_ram_oe_n,       //BaseRAM读使能，低有�????????
    output wire base_ram_we_n,       //BaseRAM写使能，低有�????????

    //ExtRAM信号
    inout wire[31:0] ext_ram_data,  //ExtRAM数据
    output wire[19:0] ext_ram_addr, //ExtRAM地址
    output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持�????????0
    output wire ext_ram_ce_n,       //ExtRAM片�?�，低有�????????
    output wire ext_ram_oe_n,       //ExtRAM读使能，低有�????????
    output wire ext_ram_we_n,       //ExtRAM写使能，低有�????????

    //直连串口信号
    output wire txd,  //直连串口发�?�端
    input  wire rxd,  //直连串口接收�????????

    //Flash存储器信号，参�?? JS28F640 芯片手册
    output wire [22:0]flash_a,      //Flash地址，a0仅在8bit模式有效�????????16bit模式无意�????????
    inout  wire [15:0]flash_d,      //Flash数据
    output wire flash_rp_n,         //Flash复位信号，低有效
    output wire flash_vpen,         //Flash写保护信号，低电平时不能擦除、烧�????????
    output wire flash_ce_n,         //Flash片�?�信号，低有�????????
    output wire flash_oe_n,         //Flash读使能信号，低有�????????
    output wire flash_we_n,         //Flash写使能信号，低有�????????
    output wire flash_byte_n,       //Flash 8bit模式选择，低有效。在使用flash�????????16位模式时请设�????????1

    //USB 控制器信号，参�?? SL811 芯片手册
    output wire sl811_a0,
    //inout  wire[7:0] sl811_d,     //USB数据线与网络控制器的dm9k_sd[7:0]共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    //网络控制器信号，参�?? DM9000A 芯片手册
    output wire dm9k_cmd,
    inout  wire[15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input  wire dm9k_int,

    //图像输出信号
    output wire[2:0] video_red,    //红色像素�????????3�????????
    output wire[2:0] video_green,  //绿色像素�????????3�????????
    output wire[1:0] video_blue,   //蓝色像素�????????2�????????
    output wire video_hsync,       //行同步（水平同步）信�????????
    output wire video_vsync,       //场同步（垂直同步）信�????????
    output wire video_clk,         //像素时钟输出
    output wire video_de           //行数据有效信号，用于区分消隐�????????
);

/* =========== Demo code begin =========== */

// PLL分频示例
wire locked, clk_pll;
pll_example clock_gen 
 (
  // Clock in ports
  .clk_in1(clk_50M),  // 外部时钟输入
  // Clock out ports
  .clk_out1(clk_pll), // 时钟输出1，频率在IP配置界面中设�????????
//   .clk_out2(clk_20M), // 时钟输出2，频率在IP配置界面中设�????????
  // Status and control signals
  .reset(reset_btn), // PLL复位输入
  .locked(locked)    // PLL锁定指示输出�????????"1"表示时钟稳定�????????
                     // 后级电路复位信号应当由它生成（见下）
 );

reg rst_pll;
always@(posedge clk_pll or negedge locked) begin
    if(~locked) rst_pll <= 1'b1;
    else        rst_pll <= 1'b0;
end

// 数码管连接关系示意图，dpy1同理
// p=dpy0[0] // ---a---
// c=dpy0[1] // |     |
// d=dpy0[2] // f     b
// e=dpy0[3] // |     |
// b=dpy0[4] // ---g---
// a=dpy0[5] // |     |
// f=dpy0[6] // e     c
// g=dpy0[7] // |     |
//           // ---d---  p

// 7段数码管译码器演示，将number�????????16进制显示在数码管上面
wire[7:0] number;
SEG7_LUT segL(.oSEG1(dpy0), .iDIG(number[3:0])); //dpy0是低位数码管
SEG7_LUT segH(.oSEG1(dpy1), .iDIG(number[7:4])); //dpy1是高位数码管

reg[15:0] led_bits;
assign leds = led_bits;

always@(posedge clock_btn or posedge reset_btn) begin
    if(reset_btn)begin //复位按下，设置LED为初始�??
        led_bits <= 16'h1;
    end
    else begin //每次按下时钟按钮，LED循环左移
        led_bits <= {led_bits[14:0],led_bits[15]};
    end
end

//直连串口接收发�?�演示，从直连串口收到的数据再发送出�????????
wire [7:0] ext_uart_rx;
reg  [7:0] ext_uart_buffer, ext_uart_tx;
wire ext_uart_ready, ext_uart_clear, ext_uart_busy;
reg ext_uart_start, ext_uart_avai;
    
assign number = ext_uart_buffer;

async_receiver #(.ClkFrequency(50000000),.Baud(9600)) //接收模块�????????9600无检验位
    ext_uart_r(
        .clk(clk_50M),                       //外部时钟信号
        .RxD(rxd),                           //外部串行信号输入
        .RxD_data_ready(ext_uart_ready),  //数据接收到标�????????
        .RxD_clear(ext_uart_clear),       //清除接收标志
        .RxD_data(ext_uart_rx)             //接收到的�????????字节数据
    );

assign ext_uart_clear = ext_uart_ready; //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer�????????
always @(posedge clk_50M) begin //接收到缓冲区ext_uart_buffer
    if(ext_uart_ready)begin
        ext_uart_buffer <= ext_uart_rx;
        ext_uart_avai <= 1;
    end else if(!ext_uart_busy && ext_uart_avai)begin 
        ext_uart_avai <= 0;
    end
end
always @(posedge clk_50M) begin //将缓冲区ext_uart_buffer发�?�出�????????
    if(!ext_uart_busy && ext_uart_avai)begin 
        ext_uart_tx <= ext_uart_buffer;
        ext_uart_start <= 1;
    end else begin 
        ext_uart_start <= 0;
    end
end

async_transmitter #(.ClkFrequency(50000000),.Baud(9600)) //发�?�模块，9600无检验位
    ext_uart_t(
        .clk(clk_50M),                  //外部时钟信号
        .TxD(txd),                      //串行信号输出
        .TxD_busy(ext_uart_busy),       //发�?�器忙状态指�????????
        .TxD_start(ext_uart_start),    //�????????始发送信�????????
        .TxD_data(ext_uart_tx)        //待发送的数据
    );

//图像输出演示，分辨率800x600@75Hz，像素时钟为50MHz
wire [11:0] hdata;
assign video_red = hdata < 266 ? 3'b111 : 0; //红色竖条
assign video_green = hdata < 532 && hdata >= 266 ? 3'b111 : 0; //绿色竖条
assign video_blue = hdata >= 532 ? 2'b11 : 0; //蓝色竖条
assign video_clk = clk_50M;
vga #(12, 800, 856, 976, 1040, 600, 637, 643, 666, 1, 1) vga800x600at75 (
    .clk(clk_50M), 
    .hdata(hdata), //横坐�????????
    .vdata(),      //纵坐�????????
    .hsync(video_hsync),
    .vsync(video_vsync),
    .data_enable(video_de)
);

/* =========== Demo code end =========== */

wire clk, rst;
// assign clk = clk_50M;
// assign rst = reset_btn;
assign clk = clk_pll;
assign rst = rst_pll;

// group wires by output
/* if */
wire[`DataBus] pc;
wire[`DataBus] if_inst;
wire if_exception_req;
wire update_tlb_req;

/* if_id */
wire[`PrivLvlBus] id_priv_lvl;
wire[`DataBus]  id_pc;
wire[`DataBus]  id_inst;

/* id */
wire[`AluOpBus] alu_op;
wire[`DataBus] alu_a;
wire[`DataBus] alu_b;

wire[`DataBus] data_rs2;
wire[`RegAddrBus] rd;

wire reg_write;
wire reg_write_sel;
wire mem_read;
wire mem_write;
wire[`Func3Bus] mem_option;

// to if
wire branch_flag;
wire[`DataBus] branch_target_addr;

// to ctrl
wire[`OpcodeBus] opcode;
wire[`Func3Bus] func3;
wire[`RegAddrBus] rs1;
wire[`RegAddrBus] rs2;

// to csr reg
wire[`CSRAddrBus] csr;
wire csr_we;
wire[`CSRAddrBus] csr_rd;
wire[`DataBus] csr_wdata;

// exception
wire id_exception_req;
wire[`DataBus] id_cause;

/* id_ex */
wire[`PrivLvlBus] ex_priv_lvl;
wire[`DataBus] ex_pc;
wire[`AluOpBus] ex_alu_op;
wire[`DataBus] ex_alu_a;
wire[`DataBus] ex_alu_b;

wire[`DataBus] ex_data_rs2;
wire[`RegAddrBus] ex_rd;
wire ex_reg_write;
wire ex_reg_write_sel;

wire ex_mem_read;
wire ex_mem_write;
wire[`Func3Bus] ex_mem_option;

wire ex_csr_we;
wire[`CSRAddrBus] ex_csr_rd;
wire[`DataBus] ex_csr_wdata;

/* ex */
wire[`DataBus] alu_out;
wire ex_exception_req;
wire[`DataBus] ex_cause;

/* ex_mem */
wire[`PrivLvlBus] mem_priv_lvl;
wire[`DataBus] mem_pc;
wire[`DataBus] mem_alu_out;
wire[`DataBus] mem_data_rs2;
wire[`RegAddrBus] mem_rd;
wire mem_reg_write;
wire mem_reg_write_sel;

wire mem_mem_read;
wire mem_mem_write;
wire[`Func3Bus] mem_mem_option;

wire mem_csr_we;
wire[`CSRAddrBus] mem_csr_rd;
wire[`DataBus] mem_csr_wdata;

/* mem */
wire[`DataBus] mem_addr;

/* mem_wb */
wire[`DataBus] wb_alu_out;
wire[`DataBus] wb_data_out;
wire[`RegAddrBus] wb_rd;
wire wb_reg_write;
wire wb_reg_write_sel;

/* wb */
wire[`DataBus] wdata;
wire wb_exception_req;
wire[`DataBus] wb_cause;

/* mem controller */
wire busy;
wire[`DataBus] mem_data_out;
wire tlb_update_done;

wire mem_if_exception_req;
wire[`DataBus] if_cause;
wire mem_exception_req;
wire[`DataBus] mem_cause;

wire time_we;
wire[1:0] time_sel;
wire[`DataBus] time_wdata;

/* ctrl */
wire[`ForwardSelBus] forward1;
wire[`ForwardSelBus] forward2;
wire[`ForwardSelBus] csr_forward;
wire[`StallBus] stall;
wire[`StallBus] bubble;
wire update_tlb;

/* csr reg file */
wire[`DataBus] csr_rdata;
wire[`DataBus] satp_rdata;
wire[`DataBus] xstatus_rdata;

wire[`DataBus] xepc;
wire[`DataBus] xstatus;
wire[`DataBus] xie;
wire[`DataBus] xip;
wire[`DataBus] medeleg;
wire[`DataBus] mideleg;
wire[`DataBus] xtvec;

wire[`PrivLvlBus] priv_lvl;
wire[`DataBus] time_;

/* exception handler */
wire exc_flag;
wire[`DataBus] exc_target_addr;
wire[`StageBus] flush_up_to;
wire[`PrivLvlBus] exc_priv_lvl;
wire xepc_we;
wire[`StageBus] pc_sel;
wire xcause_we;
wire[`DataBus] xcause_o;
wire xstatus_we;
wire[`DataBus] xstatus_o;
wire priv_lvl_we;
wire[`PrivLvlBus] priv_lvl_o;


if_ if_0 (
    .clk(clk),
    .rst(rst),
    .stall(stall[`IF]),
    .bubble(bubble[`IF]),

    .exc_flag(exc_flag),
    .exc_target_address(exc_target_addr),
    .branch_flag(branch_flag),
    .branch_target_address(branch_target_addr),

    .inst_i(mem_data_out),
    .pc(pc),
    .inst_o(if_inst),
    
    .exception_req_i(mem_if_exception_req),
    .exception_req_o(if_exception_req),
    .update_tlb_req(update_tlb_req)
);

if_id if_id0 (
    .clk(clk),
    .rst(rst),
    .stall(stall[`ID]),
    .bubble(bubble[`ID]),

    .if_priv_lvl(priv_lvl),
    .if_inst(if_inst),
    .if_pc(pc),
    .id_priv_lvl(id_priv_lvl),
    .id_inst(id_inst),
    .id_pc(id_pc)
);

id id0 (
    .clk(clk),
    .rst(rst),
    // .stall(stall[`ID]),

    .priv_lvl(id_priv_lvl),
    .pc(id_pc),
    .inst(id_inst),

    .ex_alu_out(alu_out),
    .mem_data_out(mem_data_out),
    .mem_alu_out(mem_alu_out),

    .wb_reg_write(wb_reg_write),
    .wb_rd(wb_rd),
    .wb_wdata(wdata),

    .forward1(forward1),
    .forward2(forward2),

    .alu_op(alu_op),
    .alu_a(alu_a),
    .alu_b(alu_b),
    .data_rs2(data_rs2),
    .rd(rd),

    .reg_write(reg_write),
    .reg_write_sel(reg_write_sel),
    .mem_read(mem_read),
    .mem_write(mem_write),
    .mem_option(mem_option),

    .branch_flag(branch_flag),
    .branch_target_addr(branch_target_addr),

    .opcode(opcode),
    .func3(func3),
    .rs1(rs1),
    .rs2(rs2),

    .csr(csr),
    .csr_rdata_buf(csr_rdata),
    .csr_we(csr_we),
    .csr_rd(csr_rd),
    .csr_wdata(csr_wdata),

    .csr_forward(csr_forward),
    .ex_csr_wdata(ex_csr_wdata),
    // .mem_csr_wdata(mem_csr_wdata),

    .exception_req(id_exception_req),
    .cause(id_cause)
);

id_ex id_ex0 (
    .clk(clk),
    .rst(rst),
    .stall(stall[`EXE]),
    .bubble(bubble[`EXE]),

    .id_priv_lvl(id_priv_lvl),
    .id_pc(id_pc),
    .id_alu_op(alu_op),
    .id_alu_a(alu_a),
    .id_alu_b(alu_b),

    .id_data_rs2(data_rs2),
    .id_rd(rd),
    .id_reg_write(reg_write),
    .id_reg_write_sel(reg_write_sel),

    .id_mem_read(mem_read),
    .id_mem_write(mem_write),
    .id_mem_option(mem_option),

    .id_csr_we(csr_we),
    .id_csr_rd(csr_rd),
    .id_csr_wdata(csr_wdata),

    .ex_priv_lvl(ex_priv_lvl),
    .ex_pc(ex_pc),
    .ex_alu_op(ex_alu_op),
    .ex_alu_a(ex_alu_a),
    .ex_alu_b(ex_alu_b),

    .ex_data_rs2(ex_data_rs2),
    .ex_rd(ex_rd),
    .ex_reg_write(ex_reg_write),
    .ex_reg_write_sel(ex_reg_write_sel),

    .ex_mem_read(ex_mem_read),
    .ex_mem_write(ex_mem_write),
    .ex_mem_option(ex_mem_option),

    .ex_csr_we(ex_csr_we),
    .ex_csr_rd(ex_csr_rd),
    .ex_csr_wdata(ex_csr_wdata)
);

ex ex0 (
    .rst(rst),

    .alu_op(ex_alu_op),
    .alu_a(ex_alu_a),
    .alu_b(ex_alu_b),

    .alu_out(alu_out),

    .exception_req(ex_exception_req),
    .cause(ex_cause)
);

ex_mem ex_mem0 (
    .clk(clk),
    .rst(rst),
    .stall(stall[`MEM]),
    .bubble(bubble[`MEM]),

    .ex_priv_lvl(ex_priv_lvl),
    .ex_pc(ex_pc),
    .ex_alu_out(alu_out),
    .ex_data_rs2(ex_data_rs2),
    .ex_rd(ex_rd),
    .ex_reg_write(ex_reg_write),
    .ex_reg_write_sel(ex_reg_write_sel),

    .ex_mem_read(ex_mem_read),
    .ex_mem_write(ex_mem_write),
    .ex_mem_option(ex_mem_option),

    .ex_csr_we(ex_csr_we),
    .ex_csr_rd(ex_csr_rd),
    .ex_csr_wdata(ex_csr_wdata),

    .mem_priv_lvl(mem_priv_lvl),
    .mem_pc(mem_pc),
    .mem_alu_out(mem_alu_out),
    .mem_data_rs2(mem_data_rs2),
    .mem_rd(mem_rd),
    .mem_reg_write(mem_reg_write),
    .mem_reg_write_sel(mem_reg_write_sel),

    .mem_mem_read(mem_mem_read),
    .mem_mem_write(mem_mem_write),
    .mem_mem_option(mem_mem_option),

    .mem_csr_we(mem_csr_we),
    .mem_csr_rd(mem_csr_rd),
    .mem_csr_wdata(mem_csr_wdata)
);

mem mem0 (
    .rst(rst),
    .mem_addr_i(mem_alu_out),
    .mem_addr_o(mem_addr)
);

mem_wb mem_wb0 (
    .clk(clk),
    .rst(rst),
    .stall(stall[`WB]),
    .bubble(bubble[`WB]),

    .mem_alu_out(mem_alu_out),
    .mem_data_out(mem_data_out),
    .mem_rd(mem_rd),
    .mem_reg_write(mem_reg_write),
    .mem_reg_write_sel(mem_reg_write_sel),

    .wb_alu_out(wb_alu_out),
    .wb_data_out(wb_data_out),
    .wb_rd(wb_rd),
    .wb_reg_write(wb_reg_write),
    .wb_reg_write_sel(wb_reg_write_sel)
);

wb wb0 (
    .rst(rst),

    .mem_data_out(wb_data_out),
    .alu_out(wb_alu_out),
    .reg_write_sel(wb_reg_write_sel),

    .wdata(wdata),

    .exception_req(wb_exception_req),
    .cause(wb_cause)
);

mem_controller mem_controller0 (
    .clk(clk),
    .rst(rst),

    .busy(busy),

    .sum(xstatus_rdata[`MSTATUS_SUM]),
    .pc(pc),
    .pc_priv_lvl(priv_lvl),
    .mem_addr(mem_addr),
    .mem_priv_lvl(mem_priv_lvl),

    .mem_option(mem_mem_option),
    .data_in(mem_data_rs2),
    .data_out(mem_data_out),

    .oe_n(mem_mem_read),
    .we_n(mem_mem_write),

    .satp(satp_rdata),

    .update_tlb(update_tlb),
    .update_done(tlb_update_done),

    .if_exception_req(mem_if_exception_req),
    .if_cause(if_cause),
    .mem_exception_req(mem_exception_req),
    .mem_cause(mem_cause),

    .time_we(time_we),
    .time_sel(time_sel),
    .time_wdata(time_wdata),
    .time_i(time_),
    
    .uart_rdn(uart_rdn),
    .uart_wrn(uart_wrn),
    .uart_dataready(uart_dataready),
    .uart_tbre(uart_tbre),
    .uart_tsre(uart_tsre),

    .base_ram_data(base_ram_data),
    .base_ram_addr(base_ram_addr),
    .base_ram_be_n(base_ram_be_n),
    .base_ram_ce_n(base_ram_ce_n),
    .base_ram_oe_n(base_ram_oe_n),
    .base_ram_we_n(base_ram_we_n),

    .ext_ram_data(ext_ram_data),
    .ext_ram_addr(ext_ram_addr),
    .ext_ram_be_n(ext_ram_be_n),
    .ext_ram_ce_n(ext_ram_ce_n),
    .ext_ram_oe_n(ext_ram_oe_n),
    .ext_ram_we_n(ext_ram_we_n)
);

hazard_ctrl hazard_ctrl0 (
    .clk(clk),
    .rst(rst),
    
    .if_update_tlb(update_tlb_req),

    .id_opcode(opcode),
    .id_func3(func3),
    .id_rs1(rs1),
    .id_rs2(rs2),
    .id_branch_flag(branch_flag),

    .ex_reg_write(ex_reg_write),
    .ex_rd(ex_rd),
    .ex_mem_read(ex_mem_read),
    .ex_mem_write(ex_mem_write),

    .mem_reg_write(mem_reg_write),
    .mem_rd(mem_rd),
    .mem_mem_read(mem_mem_read),
    .mem_mem_write(mem_mem_write),

    .mem_controller_busy(busy),
    .tlb_update_done(tlb_update_done),

    .flush_up_to(flush_up_to),

    .id_csr(csr),
    .ex_csr_we(ex_csr_we),
    .ex_csr_rd(ex_csr_rd),
    // .mem_csr_we(mem_csr_we),
    // .mem_csr_rd(mem_csr_rd),
    .csr_forward(csr_forward),

    .forward1(forward1),
    .forward2(forward2),

    .bubble(bubble),
    .stall(stall),

    .update_tlb(update_tlb)
);

csr_reg_file csr_reg_file0 (
    .clk(clk),
    .rst(rst),

    .priv_lvl_cur(priv_lvl), // the current privilege level

    .csr(csr),
    .csr_rdata(csr_rdata),
    .satp_rdata(satp_rdata),
    .xstatus_rdata(xstatus_rdata),

    .csr_we(mem_csr_we),
    .csr_rd(mem_csr_rd),
    .csr_wdata(mem_csr_wdata),

    .exc_priv_lvl(exc_priv_lvl),

    .xepc_we(xepc_we),
    .pc_sel(pc_sel),
    .pc_i_1d({pc, id_pc, ex_pc, mem_pc, `ZeroWord}),
    .xepc_o(xepc),

    .xcause_we(xcause_we),
    .cause_i(xcause_o),

    .xstatus_we(xstatus_we),
    .xstatus_i(xstatus_o),
    .xstatus_o(xstatus),

    .xie_o(xie),
    .xip_o(xip),

    .medeleg_o(medeleg),
    .mideleg_o(mideleg),

    .xtvec_o(xtvec),

    .priv_lvl_we(priv_lvl_we),
    .priv_lvl_i(priv_lvl_o),
    .priv_lvl_o(priv_lvl),

    .time_we(time_we),
    .time_sel(time_sel),
    .time_i(time_wdata),
    .time_o(time_)
);

exception_handler exception_handler0 (
    .rst(rst),
    .clk(clk),

    .priv_lvls_1d({priv_lvl, id_priv_lvl, ex_priv_lvl, mem_priv_lvl, `U_MODE}),
    .stalls(stall),

    .ex_csr_we(ex_csr_we),
    .mem_csr_we(mem_csr_we),

    .exc_req({if_exception_req,id_exception_req,ex_exception_req,mem_exception_req,wb_exception_req}),
    .exc_causes_1d({if_cause,id_cause,ex_cause,mem_cause,wb_cause}),

    .exc_flag(exc_flag),
    .exc_target_address(exc_target_addr),

    .flush_up_to(flush_up_to),

    .exc_priv_lvl(exc_priv_lvl),

    .xepc_we(xepc_we),
    .pc_sel(pc_sel),
    .xepc(xepc),

    .xcause_we(xcause_we),
    .xcause_o(xcause_o),

    .xstatus_we(xstatus_we),
    .xstatus_o(xstatus_o),
    .xstatus_i(xstatus),

    .xip(xip),
    .xie(xie),

    .medeleg(medeleg),
    .mideleg(mideleg),
    .xtvec(xtvec),

    .priv_lvl_we(priv_lvl_we),
    .priv_lvl_o(priv_lvl_o)
);


endmodule
