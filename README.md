## Thinpad



#### 基本控制信号

| 信号名                | 作用                                                         | 赋值                                                         |
| --------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `alu_a_src_sel`[1:0]  | 00-rdata1, 01-pc, 10-0                                       | 普通数逻运算, load, store为 00; jal , jalr, auipc 为 01; lui 和 **csr操作**为 10 |
| `alu_b_src_sel[1:0]`  | 00-rdata2, 01-4, 10-Imm，**11-csr_data**                     | 普通数逻运算为 00/10，jal 和 jalr 为 01，lui 、load、store 和 auipc 为 10; **csr操作11** |
| `reg_write`           | 1 for write                                                  | 当指令需要写寄存器时为 1 (alu, alu_imm, **csr***); mret, ebreak, ecall 不需要写寄存器 |
| `reg_write_sel`       | 0-from mem, 1-from alu                                       | 普通数逻运算、jal,jalr 为1，load 指令为 0                    |
| `alu_op[?]`           | opcode for alu                                               | jal,jalr,lui,auipc,store为加法，其余按照 func7 和 func3 设置 |
| `branch_flag`         | 0-pc+4, 1-branch_target_addr                                 | 当跳转条件满足时为1（包括 bne,beq 满足条件和遇到 jal,jalr, **mret, ebreak, ecall**） |
| `branch_addr_op[1:0]` | 00-Imm+pc, 01-Imm+rdata1, 10-csr_data                        | beq,bne,jal 时为00， jalr 为 01, **mret，ebreak, ecall 为 10** |
| `mem_read`            | 1 for read                                                   | load                                                         |
| `mem_write`           | 1 for write                                                  | store                                                        |
| `byte_en`             |                                                              | lb, sb...                                                    |
| `imm_type`            | I, J, U ...                                                  | different types of immediates based on instruction.          |
| `csr_we`              |                                                              | csr 指令需要写 csr 寄存器时 (csr*, mret， ecall, ebreak) 为 1 （csrrs, csrrc 在 rs1 为 x0 时不需要写寄存器） |
| `csr_wdata_sel`       | 000-data_rs1, 001-`rs1&csr_rdata`, 010-`~rs1&csr_data`, 011-`mret_mstatus manip`, 100-`mcause data` | csrrw 为 000，csrrs 为 001，csrrc 为 010, mret 为 011, ebreak, ecall 为 100 |
| `mepc_we`             | 1 to update mepc                                             | ecall, ebreak 为 1                                           |
| `csr_sel`             | csr_src_mux: 00-csr_buf, 01-mepc, 10-mtvec                   | csr* 为 00， mret 为 01， ecall, ebreak 为 10                |
|                       | csr_rd_mux: 00-csr_buf, 01-mstatus, 10-mcause                |                                                              |



### 冲突检测信号

| 信号名          | 作用                                                     | 赋值                                                         |
| --------------- | -------------------------------------------------------- | ------------------------------------------------------------ |
| `forward1[1:0]` | 00-rdata1, 01-alu_out, 10-mem_data_out                   | 如果没有数据冲突，那么取 00；否则，判断前传的来源，如果最近的写入来自alu (RAW)，那么取 01，否则 (load-use) 取 10 |
| `forward2[1:0]` | 00-rdata2, 01-alu_out, 10-mem_data_out                   | （续）如果有 EXE 阶段的数据冲突：1）是数逻指令的结果，那么直接前传 2）是 load 指令的结果，那么 stall；如果是 MEM 阶段的数据冲突：数逻指令的结果 或 是 load 指令的结果，都直接前传；如果是 WB 阶段的冲突，那么 reg_file 支持 RAW 即可 |
| `bubble`        | 1 for bubble (to flush the next instruction in IF stage) | branch taken (bne, beq), jalr, jal, **mret, ecall, ebreak**  |
| `stall[0]`      | 1 for stall if, id, ex                                   | load-use (ID_EXE.opcode == load && ID_EXE.rd = rs1 or rs2),**similar for csr* ** |
|                 |                                                          | branch taken, jalr, jal, **mret, ecall, ebreak**             |
|                 |                                                          | memory module not `done`                                     |
|                 |                                                          |                                                              |
| `stall[1]`      | 1 for stall mem                                          | memory module data not ready                                 |
| `forward_csr`   | 0-csr_rdata from reg file, 1-ex_csr_wdata                | 如果 id 阶段读取的 csr 要被在 exe 或 mem 阶段执行的指令写入，那么置 1 |



### memory controller logic

| oe_n | we_n | mem_addr | bus     | mem_oe | mem_we | data_z | data_out           | addr |
| ---- | ---- | -------- | ------- | ------ | ------ | ------ | ------------------ | ---- |
| 1    | 1    | pc       | z       | 0      | 1      | 1      | pc[22] ? ext :base |      |
| 0    | 1    | addr     | z       | 0      | 1      | 1      | addr[22] ? :       | 8-   |
| 1    | 0    | addr     | data_in | 1      | 0      | 0      | -                  | 8-   |
| 0    | 1    | addr     | z       | 1      | 1      | 1      | base               | 1-   |
| 1    | 0    | addr     | data_in | 1      | 1      | 0      | base               | 1-   |
|      |      |          |         |        |        |        |                    |      |
|      |      |          |         |        |        |        |                    |      |
|      |      |          |         |        |        |        |                    |      |
|      |      |          |         |        |        |        |                    |      |





## Exception handler

允许每个stage提出异常/中断请求，由exception handler 统一处理



- mem_controller 可以发出 paging 相关异常，分别给 if 和 mem 阶段，再转发给 exception handler
- id 可以触发异常指令异常
- ex, wb 无异常



csr 寄存器模块单独设置，负责 csr 的读写，与 exception handler 和各个阶段通信