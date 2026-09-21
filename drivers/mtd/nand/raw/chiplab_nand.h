/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Chiplab (Loongson Artix-7 实验箱) APB NAND 控制器 —— 寄存器/几何/ECC/地址换算核心层
 *
 * 本头文件是**唯一真源**，且**不依赖 U-Boot**（只用 <stdint.h>/<stddef.h>/<string.h>），
 * 因此可以被宿主机单元测试（board/loongson/chiplab/nand-unittest/）直接编译。
 * 一切寄存器偏移、几何常量、ECC 布局、坏块偏移、地址换算、DMA 描述符打包都放这里；
 * U-Boot 侧的 MMIO 驱动（chiplab_nand.c）只做寄存器读写与框架注册。
 *
 * =====================================================================
 * 寄存器映射表（每个用到的控制器寄存器）
 * =====================================================================
 * 控制器基址 0x1FE7_8000（软件侧 [RTL 实测] la32r-Linux/.../ls1a_nand.c:86
 * _NAND_BASE 0x9fe78000）。平台 AXI mux 只译到高 16 位 0x1FE7
 * (chiplab/IP/AMBA/axi_mux_syn.v:856-857)，窗口内偏移由 nand.v 用 ADDR[10:0]
 * 逐 4 字节译码（nand.v:116-129）。RTL 唯一真源 = chiplab/IP/APB_DEV/NAND/nand.v。
 *
 *  偏移  名称               RTL 出处（文件:行）              读写语义
 *  ----  -----------------  -------------------------------  ------------------------------------------
 *  0x00  NAND_CMD           nand.v:116 (HIT0 hit)            W：写 [15:0] 命令字（位域见下）
 *                           nand.v:171-173 (写 nand_command[15:0])
 *                           nand.v:337 (读回 nand_command)
 *                           nand.v:180-182 (高位由硬件回填：
 *                             [31]=NAND_DMA_REQ [30]=0 [29]=0 [28:24]=NAND_STATE
 *                             [23:20]=NAND_CE_o [19:16]=NAND_IORDY_i)
 *  0x04  NAND_ADDRL         nand.v:117 (HIT1)                W：列地址，取 DAT_I[13:0] -> nand_addr_c
 *                           nand.v:184 (nand_addr_c <= DAT_I[13:0])
 *                           nand.v:339 (读回 {20'b0, nand_addr_c})
 *  0x08  NAND_ADDRH         nand.v:118 (HIT2)                W：行地址，取 DAT_I[24:0] -> nand_addr_r
 *                           nand.v:185 (nand_addr_r <= DAT_I[24:0])
 *                           nand.v:341 (读回 {7'b0, nand_addr_r})
 *  0x0C  NAND_TIMING        nand.v:119 (HIT3)                W：时序参数（含下限钳位）
 *                           nand.v:186-189（t[7:0]=max(dat[7:0],5)，t[15:8]=max(dat[15:8],2)）
 *                           nand.v:343 (读回 nand_timing)
 *  0x10  NAND_IDL           nand.v:120 (HIT4)                只读：ID_INFORM[31:0]（器件 ID 低 32 位）
 *                           nand.v:345 (REG_DAT_T = ID_INFORM[31:0])
 *  0x14  NAND_STATUS_IDH    nand.v:121 (HIT5)                只读：{status[7:0], ID_INFORM[47:32]}
 *                           nand.v:347 (REG_DAT_T = {status, ID_INFORM[47:32]})
 *                           status 采样于 nand.v:1216；status[0] = 1 表示**失败/忙**
 *  0x18  NAND_PARAM         nand.v:122 (HIT6)                W：几何/单页容量参数；复位值由 nand_type 决定
 *                           nand.v:190 (nand_parameter <= DAT_I)
 *                           nand.v:159-161（nand_type==2'h2 -> 32'h0800_5000）
 *                           nand.v:349 (读回 nand_parameter)
 *                           nand.v:203-205（位域见下）
 *  0x1C  NAND_OP_NUM        nand.v:123 (HIT7)                W：一次搬运的字节数（nand_op_num）
 *                           nand.v:191 (nand_op_num <= DAT_I)
 *                           nand.v:351 (读回 nand_op_num)
 *  0x20  NAND_CE_MAP0       nand.v:124 (HIT8)                W：CE / RDY 映射
 *                           nand.v:192 (nand_ce_map0 <= DAT_I)
 *                           nand.v:353 (读回)
 *  0x24  NAND_CE_MAP1       nand.v:125 (HIT9)                混合：写=CE 映射（nand.v:193）；
 *                           nand.v:193 / nand.v:194            未写时硬件回填 {READ_MAX_COUNT, NAND_OP_NUM[15:0]}
 *                           nand.v:355 (读回)                  → 只读时是**上一次读取的剩余字节数/实际传输量**
 *  0x28  NAND_RDY_MAP0      nand.v:126 (HIT10)               W：RDY 映射
 *                           nand.v:195 (nand_rdy_map0 <= DAT_I)
 *                           nand.v:357 (读回)
 *  0x2C  NAND_RDY_MAP1      nand.v:127 (HIT11)               混合：写=RDY 映射（nand.v:196）；
 *                           nand.v:196 / nand.v:197            未写时硬件回填 {WRITE_MAX_COUNT, NAND_OP_NUM[15:0]}
 *                           nand.v:359 (读回)
 *  0x40  NAND_DMA_DOORBELL  nand.v:128-129                    **DMA 应答/门铃**：psel 命中且 ADDR[10:0]==11'h40
 *                           nand.v:129 (nand_dma_ack_i)       即拉高 nand_dma_ack_i；
 *                           nand.v:360-361                     读回 NAND_DAT_O_RD（最后一个 32 位字）
 *                           nand.v:100 (nand_clr_ack)
 *
 * 注意：本表与 docs/kb/platform-facts.md §4.2 的 HIT0..HIT11 偏移表一致，但 PF §4.3/U1
 * 当时把逐寄存器语义列为"不确定项"。本节按 nand.v 读写两侧实码关闭 U1（见 README
 * "NAND 驱动"节，以及 docs/porting/04-nand-driver.md 勘误节）。
 *
 * ---------------------------------------------------------------------
 * NAND_CMD(0x00) 命令字位域（写侧；与 RTL 逐位对应）
 * ---------------------------------------------------------------------
 *  [0]  CMD_VALID     启动一次操作（nand.v:461, 555, 564）
 *  [1]  READ          读（nand.v:470, 478, 486, 607）
 *  [2]  WRITE         写/编程（nand.v:494, 501, 508, 515, 610）
 *  [3]  ERASE         擦除（nand.v:520 -> 命令 0x60，nand.v:1250/1259）
 *  [4]  ERASE_SERIAL  连续擦除（nand.v:524 ERASE_SERIAL <= nand_command[4]）
 *  [5]  READ_ID       读 ID（nand.v:526 -> 命令 0x90，nand.v:1332+）
 *  [6]  RESET         复位（nand.v:531 -> 命令 0xFF，nand.v:1176）
 *  [7]  READ_STATUS   读状态（nand.v:536 -> 命令 0x70，nand.v:1199）
 *  [8]  OP_MAIN       主区操作（nand.v:206 main_op = nand_command[8]）
 *  [9]  OP_SPARE      备用区操作（nand.v:207 spare_op = nand_command[9]）
 *  [10] DONE          只读：操作完成（nand.v:635/952/1143... NAND_DONE）
 *  [11] ECC_RD        保留：RTL 未引用 nand_command[11]
 *  [12] ECC_WR        保留：RTL 未引用 nand_command[12]
 *  [13] INT_EN        中断使能（nand.v:208 nand_int_en = nand_command[13]）
 *  [14] RESERVED      保留
 *  [15] RAM_OP        保留：RTL 未引用 nand_command[15]
 *  [16:19] RDY       只读：NAND_IORDY_i（nand.v:180）
 *  [20:23] CE        只读：NAND_CE_o（nand.v:180）
 *  [24:28] STATE     只读：NAND_STATE（nand.v:180）
 *  [31]   DMA_REQ     只读：NAND_DMA_REQ（nand.v:180，属 RTL 行为，不在位域表内）
 *
 * ---------------------------------------------------------------------
 * NAND_PARAM(0x18) 位域（只读推导；nand.v:203-205）
 * ---------------------------------------------------------------------
 *  [15:0]  OP_SCOPE_LO  与 op_scope[13:0] 重叠的一部分（本节只关心 OP_SCOPE）
 *  [29:16] OP_SCOPE     单页容量/搬运上界（nand.v:203）
 *  [14:12] NAND_ID_NUM  读 ID 的字节数（nand.v:204）
 *  [11:8]  NAND_SIZE    几何编码（nand.v:205），见下方 CHIPLAB_NAND_SIZE_*
 *  其余位由软件写入（Linux 侧 ls1a_nand.c:910 写 0x08005300），驱动按需覆盖。
 */

#ifndef __CHIPLAB_NAND_H__
#define __CHIPLAB_NAND_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* --------------------------------------------------------------------- */
/* 1. 控制器寄存器（偏移 / 位域）                                          */
/* --------------------------------------------------------------------- */

#define CHIPLAB_NAND_REG_CMD		0x00u	/* nand.v:116 */
#define CHIPLAB_NAND_REG_ADDRL		0x04u	/* nand.v:117 */
#define CHIPLAB_NAND_REG_ADDRH		0x08u	/* nand.v:118 */
#define CHIPLAB_NAND_REG_TIMING		0x0cu	/* nand.v:119 */
#define CHIPLAB_NAND_REG_IDL		0x10u	/* nand.v:120 */
#define CHIPLAB_NAND_REG_STATUS_IDH	0x14u	/* nand.v:121 */
#define CHIPLAB_NAND_REG_PARAM		0x18u	/* nand.v:122 */
#define CHIPLAB_NAND_REG_OP_NUM		0x1cu	/* nand.v:123 */
#define CHIPLAB_NAND_REG_CE_MAP0	0x20u	/* nand.v:124 */
#define CHIPLAB_NAND_REG_CE_MAP1	0x24u	/* nand.v:125 */
#define CHIPLAB_NAND_REG_RDY_MAP0	0x28u	/* nand.v:126 */
#define CHIPLAB_NAND_REG_RDY_MAP1	0x2cu	/* nand.v:127 */
#define CHIPLAB_NAND_REG_DMA_ACK	0x40u	/* nand.v:128-129 门铃 */

/* 命令字位（写侧，0x00） */
#define CHIPLAB_CMD_VALID		(1u << 0)	/* nand.v:461 */
#define CHIPLAB_CMD_READ		(1u << 1)	/* nand.v:470 */
#define CHIPLAB_CMD_WRITE		(1u << 2)	/* nand.v:494 */
#define CHIPLAB_CMD_ERASE		(1u << 3)	/* nand.v:520 */
#define CHIPLAB_CMD_ERASE_SERIAL	(1u << 4)	/* nand.v:524 */
#define CHIPLAB_CMD_READ_ID		(1u << 5)	/* nand.v:526 */
#define CHIPLAB_CMD_RESET		(1u << 6)	/* nand.v:531 */
#define CHIPLAB_CMD_READ_STATUS		(1u << 7)	/* nand.v:536 */
#define CHIPLAB_CMD_OP_MAIN		(1u << 8)	/* nand.v:206 */
#define CHIPLAB_CMD_OP_SPARE		(1u << 9)	/* nand.v:207 */
#define CHIPLAB_CMD_DONE		(1u << 10)	/* nand.v:635 只读 */
#define CHIPLAB_CMD_ECC_RD		(1u << 11)	/* 保留，RTL 未引用 */
#define CHIPLAB_CMD_ECC_WR		(1u << 12)	/* 保留，RTL 未引用 */
#define CHIPLAB_CMD_INT_EN		(1u << 13)	/* nand.v:208 */
#define CHIPLAB_CMD_RAM_OP		(1u << 15)	/* 保留，RTL 未引用 */
#define CHIPLAB_CMD_DMA_REQ		(1u << 31)	/* nand.v:180 只读 */

/* NAND_PARAM(0x18) 位域 */
#define CHIPLAB_PARAM_OP_SCOPE_SHIFT	16	/* nand.v:203 */
#define CHIPLAB_PARAM_OP_SCOPE_MASK	0x3fffu
#define CHIPLAB_PARAM_ID_NUM_SHIFT	12	/* nand.v:204 */
#define CHIPLAB_PARAM_ID_NUM_MASK	0x7u
#define CHIPLAB_PARAM_SIZE_SHIFT	8	/* nand.v:205 */
#define CHIPLAB_PARAM_SIZE_MASK		0xfu
#define CHIPLAB_PARAM_LO_MASK		0x0000ffffu	/* 软件可覆盖的低 16 位 */

/*
 * nand_size 几何编码（nand.v:225-282 的 case 分支，单向推导）
 *
 * nand.v:228/232/236/240 给出 addr_in_die 的拼装：
 *   {0,nand_addr_r[R:0],4'b0,nand_addr_c[C:0]}
 * 即 **列地址 12 位（主区 2048 B）、行地址低 R+1 位进入 addr_in_die**，
 * 中间的 "4'b0" 是 addr_in_die 的字节内对齐空档。
 *   nand_size==4'h0 -> nand_addr_c[11:0] + nand_addr_r[15:0] -> 3 地址周期
 *   nand_size==4'h1 -> nand_addr_c[11:0] + nand_addr_r[16:0] -> 3 周期
 *   nand_size==4'h2 -> nand_addr_c[11:0] + nand_addr_r[17:0] -> 4 周期
 *   nand_size==4'h3 -> nand_addr_c[11:0] + nand_addr_r[18:0] -> 4 周期
 * 配合 READ_START/WRITE_START 的周期数选择（nand.v:809/982）与 NAND_ADDR_COUNT
 * 各分支的字节抽取（nand.v:733-757）：
 *   3 周期 = [7:0] 列低 + [15:8] 行低 + [23:16] 行高
 *   4 周期 = [7:0] 列低 + [15:8] 列高/行低 + [23:16] + [31:24]
 * 1 Gbit = 1024 块 × 64 页 ⇒ **16 位行地址**（列地址仅需 8 位，因为 2048 = 2^11
 * 只占 3 字节地址的低 11 位，nand.v:743 的 4'h0 分支正是取 NAND_ADDR[15:8] 作第 2 字节）。
 * Linux 侧同口径：ls1a_nand.c:910 把 0x9fe78018 写成 0x08005300
 *   → nand_size = (0x08005300 >> 8) & 0xf = 3，op_scope = 0x08005300 >> 16 = 0x800 = 2048。
 * 复位默认（nand_type=2'h2）为 32'h0800_5000 → nand_size=0、op_scope=2048（nand.v:160）。
 */
#define CHIPLAB_NAND_SIZE_1GBIT		3u	/* 4 地址周期，16 位行地址 */

/* --------------------------------------------------------------------- */
/* 2. 芯片几何（K9F1G08U0C 口径）                                          */
/* --------------------------------------------------------------------- */
/*
 * 口径裁决（docs/porting/04-nand-driver.md §1.2 D1 关闭方式）：
 *   「块 128 KiB = 主区 64 页 × 2048 B，备用区 64 × 64 B 另行」
 * 因此：
 *   - 主区页大小 2048 B，备用区 64 B，**控制器一次搬运 2112 B**；
 *   - 块 = 128 KiB = 主区 64 页（备用区不计入块容量）；
 *   - 芯片容量 128 MiB = 1024 块。
 * 控制器侧口径（RTL 实测，不修改 chiplab NAND 模块）：
 *   - op_scope 复位值 2048（nand.v:160 的 32'h0800_5000 高 16 位）＝主区容量；
 *   - 备用区经 0x40 门铃搬运时不使用 op_scope，另走 nand.v:838-844 的路径。
 *   - 一次「主区+备用区」读写的最大搬运量 = 2048 + 64 = 2112 B；
 *     每次门铃搬运 512 B（nand.v:838-844 以 NAND_ADDR[7:0]==0 时 256 B 为界，
 *     即 2048 B 主区 = 4 个 512 B 段，收尾再搬 64 B 备用区）。
 */
#define CHIPLAB_NAND_PAGE_SIZE		2048u
#define CHIPLAB_NAND_OOBSIZE		64u
#define CHIPLAB_NAND_PAGE_SPARE		(CHIPLAB_NAND_PAGE_SIZE + CHIPLAB_NAND_OOBSIZE) /* 2112 */
#define CHIPLAB_NAND_PAGES_PER_BLOCK	64u
#define CHIPLAB_NAND_BLOCK_SIZE		(CHIPLAB_NAND_PAGE_SIZE * CHIPLAB_NAND_PAGES_PER_BLOCK) /* 131072 */
#define CHIPLAB_NAND_BLOCKS		1024u
#define CHIPLAB_NAND_TOTAL_SIZE		(CHIPLAB_NAND_BLOCK_SIZE * CHIPLAB_NAND_BLOCKS) /* 128 MiB */

/* 器件 ID（[旧项目转述]，待上板用 IDL/STATUS_IDH 实读核对；见 04-nand-driver.md §3.2） */
#define CHIPLAB_NAND_MFR_ID		0xecu	/* Samsung */
#define CHIPLAB_NAND_DEV_ID		0xf1u	/* K9F1G08U0C */
#define CHIPLAB_NAND_ID_BYTES		5u	/* EC F1 00 1D 15（后 3 字节为内部信息） */

/* 寄存器时序值：RTL 复位值 {8'h4, 8'h12} = 0x0412（nand.v:155）；Linux 亦写 0x412（ls1a_nand.c:921） */
#define CHIPLAB_NAND_TIMING_RESET	0x0412u

/* --------------------------------------------------------------------- */
/* 3. ECC 布局（U-Boot / Linux 必须共用，04-nand-driver.md §4.3）           */
/* --------------------------------------------------------------------- */
/*
 * 算法：软件 BCH（GF(2^13)，可纠 4 bit / 512 B 段）
 *   段大小 512 B，每页 4 段，每段 ECC 7 B（52 bit 校验位 + 4 bit 填充），共 28 B。
 * 布局（备用区 64 B）：
 *   偏移 0..1   出厂坏块标记（offset 0; 非 0xFF 即坏块）+ 保留字节（offset 1，恒 0xFF）
 *   偏移 2..35  oobfree[0]（34 B，供 BBT 版本/运行时坏块标记等使用）
 *   偏移 36..63 ECC 字节（4 段 × 7 B，段 j 占 36+7j .. 36+7j+6）
 * 该布局等价于 U-Boot nand_bch_init() 的默认布局（drivers/mtd/nand/raw/nand_bch.c:
 * eccpos 放备用区尾部、oobfree 从 offset 2 开始），因此两侧口径一致。
 */
#define CHIPLAB_NAND_ECC_STEP_SIZE	512u
#define CHIPLAB_NAND_ECC_STEPS		4u	/* 2048 / 512 */
#define CHIPLAB_NAND_ECC_BYTES_PER_STEP	7u	/* ceil(13*4/8) */
#define CHIPLAB_NAND_ECC_TOTAL_BYTES	28u	/* 4 * 7 */
#define CHIPLAB_NAND_ECC_STRENGTH	4u	/* 可纠 4 bit/段 */
#define CHIPLAB_NAND_ECC_FIRST_POS	(CHIPLAB_NAND_OOBSIZE - CHIPLAB_NAND_ECC_TOTAL_BYTES) /* 36 */

/* 坏块标记（04-nand-driver.md §6.2） */
#define CHIPLAB_NAND_BBT_FACTORY_OFF	0u	/* 备用区 offset 0，非 0xFF = 坏块 */
#define CHIPLAB_NAND_BBT_RUNTIME_OFF	1u	/* 备用区 offset 1，运行时坏块标记 */
#define CHIPLAB_NAND_BB_MARKER_BAD	0x00u	/* 出厂标记值 */

/* --------------------------------------------------------------------- */
/* 4. 平台 DMA 引擎（chiplab/IP/DMA/dma.v）                                */
/* --------------------------------------------------------------------- */
/*
 * 平台 DMA 是**描述符驱动**的：CPU 只在 confreg 的 order 寄存器
 * (chiplab/IP/CONFREG/confreg_syn.v:33 `define ORDER_REG_ADDR 16'h1160
 *  → 0x1FD0_1160) 写入描述符物理地址，引擎自行经 AXI 取描述符并搬运。
 *
 * 描述符 = 连续 4 个 64 位字 = 32 B，**必须 32 字节对齐**
 * (dma.v:141 ask_addr = {order_addr_in[31:5], 5'h0}；dma.v:470 同)：
 *   字 0: {[63:32] mem_addr, [31:0] order_addr}
 *         dma.v:559-562 -> dma_order_addr = [31:0]，dma_mem_addr = [63:32]
 *   字 1: {[63:32] length(4B 字数), [31:0] dev_addr}
 *         dma.v:563-566 -> dma_dev_addr = [31:0]，dma_length = [63:32]
 *   字 2: {[63:32] step_times, [31:0] step_length(4B 字数步长)}
 *         dma.v:567-570
 *   字 3: {[63:32] 状态（引擎回填）, [31:0] cmd}
 *         dma.v:571-574 -> dma_r_w = cmd[12]；dma.v:583-584 -> 状态回写 [31:0]
 * 取描述符：AXI 读 4 beat × 64 位（dma.v:471-472 arlen=3/arsize=3）。
 *
 * order 寄存器位（dma.v:135-141）：
 *   [1:0] device_num  设备号
 *   [2]   ask_valid   完成/结束应答（**由 confreg 在 write_dma_end / finish_read_order 清零**，
 *                     confreg_syn.v:326-329）
 *   [3]   dma_start   启动（写 1 触发；confreg 在 finish_read_order 时清零）
 *   [4]   dma_stop
 *   [31:5] 描述符物理地址（低 5 位被忽略）
 *
 * cmd 字位（dma.v:202-203，ls1a_nand.c:164-174 同口径）：
 *   [0] dma_int_mask  [1] dma_int（只读状态）  [7:4] dma_mem_addr 状态位
 *   [12] dma_r_w      0 = 设备→内存（读 NAND），1 = 内存→设备（写 NAND）
 *
 * **数据方向确认**：NAND 控制器把每个 32 位数据字放在门铃寄存器上，由 DMA 引擎
 * 以 dma_ack_out 应答（dma.v:370 `assign dma_ack_out = apb_psel`），
 * 即 NAND 侧恒为「从设备读/写」；dma_r_w 只决定 DDR 侧是读还是写。
 * 因此读 NAND 页 = dma_r_w 0（引擎从 NAND 读、写 DDR），写 NAND 页 = dma_r_w 1。
 */
#define CHIPLAB_DMA_ORDER_REG		0x1fd01160u	/* confreg_syn.v:33 */
#define CHIPLAB_DMA_ORDER_ASK		(1u << 2)	/* dma.v:136 */
#define CHIPLAB_DMA_ORDER_START		(1u << 3)	/* dma.v:137 */
#define CHIPLAB_DMA_ORDER_ALIGN		32u		/* dma.v:141/470 */
#define CHIPLAB_DMA_ORDER_ADDR_MASK	0xffffffe0u	/* 低 5 位清零（32 位掩码） */
#define CHIPLAB_DMA_CMD_RW		(1u << 12)	/* dma.v:572 */
#define CHIPLAB_DMA_NAND_DEV_ADDR	0x1fe78040u	/* nand.v:128-129 门铃 */

/* DMA 描述符（4 × 64 位）在内存中的 32 位半字视图，共 8 个 uint32_t */
#define CHIPLAB_DMA_DESC_WORDS		8u
#define CHIPLAB_DMA_DESC_BYTES		32u

/* --------------------------------------------------------------------- */
/* 5. 几何 / 地址换算                                                     */
/* --------------------------------------------------------------------- */

/*
 * U-Boot 行约定：page = 芯片内页号（0 .. 65535），block = 块号（0 .. 1023），
 * page_in_block = 块内页号（0 .. 63）。
 */
static inline uint32_t chiplab_nand_block_of_page(uint32_t page)
{
	return page / CHIPLAB_NAND_PAGES_PER_BLOCK;
}

static inline uint32_t chiplab_nand_page_in_block(uint32_t page)
{
	return page % CHIPLAB_NAND_PAGES_PER_BLOCK;
}

static inline uint32_t chiplab_nand_page_of_block_start(uint32_t block)
{
	return block * CHIPLAB_NAND_PAGES_PER_BLOCK;
}

/* 列地址（控制器 0x04 寄存器只取低 14 位，nand.v:184） */
static inline uint32_t chiplab_nand_column(uint32_t page_offset)
{
	return page_offset & 0x3fffu;
}

/*
 * 行地址 = 页号（控制器把它放进 addr_in_die 的高位段，nand.v:228）：
 *   addr_in_die[27:12] = nand_addr_r[15:0]（nand_size==4'h0，1 Gbit）
 * 控制器自动把行地址从 nand_addr_r 推进到 NAND_ADDR（nand.v:467）。
 */
static inline uint32_t chiplab_nand_row(uint32_t page)
{
	return page & 0xffffu;
}

/* 控制器 0x18 参数寄存器取值：op_scope=页容量、id_num=ID 字节数、size=几何编码 */
static inline uint32_t chiplab_nand_param_encode(unsigned op_scope, unsigned id_num,
						 unsigned size)
{
	return ((op_scope & CHIPLAB_PARAM_OP_SCOPE_MASK) << CHIPLAB_PARAM_OP_SCOPE_SHIFT) |
	       ((id_num & CHIPLAB_PARAM_ID_NUM_MASK) << CHIPLAB_PARAM_ID_NUM_SHIFT) |
	       ((size & CHIPLAB_PARAM_SIZE_MASK) << CHIPLAB_PARAM_SIZE_SHIFT);
}

static inline unsigned chiplab_nand_param_op_scope(uint32_t param)
{
	return (param >> CHIPLAB_PARAM_OP_SCOPE_SHIFT) & CHIPLAB_PARAM_OP_SCOPE_MASK;
}

static inline unsigned chiplab_nand_param_id_num(uint32_t param)
{
	return (param >> CHIPLAB_PARAM_ID_NUM_SHIFT) & CHIPLAB_PARAM_ID_NUM_MASK;
}

static inline unsigned chiplab_nand_param_size(uint32_t param)
{
	return (param >> CHIPLAB_PARAM_SIZE_SHIFT) & CHIPLAB_PARAM_SIZE_MASK;
}

/* --------------------------------------------------------------------- */
/* 6. BCH(8191, 8139, t=4) 编解码 —— 自held，GF(2^13)                       */
/* --------------------------------------------------------------------- */
/*
 * 与 Linux / U-Boot 的通用 BCH 库**不共用位序**：本实现把数据位按「字节内低位在先」
 * 铺进码字高位（见 ecc bit packing），U-Boot 侧与将来 Linux 侧必须用同一份实现
 * （04-nand-driver.md §4.3 的"两侧共用单一头文件"口径）。位序一旦改动，
 * 已写入的数据将无法纠正 —— 单元测试的 test_bit_order 锁死该约定。
 *
 * 码字：n = 2^13 - 1 = 8191 bit，其中信息 = 512 B = 4096 bit，
 * 校验 = 52 bit（存 7 B，高 4 bit 填充 0）。生成多项式由 α^1..α^8 的极小多项式乘积
 * 在运行时构造（避免硬编码出错），并用 g(α^i)=0 (i=1..8) 自检。
 */

/*
 * 纠错能力上限（每 512 B 段可纠 bit 数）。**这是唯一的能力开关**：
 *   - U-Boot 侧由 CONFIG_NAND_CHIPLAB_ECC_STRENGTH 覆盖（Kconfig range 1..4）；
 *   - 宿主机单元测试可用 -DCHIPLAB_ECC_STRENGTH_DEFAULT=n 覆盖，
 *     用于「把能力调小后 >n bit 注入错误必须 FAIL」的反证实验。
 */
#ifndef CHIPLAB_ECC_STRENGTH_DEFAULT
#define CHIPLAB_ECC_STRENGTH_DEFAULT	4
#endif

/*
 * 解码侧能力上限（用于反证实验）。默认等于编码侧的强度，正常工作下
 * 两者必须一致；单元测试用 -DCHIPLAB_ECC_DECODE_STRENGTH=3 把它调小，
 * 模拟"BCH-4 被当成 BCH-3 用"，此时注入 4 bit 错必须失败 —— 若仍报成功，
 * 说明测试根本没有观测到纠正能力上限。
 */
#ifndef CHIPLAB_ECC_DECODE_STRENGTH
#define CHIPLAB_ECC_DECODE_STRENGTH	CHIPLAB_ECC_STRENGTH_DEFAULT
#endif

#define CHIPLAB_BCH_M		13
#define CHIPLAB_BCH_N		((1u << CHIPLAB_BCH_M) - 1u)	/* 8191 */
#define CHIPLAB_BCH_T		CHIPLAB_ECC_STRENGTH_DEFAULT
#define CHIPLAB_BCH_ECC_BITS	(CHIPLAB_BCH_M * CHIPLAB_BCH_T)	/* 52 */
#define CHIPLAB_BCH_PRIM_POLY	0x201bu	/* GF(2^13) 标准本原多项式 x^13+x^4+x^3+x+1 */

/* GF(2^13) 两级表 + 生成多项式（每个 ecc 上下文一份，栈上 / 静态各一）
 * a_log 按字段值直接索引（字段值 < 2^13），故需 2^13 项。
 */
struct chiplab_bch {
	uint16_t a_log[1u << CHIPLAB_BCH_M];	/* [x] = log_α(x)，x < 8192 */
	uint16_t a_pow[CHIPLAB_BCH_N * 2];	/* [i] = α^(i mod 8191) */
	uint16_t g[CHIPLAB_BCH_ECC_BITS + 1];	/* degree ecc_bits，g[ecc_bits]=1 */
};

static inline uint16_t chiplab_gf_pow(const struct chiplab_bch *b, unsigned i)
{
	return b->a_pow[i % CHIPLAB_BCH_N];
}

static inline int chiplab_gf_log(const struct chiplab_bch *b, uint16_t v)
{
	return (int)b->a_log[v & CHIPLAB_BCH_N];
}

static inline uint16_t chiplab_gf_mul(const struct chiplab_bch *b, uint16_t x, uint16_t y)
{
	int lx, ly;

	if (x == 0 || y == 0)
		return 0;
	lx = chiplab_gf_log(b, x);
	ly = chiplab_gf_log(b, y);
	return chiplab_gf_pow(b, (unsigned)(lx + ly));
}

/*
 * 多项式求值：coeff[0..n] **升幂**（coeff[k] 是 x^k 的系数，均存为 GF(2^13) 元素）。
 * 与 struct chiplab_bch.g[] / chiplab_bch_minimal_poly() 的输出约定一致。
 */
static inline uint16_t chiplab_bch_poly_eval(const struct chiplab_bch *b,
					     const uint16_t *coeff, unsigned n, uint16_t x)
{
	uint16_t acc = 0;
	int i;

	for (i = (int)n; i >= 0; i--)
		acc = (uint16_t)(chiplab_gf_mul(b, acc, x) ^ coeff[i]);
	return acc;
}

/*
 * 求 α^i 的极小多项式（GF(2) 系数）。
 * 输出 out[0..deg] **升幂**（out[k] = x^k 系数，out[deg] 恒为 1），return deg。
 * 调用者须提供 ≥ M+2 个元素的缓冲区（out[0..M+1]）。
 * 自检：out[deg]==1 且 out(α^i)==0，否则返回负值（fail-closed）。
 */
static int chiplab_bch_minimal_poly(const struct chiplab_bch *b, unsigned i,
				    uint16_t *out)
{
	uint16_t conj[CHIPLAB_BCH_M];
	uint16_t tmp[CHIPLAB_BCH_M + 2];
	uint16_t poly[CHIPLAB_BCH_M + 2];
	unsigned nconj = 0, deg, j, k, e, x0;

	/*
	 * 迹共轭集合 = {α^i, α^{2i}, α^{4i}, ...}，**包含 α^i 自身**：
	 * 先取 x = α^i，再反复平方（GF(2) 上特征为 2，故 α^{2i} = (α^i)^2）
	 * 直到回到集合内元素为止。
	 */
	e = i % CHIPLAB_BCH_N;
	x0 = (e == 0) ? 1u : b->a_pow[e];
	for (j = 0; j < CHIPLAB_BCH_M; j++) {
		unsigned x;
		int dup = 0;

		x = (j == 0) ? x0 : ((e == 0) ? 1u : b->a_pow[e]);
		for (k = 0; k < nconj; k++)
			if (conj[k] == (uint16_t)x)
				dup = 1;
		if (dup)
			break;
		conj[nconj++] = (uint16_t)x;
		e = (e * 2u) % CHIPLAB_BCH_N;
	}
	/* poly = 1，随后逐个乘 (x + α^{2^j})；系数下标 k = x^k 系数 */
	for (j = 0; j < CHIPLAB_BCH_M + 2u; j++)
		poly[j] = 0;
	poly[0] = 1;
	deg = 0;
	for (k = 0; k < nconj; k++) {
		unsigned jj2;

		if (conj[k] == 0)
			continue;
		for (jj2 = 0; jj2 <= deg + 1u; jj2++)
			tmp[jj2] = 0;
		for (jj2 = 0; jj2 <= deg; jj2++) {
			tmp[jj2] ^= chiplab_gf_mul(b, poly[jj2], conj[k]);
			tmp[jj2 + 1] ^= poly[jj2];
		}
		deg++;
		/* 必须拷到 deg（含新首项），否则会丢掉最高次系数 */
		for (jj2 = 0; jj2 <= deg; jj2++)
			poly[jj2] = tmp[jj2];
	}
	for (j = 0; j < CHIPLAB_BCH_M + 2u; j++)
		out[j] = 0;
	for (j = 0; j <= deg; j++)
		out[j] = poly[j];
	/* 自检：首项为 1 且以 α^i 为根 */
	if (deg == 0 || out[deg] != 1)
		return -1;
	if (chiplab_bch_poly_eval(b, out, deg, b->a_pow[i % CHIPLAB_BCH_N]) != 0)
		return -2;
	return (int)deg;
}

/* 由 α^1..α^{2t} 的极小多项式之积构造生成多项式 g(x)（deg = ecc_bits） */
static int chiplab_bch_init(struct chiplab_bch *b)
{
	uint16_t minp[CHIPLAB_BCH_M + 2];
	/* 卷积中间结果次数可达 deg+md（上限 ecc_bits+M），故按 M 加宽缓冲区 */
	uint16_t next[CHIPLAB_BCH_ECC_BITS + CHIPLAB_BCH_M + 2];
	uint16_t dist[2 * CHIPLAB_BCH_T + 1][CHIPLAB_BCH_M + 2];
	unsigned dist_deg[2 * CHIPLAB_BCH_T + 1];
	unsigned ndist = 0;
	unsigned deg, i, j, k;
	uint32_t x = 1;

	for (i = 0; i < CHIPLAB_BCH_N; i++) {
		b->a_pow[i] = (uint16_t)x;
		b->a_log[x] = (uint16_t)i;
		x <<= 1;
		if (x & (1u << CHIPLAB_BCH_M))
			x ^= CHIPLAB_BCH_PRIM_POLY;
	}
	for (i = CHIPLAB_BCH_N; i < CHIPLAB_BCH_N * 2; i++)
		b->a_pow[i] = b->a_pow[i - CHIPLAB_BCH_N];
	b->a_log[0] = 0;	/* 未定义，禁止使用 */

	/* g = 1 */
	for (j = 0; j <= CHIPLAB_BCH_ECC_BITS; j++)
		b->g[j] = 0;	/* 含 j == ecc_bits：置 0，避免读到未初始化值 */
	b->g[0] = 1;
	deg = 0;
	for (i = 1; i <= 2 * CHIPLAB_BCH_T; i++) {
		int md = chiplab_bch_minimal_poly(b, i, minp);

		if (md <= 0)
			return -1;
		/* 同一极小多项式可能被多个 α^i 共用（如 α、α²、α⁴、α⁸），
		 * 重复乘入会得到 g^2，次数翻倍 —— 必须按多项式去重。 */
		if (md > (int)CHIPLAB_BCH_M + 1)
			return -1;
		for (k = 0; k < ndist; k++) {
			int same = (dist_deg[k] == (unsigned)md);

			for (j = 0; same && j <= (unsigned)md; j++)
				if (dist[k][j] != minp[j])
					same = 0;
			if (same)
				break;
		}
		if (k < ndist)
			continue;	/* 本幂次的极小多项式已并入 g */
		if (ndist >= 2 * CHIPLAB_BCH_T + 1u)
			return -1;
		for (j = 0; j <= (unsigned)md; j++)
			dist[ndist][j] = minp[j];
		for (j = (unsigned)md + 1u; j < CHIPLAB_BCH_M + 2u; j++)
			dist[ndist][j] = 0;
		dist_deg[ndist] = (unsigned)md;
		ndist++;

		if (deg + (unsigned)md > CHIPLAB_BCH_ECC_BITS)
			return -1;	/* 次数超界：参数不匹配，fail-closed */
		/*
		 * next = g * minp（稀疏卷积，只用到 minp[0..md]，其中 minp[md]==1）。
		 * 写入范围恒为 j+md ≤ deg+md ≤ ecc_bits（上方已校验），不会越界。
		 */
		for (j = 0; j < CHIPLAB_BCH_ECC_BITS + CHIPLAB_BCH_M + 2u; j++)
			next[j] = 0;
		for (j = 0; j <= deg; j++) {
			unsigned k2;

			for (k2 = 0; k2 <= (unsigned)md; k2++)
				next[j + k2] ^= chiplab_gf_mul(b, b->g[j], minp[k2]);
		}
		deg += (unsigned)md;
		for (j = 0; j <= deg; j++)
			b->g[j] = next[j];
		for (j = deg + 1u; j <= CHIPLAB_BCH_ECC_BITS; j++)
			b->g[j] = 0;
	}
	if (ndist != CHIPLAB_BCH_T)
		return -1;	/* BCH(2t) 的 g 必须由 t 个互异极小多项式构成 */
	if (deg != CHIPLAB_BCH_ECC_BITS)
		return -1;
	if (b->g[CHIPLAB_BCH_ECC_BITS] != 1)
		return -1;
	/* 自检：g(α^i) 必须为 0，i = 1..2t */
	for (i = 1; i <= 2 * CHIPLAB_BCH_T; i++)
		if (chiplab_bch_poly_eval(b, b->g, CHIPLAB_BCH_ECC_BITS,
					  b->a_pow[i]) != 0)
			return -2;
	return 0;
}

/*
 * ---- BCH 编解码实现 ----
 *
 * 位序约定（**改动即破坏已写入数据**，单元测试 test_bit_order 锁死）：
 *   data byte i 的 bit j ↔ 码字位 pos = 8*i + j        （pos 即多项式幂次）
 *   ECC  byte e 的 bit j ↔ 码字位 pos = 8*nbytes + 8*e + j，仅前 52 位有效
 *   （ecc_bytes = 7，前 52 位用满，后 4 位固定为 0）
 *
 * 编码：V(x) = M(x)·x^52 + R(x)，R(x) = M(x)·x^52 mod g(x)
 * 解码：S_i = V(α^i)（i = 1..2t）应全为 0；非零则 Berlekamp-Massey 求
 *       错误定位多项式 Λ(x)，Chien 搜索定位错误位并翻转。
 *
 * 原语：
 */

/* 翻转码字位（p < 8·nbytes 落数据，否则落 ECC 的 x^(p-dbase) 系数） */
static void chiplab_bch_cw_flip(uint8_t *data, unsigned nbytes,
				uint8_t *ecc, unsigned p)
{
	unsigned dbase = nbytes * 8u;

	if (p < dbase) {
		data[p >> 3] ^= (uint8_t)(1u << (p & 7u));
		return;
	}
	p -= dbase;
	if (p < CHIPLAB_BCH_ECC_BITS)
		ecc[p >> 3] ^= (uint8_t)(1u << (p & 7u));
}

/* 2t 个 syndrome：S_i = V(α^i)，i = 1..2t（按上面的唯一定义） */
static void chiplab_bch_syndromes(const struct chiplab_bch *b,
				  const uint8_t *data, unsigned nbytes,
				  const uint8_t *ecc, unsigned ecc_bytes,
				  uint16_t *s)
{
	unsigned dbase = nbytes * 8u;
	unsigned i, p, k;

	if (nbytes == 0 || dbase + CHIPLAB_BCH_ECC_BITS > CHIPLAB_BCH_N ||
	    ecc_bytes * 8u < CHIPLAB_BCH_ECC_BITS) {
		for (i = 0; i < 2 * CHIPLAB_BCH_T; i++)
			s[i] = 0;
		return;
	}

	for (i = 0; i < 2 * CHIPLAB_BCH_T; i++) {
		uint16_t acc = 0;

		for (p = 0; p < dbase; p++) {
			unsigned bit = (data[p >> 3] >> (p & 7u)) & 1u;

			if (bit)
				acc ^= chiplab_gf_pow(b, ((i + 1u) * p) % CHIPLAB_BCH_N);
		}
		for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++) {
			unsigned bit = (ecc[k >> 3] >> (k & 7u)) & 1u;

			if (bit)
				acc ^= chiplab_gf_pow(b, ((i + 1u) * (dbase + k)) % CHIPLAB_BCH_N);
		}
		s[i] = acc;
	}
}

/*
 * 计算 ECC：解 2·ability 个方程 V(α^i) = 0（i = 1..2·ability）得到 52 个
 * ECC 位。ability 是**纠错能力上限的唯一开关**：调小它会让 ECC 只满足更弱
 * 的根条件，于是注入 >ability 位错误时解码必须报失败（反证实验用）。
 */
/*
 * 计算 ECC。位序约定（与 chiplab_bch_syndromes 完全一致）：
 *   V(x) = Σ_{p=0}^{8n-1} b_p·x^p + Σ_{k=0}^{51} E_k·x^(8n+k)
 * 编码即求解 V(α^s) = 0（s = 1..2·ability）得到 52 个 ECC 位。
 *
 * 实现：把每个 syndrome 方程按 GF(2^13) 拆成 13 个 GF(2) 位方程，
 * 得到 13·2·ability 个位方程、52 个未知位，用 GF(2) RREF 求解。
 *   - ability == t（默认）→ 完整的 BCH-t 码字，能纠 t 位；
 *   - ability <  t → **只满足更弱的根条件**的码字。这不只是"能力更小"，
 *     它同时被用作反证实验的驱动器：把能力调小后，注入超过该能力的错误
 *     必须解码失败（fail-closed），否则说明"能报错"只是运气。
 */
static int chiplab_bch_encode_t(const struct chiplab_bch *b,
				const uint8_t *data, unsigned nbytes,
				uint8_t *ecc, unsigned ecc_bytes,
				unsigned ability)
{
	uint32_t rows[CHIPLAB_BCH_M * 2 * CHIPLAB_BCH_T][2];
	unsigned dbase, neq, nrow, i, j, k, r, piv, row;

	if (nbytes == 0 || nbytes * 8u + CHIPLAB_BCH_ECC_BITS > CHIPLAB_BCH_N)
		return -1;
	if (ability == 0 || ability > CHIPLAB_BCH_T)
		return -1;
	if (ecc_bytes < (CHIPLAB_BCH_ECC_BITS + 7u) / 8u)
		return -1;

	dbase = nbytes * 8u;
	neq = 2u * ability;
	nrow = 0;

	for (i = 0; i < neq; i++) {
		uint16_t cc = 0, coef[CHIPLAB_BCH_ECC_BITS];
		unsigned s = i + 1u, p, bit;

		for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++)
			coef[k] = chiplab_gf_pow(b, (s * (dbase + k)) % CHIPLAB_BCH_N);
		for (p = 0; p < dbase; p++) {
			if ((data[p >> 3] >> (p & 7u)) & 1u)
				cc ^= chiplab_gf_pow(b, (s * p) % CHIPLAB_BCH_N);
		}

		for (bit = 0; bit < CHIPLAB_BCH_M; bit++) {
			uint32_t lo = 0, hi = 0;

			for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++) {
				if (!((coef[k] >> bit) & 1u))
					continue;
				if (k < 32)
					lo |= 1u << k;
				else
					hi |= 1u << (k - 32);
			}
			if ((cc >> bit) & 1u)
				hi |= 1u << (CHIPLAB_BCH_ECC_BITS - 32);	/* RHS 位 */
			rows[nrow][0] = lo;
			rows[nrow][1] = hi;
			nrow++;
		}
	}

	r = 0;
	for (k = 0; k < CHIPLAB_BCH_ECC_BITS && r < nrow; k++) {
		uint32_t rlo, rhi;

		row = nrow;
		for (piv = r; piv < nrow; piv++) {
			uint32_t bv = (k < 32) ? (rows[piv][0] >> k)
					       : (rows[piv][1] >> (k - 32));

			if (bv & 1u) {
				row = piv;
				break;
			}
		}
		if (row == nrow)
			continue;	/* 自由列 */
		{
			uint32_t t0 = rows[r][0], t1 = rows[r][1];

			rows[r][0] = rows[row][0];
			rows[r][1] = rows[row][1];
			rows[row][0] = t0;
			rows[row][1] = t1;
		}
		rlo = rows[r][0];
		rhi = rows[r][1];
		for (piv = 0; piv < nrow; piv++) {
			uint32_t bv;

			if (piv == r)
				continue;
			bv = (k < 32) ? (rows[piv][0] >> k) : (rows[piv][1] >> (k - 32));
			if (bv & 1u) {
				rows[piv][0] ^= rlo;
				rows[piv][1] ^= rhi;
			}
		}
		r++;
	}

	for (i = 0; i < ecc_bytes; i++)
		ecc[i] = 0;
	/* 唯一解：每个未知位由 RREF 中该列的单行给出 */
	for (i = 0; i < r; i++) {
		uint32_t lo = rows[i][0], hi = rows[i][1];
		unsigned rhs = (hi >> (CHIPLAB_BCH_ECC_BITS - 32)) & 1u;

		if (!rhs)
			continue;
		for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++) {
			uint32_t bv = (k < 32) ? (lo >> k) : (hi >> (k - 32));

			if (bv & 1u) {
				ecc[k >> 3] |= (uint8_t)(1u << (k & 7u));
				break;
			}
		}
	}
	(void)j;
	return 0;
}

static int chiplab_bch_encode(const struct chiplab_bch *b,
			      const uint8_t *data, unsigned nbytes,
			      uint8_t *ecc, unsigned ecc_bytes)
{
	return chiplab_bch_encode_t(b, data, nbytes, ecc, ecc_bytes,
				    CHIPLAB_BCH_T);
}

/*
 * 解码：返回纠正的 bit 数（0 = 无错）；负数 = 失败。
 * **fail-closed**：超出纠正能力一律返回负值，绝不静默返回 0；
 * 修正后仍有余 syndrome 时回滚原始数据后返回负值。
 */
static int chiplab_bch_decode(const struct chiplab_bch *b,
			      uint8_t *data, unsigned nbytes,
			      uint8_t *ecc, unsigned ecc_bytes)
{
	uint16_t s[2 * CHIPLAB_BCH_T];
	uint16_t C[CHIPLAB_BCH_T + 1], B[CHIPLAB_BCH_T + 1];
	uint16_t T[CHIPLAB_BCH_T + 1];
	int L = 0, m = 1, nerr = 0;
	unsigned i, j, nz = 0, nbits;
	uint16_t bloc = 1;
	unsigned errpos[CHIPLAB_BCH_T];

	if (nbytes == 0 || nbytes * 8u + CHIPLAB_BCH_ECC_BITS > CHIPLAB_BCH_N)
		return -1;
	if (ecc_bytes < (CHIPLAB_BCH_ECC_BITS + 7u) / 8u)
		return -1;

	nbits = nbytes * 8u + CHIPLAB_BCH_ECC_BITS;
	/*
	 * 参与判定的 syndrome 对数 = 解码侧能力上限（默认 = t = 4）。
	 * 调小它（反证实验）会放宽根条件：既可能把超限错误当"无错"放过，
	 * 也会让 BM 找不到正确定位多项式 —— 两种情况测试都必须变红。
	 */
	if (CHIPLAB_ECC_DECODE_STRENGTH == 0 ||
	    CHIPLAB_ECC_DECODE_STRENGTH > CHIPLAB_BCH_T)
		return -1;
	chiplab_bch_syndromes(b, data, nbytes, ecc, ecc_bytes, s);
	for (i = 0; i < 2u * CHIPLAB_ECC_DECODE_STRENGTH; i++)
		if (s[i])
			nz++;
	if (nz == 0)
		return 0;	/* 无错 */

	/*
	 * Berlekamp-Massey（标准形式，S 索引 0..2t-1 对应 S_1..S_{2t}）：
	 *   d = S[n] ^ Σ_{i=1..L} C[i]·S[n-i]
	 *   C(x) ← C(x) - (d/b)·x^m·B(x)
	 * 收敛后 C 即错误定位多项式 Λ(x)（C[0]=1），其根为 α^{-p}。
	 */
	for (i = 0; i <= CHIPLAB_BCH_T; i++)
		C[i] = B[i] = 0;
	C[0] = 1;
	B[0] = 1;

	for (i = 0; i < 2u * CHIPLAB_ECC_DECODE_STRENGTH; i++) {
		uint16_t d = 0;
		uint16_t coef;

		for (j = 0; j <= (unsigned)L; j++)
			d ^= chiplab_gf_mul(b, C[j], s[i - j]);

		if (d == 0) {
			m++;
			continue;
		}
		for (j = 0; j <= CHIPLAB_BCH_T; j++)
			T[j] = C[j];
		coef = chiplab_gf_mul(b, d,
				     chiplab_gf_pow(b, CHIPLAB_BCH_N - (unsigned)chiplab_gf_log(b, bloc)));
		for (j = 0; j + (unsigned)m <= CHIPLAB_BCH_T; j++)
			C[j + (unsigned)m] ^= chiplab_gf_mul(b, coef, B[j]);
		if (2 * L <= (int)i) {
			unsigned oldL = (unsigned)L;

			for (j = 0; j <= CHIPLAB_BCH_T; j++)
				B[j] = T[j];
			L = (int)i + 1 - (int)oldL;
			bloc = d;
			m = 1;
		} else {
			m++;
		}
	}
	if (L > CHIPLAB_BCH_T)
		return -(L);

	/* Chien 搜索：位 p 出错 ⇔ Λ(α^{-p}) = 0（α^{-p} = α^{n-p}） */
	for (i = 0; i < nbits; i++) {
		uint16_t v = 0;

		for (j = 0; j <= (unsigned)L; j++)
			v ^= chiplab_gf_mul(b, C[j],
					    chiplab_gf_pow(b, j * ((unsigned)CHIPLAB_BCH_N - i)));
		if (v == 0) {
			if (nerr >= CHIPLAB_BCH_T)
				return -(CHIPLAB_BCH_T + 1);
			errpos[nerr++] = i;
		}
	}
	if (nerr != L)
		return -(CHIPLAB_BCH_T + 1);

	for (i = 0; i < (unsigned)nerr; i++)
		chiplab_bch_cw_flip(data, nbytes, ecc, errpos[i]);

	/* 修正后必须全 syndrome 归零；否则回滚并判为不可纠正 */
	chiplab_bch_syndromes(b, data, nbytes, ecc, ecc_bytes, s);
	for (i = 0; i < 2u * CHIPLAB_ECC_DECODE_STRENGTH; i++) {
		if (s[i]) {
			for (j = 0; j < (unsigned)nerr; j++)
				chiplab_bch_cw_flip(data, nbytes, ecc, errpos[j]);
			return -(CHIPLAB_BCH_T + 1);
		}
	}
	return nerr;
}

/* --------------------------------------------------------------------- */
/* 7. 坏块表（BBT）逻辑                                                   */
/* --------------------------------------------------------------------- */
/*
 * 口径（docs/porting/04-nand-driver.md §6.2）：
 *   - 出厂坏块标记：备用区 offset 0，**非 0xFF 即坏块**；
 *   - 运行时坏块标记：备用区 offset 1，同样非 0xFF 即坏块；
 *   - 两者都不占用 ECC 区（ECC 在备用区 offset 36..63，见 §3）。
 */
static inline int chiplab_nand_bb_marker_isbad(uint8_t v)
{
	return v != 0xffu;
}

/* 判定一页备用区是否标记为坏块（检出出厂或运行时标记） */
static inline int chiplab_nand_oob_isbad(const uint8_t *oob, unsigned oob_size)
{
	if (oob_size <= CHIPLAB_NAND_BBT_RUNTIME_OFF)
		return 1;	/* 备用区过小：无法判定 ⇒ 保守判坏（fail-closed） */

	return chiplab_nand_bb_marker_isbad(oob[CHIPLAB_NAND_BBT_FACTORY_OFF]) ||
	       chiplab_nand_bb_marker_isbad(oob[CHIPLAB_NAND_BBT_RUNTIME_OFF]);
}

/* 在备用区写入坏块标记（两个偏移都写 BAD 值） */
static inline int chiplab_nand_oob_markbad(uint8_t *oob, unsigned oob_size)
{
	if (oob_size <= CHIPLAB_NAND_BBT_RUNTIME_OFF)
		return -1;
	oob[CHIPLAB_NAND_BBT_FACTORY_OFF] = CHIPLAB_NAND_BB_MARKER_BAD;
	oob[CHIPLAB_NAND_BBT_RUNTIME_OFF] = CHIPLAB_NAND_BB_MARKER_BAD;
	return 0;
}

/* BBT 覆盖的块数 → BBT 自身需要的字节数（每块 2 bit：出厂/运行时，1 = 坏） */
static inline unsigned chiplab_nand_bbt_bytes(unsigned blocks)
{
	return (blocks * 2u + 7u) / 8u;
}

static inline void chiplab_nand_bbt_set(uint8_t *bbt, unsigned block, unsigned which,
					int bad)
{
	unsigned bit = block * 2u + which;

	if (bad)
		bbt[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
	else
		bbt[bit >> 3] &= (uint8_t)~(1u << (bit & 7u));
}

static inline int chiplab_nand_bbt_get(const uint8_t *bbt, unsigned block,
				       unsigned which)
{
	unsigned bit = block * 2u + which;

	return (bbt[bit >> 3] >> (bit & 7u)) & 1u;
}

/* --------------------------------------------------------------------- */
/* 8. 平台 DMA 描述符打包（chiplab/IP/DMA/dma.v 的 order 块）             */
/* --------------------------------------------------------------------- */
/*
 * 描述符 = 4 × 64 位 = 32 B，**必须 32 字节对齐**（dma.v:141/470）。
 * 数组下标即 32 位半字：word[2k] 是第 k 个 64 位字的低 32 位，[2k+1] 是高 32 位。
 * 字段映射（dma.v:559-572）：
 *   word[0]  order_addr  低 32 位（Linux 侧传 0，RTL 未使用）
 *   word[1]  mem_addr    高 32 位（DDR 数据缓冲区物理地址）
 *   word[2]  dev_addr    低 32 位（设备数据口，本平台 = NAND 门铃 0x1fe78040）
 *   word[3]  length      高 32 位（**4 字节字数**，不是字节数）
 *   word[4]  step_length 低 32 位（每步 DDR 步进，4 字节字为单位）
 *   word[5]  step_times  高 32 位（步数；必须 = 1，否则 dma_trans_over 判定不同，dma.v:257）
 *   word[6]  cmd         低 32 位（bit12 = dma_r_w：0 = 设备→内存，1 = 内存→设备）
 *   word[7]  状态        高 32 位（引擎回填，软件无需填）
 *
 * 返回 0 成功；参数非法（长度为 0、地址未 32 字节对齐、数组越界）返回负值。
 */
#define CHIPLAB_DMA_DESC_VALID	0
#define CHIPLAB_DMA_DESC_EINVAL	(-1)

static inline int chiplab_nand_dma_desc_pack(uint32_t *desc, unsigned desc_words,
					     uint32_t order_addr, uint32_t mem_addr,
					     uint32_t dev_addr, uint32_t length_bytes,
					     int to_device)
{
	unsigned i;

	if (!desc || desc_words < CHIPLAB_DMA_DESC_WORDS)
		return CHIPLAB_DMA_DESC_EINVAL;
	if ((length_bytes & 3u) || length_bytes == 0)
		return CHIPLAB_DMA_DESC_EINVAL;
	if (mem_addr & (CHIPLAB_DMA_ORDER_ALIGN - 1u))
		return CHIPLAB_DMA_DESC_EINVAL;

	for (i = 0; i < CHIPLAB_DMA_DESC_WORDS; i++)
		desc[i] = 0;

	desc[0] = order_addr;
	desc[1] = mem_addr;
	desc[2] = dev_addr;
	desc[3] = length_bytes / 4u;		/* 单位：4 字节字 */
	desc[4] = 0;				/* step_length = 0 */
	desc[5] = 1;				/* step_times = 1（必须） */
	desc[6] = to_device ? CHIPLAB_DMA_CMD_RW : 0;
	desc[7] = 0;
	return CHIPLAB_DMA_DESC_VALID;
}

/*
 * order 寄存器取值 = 描述符物理地址（低 5 位清零）| dma_start（dma.v:137,141）。
 * 注意掩码必须显式写成 uint32_t：`~(32u - 1u)` 在提升到 64 位时是
 * 0x00000000FFFFFFE0，直接与 64 位量按位与会把高 32 位抹掉。
 */
static inline uint32_t chiplab_nand_dma_order_start(uint32_t desc_phys)
{
	return (desc_phys & CHIPLAB_DMA_ORDER_ADDR_MASK) | CHIPLAB_DMA_ORDER_START;
}

#endif /* __CHIPLAB_NAND_H__ */
