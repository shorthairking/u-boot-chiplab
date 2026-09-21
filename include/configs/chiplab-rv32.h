/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Config for the Loongson Chiplab RV32 board (Artix-7 A200T 实验箱).
 *
 * U-Boot 运行在 S 模式（M 模式由 OpenSBI 负责，见 rv32gc-cpu/docs/porting/01-overview.md），
 * 因此这里不定义任何 CLINT/M-mode 定时器口径：定时器 / IPI / 复位全部走 SBI。
 *
 * 平台事实唯一真源：rv32gc-cpu/docs/kb/platform-facts.md
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

/*
 * DDR3 = 平台 AXI 默认从设备，0x0000_0000 - 0x07FF_FFFF (128 MiB)。
 * [RTL 实测] platform-facts.md §2.1
 * bank 大小在运行时由 arch/riscv/cpu/generic/dram.c 从自带 DTS 的
 * /memory 节点读入，board/loongson/chiplab/chiplab.c 会与这里的常量核对。
 */
#define CFG_SYS_SDRAM_BASE		0x00000000

/*
 * APB NAND 控制器基址（platform-facts.md §2.5/§4.2；门铃 +0x40）。
 * 控制器驱动（阶段四 NAND）走 SYS_NAND_SELF_INIT 自注册路径，
 * 这里保留常量作为地址口径与 legacy 编译路径的兜底。
 */
#define CFG_SYS_NAND_BASE		0x1fe78000

/*
 * 33 MHz uncore 域（config.h:33 FREQ=33000000，platform-facts.md §5）。
 * 只用于重定位前的早期定时器；DM 定时器从 DTS /cpus/timebase-frequency 取得。
 */
#define RISCV_SMODE_TIMER_FREQ		33000000

/*
 * DDR3 内地址划分（U-Boot 视角；权威表见 docs/porting/01-overview.md §2.3）
 *
 *   0x0000_0000  阶段A/B 早期引导暂存（本板 U-Boot 不用）
 *   0x0100_0000  OpenSBI (M 模式, ≤ 1 MiB)
 *   0x01e0_0000  U-Boot 重定位前栈/malloc（CUSTOM_SYS_INIT_SP_ADDR）
 *   0x0200_0000  U-Boot (S) 代码段（TEXT_BASE）
 *   0x0400_0000  Linux kernel（kernel_addr_r，≤ 50 MiB，对齐 NAND kernel 分区）
 *   0x0740_0000  DTB（fdt_addr_r）
 *   0x0750_0000  initramfs/ramdisk（ramdisk_addr_r）
 *   0x07a0_0000+ U-Boot 重定位区 + malloc（DDR 顶端）
 */
#define CFG_EXTRA_ENV_SETTINGS \
	"kernel_addr_r=0x04000000\0" \
	"fdt_addr_r=0x07400000\0" \
	"ramdisk_addr_r=0x07500000\0" \
	"scriptaddr=0x07300000\0" \
	"pxefile_addr_r=0x07380000\0" \
	"loadaddr=0x04000000\0"

#endif /* __CONFIG_H */
