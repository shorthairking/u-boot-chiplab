// SPDX-License-Identifier: GPL-2.0+
/*
 * Board init for the Loongson Chiplab RV32 reference platform
 * (chiplab FPGA SoC, Artix-7 A200T 实验箱).
 *
 * 地址/时钟口径唯一真源：rv32gc-cpu/docs/kb/platform-facts.md
 * 移植方案：rv32gc-cpu/docs/porting/02-uboot.md
 *
 * U-Boot 运行在 S 模式（OpenSBI 在 M 模式，见 docs/porting/01-overview.md §2.5）：
 *   - 定时器 / IPI / 复位一律走 SBI 调用，板级代码不碰 CLINT；
 *   - 串口用 DTS 里的 ns16550a 节点（DM_SERIAL），板级代码不做时钟配置。
 */

#include <config.h>
#include <init.h>
#include <nand.h>
#include <asm/global_data.h>
#include <linux/compiler_attributes.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

/* DDR3 是平台 AXI 的默认从设备：0x0000_0000 - 0x07FF_FFFF (128 MiB)
 * [RTL 实测] platform-facts.md §2.1 (chiplab/IP/AMBA/axi_mux_syn.v:860, 951)
 */
#define CHIPLAB_DDR_BASE	0x00000000UL
#define CHIPLAB_DDR_SIZE	SZ_128M

/*
 * U-Boot 的内存 bank 来自自带设备树的 /memory 节点
 * (arch/riscv/dts/chiplab-rv32.dts -> arch/riscv/cpu/generic/dram.c)。
 * 这里把 DT 的观测值与平台事实台账的常量做一次核对：DTS 与平台事实一旦分叉
 * 就停下（未捕获即失败），而不是带着错的内存图继续启动。
 */
int board_init(void)
{
	if (gd->ram_base != CHIPLAB_DDR_BASE ||
	    gd->ram_size != CHIPLAB_DDR_SIZE) {
		printf("chiplab: DDR3 bank %08lx+%08lx does not match platform facts %08lx+%08lx\n",
		       (ulong)gd->ram_base, (ulong)gd->ram_size,
		       (ulong)CHIPLAB_DDR_BASE, (ulong)CHIPLAB_DDR_SIZE);
		printf("chiplab: fix /memory in arch/riscv/dts/chiplab-rv32.dts "
		       "(see docs/kb/platform-facts.md §2.1)\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * NAND / MTD“编译期预留”的接点。
 *
 * board/loongson/chiplab/Kconfig 里 select 了 SYS_NAND_SELF_INIT，该路径下
 * nand_init() 会直接调用 board_nand_init()（drivers/mtd/nand/raw/nand.c:191），
 * 而 chiplab 的 APB NAND 控制器驱动要到 NAND 阶段才写
 * （见 rv32gc-cpu/docs/porting/04-nand-driver.md 与 platform-facts.md §4）。
 * 这里给一个弱桩：本阶段链接通过、运行期 NAND 栈保持“无设备”；
 * 将来驱动里的强符号会自动覆盖它，删除本桩不是必须动作。
 */
#if CONFIG_IS_ENABLED(MTD_RAW_NAND) && CONFIG_IS_ENABLED(SYS_NAND_SELF_INIT)
__weak void board_nand_init(void)
{
}
#endif
