// SPDX-License-Identifier: GPL-2.0+
/*
 * Loongson Chiplab FPGA SoC (Artix-7 A200T) - DEC 21140 "Tulip" compatible MAC
 *
 * 硬件口径唯一真源：chiplab/IP/MAC/ 下的 Verilog 源码
 *   - 寄存器/CSR      csr.v（CSR*_ID 定义在 utility.v:77-134）
 *   - 收描述符/过滤   rlsm.v + rc.v
 *   - 发描述符/SETUP  tlsm.v
 *   - 软复位          rstc.v
 * 逐条对齐表见 docs/linux-port/mac-tftp-boot.md（§4 RTL 对齐表）。
 *
 * 与 la32r-uboot/drivers/net/dmfe.c（龙芯旧驱动）的差异（刻意为之）：
 *   1) MAC 地址来自 pdata->enetaddr（DT / ethaddr），不写死 00:98:76:64:32:19；
 *   2) DMA 地址就是物理地址（本板 U-Boot 无 MMU，虚拟==物理），
 *      不做 CACHED_TO_UNCACHED 窗口偏移；
 *   3) 缓存一致性显式用 flush/invalidate_dcache_range；
 *   4) RV32 描述符/缓冲默认 32 B 对齐（ARCH_DMA_MINALIGN）；
 *   5) SETUP 帧按 RTL 的实际比对顺序摆放（见 chiplab_dmfe_setup_frame()），
 *      不照抄旧驱动的顺序（旧驱动靠 CSR6.PR/PM 全通过掩盖了顺序差异）。
 */

#include <cpu_func.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <miiphy.h>
#include <net.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>

/* ------------------------------------------------------------------ */
/* CSR：CSR n 位于字节偏移 n*8（CSR0_ID=0、CSR1_ID=2 … CSR11_ID=22）  */
/* ------------------------------------------------------------------ */
#define CHIPLAB_CSR0		0x00	/* 总线模式：SWR/DSL/PBL/BLE */
#define CHIPLAB_CSR1		0x08	/* 发送轮询请求（写触发，读 0） */
#define CHIPLAB_CSR2		0x10	/* 接收轮询请求（写触发，读 0） */
#define CHIPLAB_CSR3		0x18	/* 接收环基址 */
#define CHIPLAB_CSR4		0x20	/* 发送环基址 */
#define CHIPLAB_CSR5		0x28	/* 状态（W1C） */
#define CHIPLAB_CSR6		0x30	/* 工作模式（ST/SR/PR/PM/FD…） */
#define CHIPLAB_CSR7		0x38	/* 中断使能 */
#define CHIPLAB_CSR8		0x40	/* 丢失帧/计数器 */
#define CHIPLAB_CSR9		0x48	/* MIIM 软件位拍 + SROM */
#define CHIPLAB_CSR10		0x50	/* bit0 = insert_en */
#define CHIPLAB_CSR11		0x58	/* 定时器/看门狗 */

/* CSR0（csr.v:1150-1152 读回，csr.v:571-597 写入） */
#define CSR0_SWR		BIT(0)	/* 软复位；读回 (rst | swr)，不自清 */
#define CSR0_DSL_SHIFT		2
#define CSR0_DSL_MASK		(0x1f << CSR0_DSL_SHIFT)

/* CSR5（csr.v:1153-1156 读回；csr.v:746-790 写 1 清） */
#define CSR5_TI			BIT(0)	/* 发送完成（W1C） */
#define CSR5_TPS		BIT(1)	/* 发送挂起（W1C） */
#define CSR5_TU			BIT(2)	/* 发送下溢（W1C） */
#define CSR5_UNF		BIT(5)	/* 接收下溢（W1C） */
#define CSR5_RI			BIT(6)	/* 接收完成（W1C） */
#define CSR5_RU			BIT(7)	/* 接收缓冲不可用（W1C） */
#define CSR5_RPS		BIT(8)	/* 接收挂起（W1C） */
#define CSR5_ETI		BIT(10)	/* 提前发送中断（W1C） */
#define CSR5_GTE		BIT(11)	/* 通用定时器到期（W1C） */
#define CSR5_ERI		BIT(14)	/* 早期接收中断（W1C） */
#define CSR5_AIS		BIT(15)	/* 异常中断汇总（W1C） */
#define CSR5_NIS		BIT(16)	/* 正常中断汇总（W1C） */
#define CSR5_RS_SHIFT		17	/* 接收进程状态 [19:17] */
#define CSR5_TS_SHIFT		20	/* 发送进程状态 [22:20] */
#define CSR5_RS_MASK		(0x7 << CSR5_RS_SHIFT)
#define CSR5_TS_MASK		(0x7 << CSR5_TS_SHIFT)
#define CSR5_W1C_BITS		(CSR5_TI | CSR5_TPS | CSR5_TU | CSR5_UNF | \
				 CSR5_RI | CSR5_RU | CSR5_RPS | CSR5_ETI | \
				 CSR5_GTE | CSR5_ERI | CSR5_AIS | CSR5_NIS)

/* CSR6（csr.v:1156-1163 读回；csr.v:893-950 写入：st=bit13, sr=bit1） */
#define CSR6_HP			BIT(0)	/* 仅哈希过滤 */
#define CSR6_SR			BIT(1)	/* 启动/停止接收 */
#define CSR6_HO			BIT(2)	/* 哈希过滤 */
#define CSR6_PB			BIT(3)	/* 通过坏帧 */
#define CSR6_IF			BIT(4)	/* 反过滤 */
#define CSR6_PR			BIT(6)	/* 混杂模式（通过全部单播） */
#define CSR6_PM			BIT(7)	/* 通过全部组播 */
#define CSR6_FD			BIT(9)	/* 全双工 */
#define CSR6_ST			BIT(13)	/* 启动/停止发送 */
#define CSR6_TR			(3 << 14)/* 发送阈（保留，RTL 只存不用） */
#define CSR6_SF			BIT(21)	/* 存储转发 */
#define CSR6_TTM		BIT(22)	/* 10 Mbps 发送阈 */
#define CSR6_PS			BIT(18)	/* 端口选择（Tulip 语义，本 RTL 忽略） */
#define CSR6_SDP		BIT(25)	/* SD 极性（Tulip 语义，本 RTL 忽略） */

/* CSR9（csr.v:1174-1176 读回；csr.v:1044-1071 写入） */
#define CSR9_MDC		BIT(16)	/* 软件直接翻转 MDC 电平 */
#define CSR9_MDO		BIT(17)	/* MDIO 输出数据 */
#define CSR9_MDEN		BIT(18)	/* MDIO 输出使能（1=MAC 驱动） */
#define CSR9_MDI		BIT(19)	/* MDIO 输入数据（PHY 驱动） */

/* CSR10（csr.v:1178） */
#define CSR10_INSERT_EN		BIT(0)	/* 置位后收帧长（RDES0[29:16]）减 2 */

/* 描述符控制/状态位（TDES0/TDES1/RDES0/RDES1） */
#define DESC_OWN		BIT(31)	/* TDES0/RDES0 的 OWN 位 */
#define TDES1_IC		BIT(31)	/* 完成时中断 */
#define TDES1_LS		BIT(30)	/* 最后一段 */
#define TDES1_FS		BIT(29)	/* 第一段 */
#define TDES1_SET		BIT(27)	/* SETUP 帧（写地址过滤 RAM，不发送） */
#define TDES1_TER		BIT(25)	/* 环尾（回到 CSR4 基址） */
#define TDES1_TCH		BIT(24)	/* 第二地址链（用 TDES3 作 next） */
#define TDES1_BS1_MASK		0x7ff	/* 缓冲 1 字节数 [10:0] */
#define RDES0_FF		BIT(30)	/* 首描述符（rlsm.v:545 ff） */
#define RDES0_LEN_SHIFT		16	/* 帧长 [29:16]（rlsm.v:543-545） */
#define RDES0_LEN_MASK		0x3fff
#define RDES0_ES		BIT(15)	/* 错误汇总（rlsm.v:547 res_c） */
#define RDES0_LS		BIT(8)	/* 末描述符（rlsm.v:545 rls） */
#define RDES0_FS		BIT(9)	/* 首状态写（rlsm.v:545 rfs） */
#define RDES1_RER		BIT(25)	/* 环尾（rlsm.v:940 rer） */
#define RDES1_RCH		BIT(24)	/* 链模式（rlsm.v:941 rch） */
#define RDES1_BS1_MASK		0x7ff

#define CHIPLAB_DMFE_RX_DESC	8
#define CHIPLAB_DMFE_TX_DESC	8
#define CHIPLAB_DMFE_RX_BUF	2048	/* 单缓冲（含对齐余量） */
#define CHIPLAB_DMFE_TX_BUF	2048
#define CHIPLAB_DMFE_SETUP_LEN	192	/* 与 Tulip/RTL 一致（dma 按字写 48 项） */
#define CHIPLAB_DMFE_TIMEOUT_MS	2000
#define CHIPLAB_DMFE_LINK_TIMEOUT_MS 200	/* PHY 链路等待上限 */
#define CHIPLAB_DMFE_PHY_ADDR	1	/* 板级 PHYAD=1（platform-facts.md §6） */

struct chiplab_dmfe_desc {
	u32 status;		/* TDES0 / RDES0 */
	u32 des1;		/* TDES1 / RDES1 */
	u32 buf;		/* TDES2 / RDES2：数据缓冲物理地址 */
	u32 next;		/* TDES3 / RDES3：链指针 / 第二缓冲 */
};

struct chiplab_dmfe_priv {
	void __iomem *base;
	struct mii_dev *bus;

	struct chiplab_dmfe_desc *rx_ring;
	phys_addr_t rx_ring_phys;
	u8 *rx_buf;			/* CHIPLAB_DMFE_RX_DESC * RX_BUF */
	phys_addr_t rx_buf_phys;
	u32 rx_buf_size;
	int rx_new;

	struct chiplab_dmfe_desc *tx_ring;
	phys_addr_t tx_ring_phys;
	u8 *tx_buf;
	phys_addr_t tx_buf_phys;
	u32 tx_buf_size;
	int tx_new;

	u8 *setup_frame;
	phys_addr_t setup_frame_phys;
};

/* ------------------------------------------------------------------ */
/* 寄存器访问                                                          */
/* ------------------------------------------------------------------ */
static inline u32 chiplab_dmfe_rd(struct chiplab_dmfe_priv *priv, u32 off)
{
	return readl(priv->base + off);
}

static inline void chiplab_dmfe_wr(struct chiplab_dmfe_priv *priv, u32 off,
				   u32 val)
{
	writel(val, priv->base + off);
}

/* ------------------------------------------------------------------ */
/* MIIM 软件位拍（CSR9 bit16=MDC 电平、bit17=MDO、bit18=MDEN、bit19=MDI）*/
/* ------------------------------------------------------------------ */
static void chiplab_dmfe_mdio_drive(struct chiplab_dmfe_priv *priv, int bit)
{
	u32 v = CSR9_MDEN;

	if (bit)
		v |= CSR9_MDO;

	chiplab_dmfe_wr(priv, CHIPLAB_CSR9, v);		/* MDC=0，摆好数据 */
	udelay(1);
	chiplab_dmfe_wr(priv, CHIPLAB_CSR9, v | CSR9_MDC);	/* 上升沿 */
	udelay(1);
	chiplab_dmfe_wr(priv, CHIPLAB_CSR9, v);		/* 下降沿 */
	udelay(1);
}

static int chiplab_dmfe_mdio_sample(struct chiplab_dmfe_priv *priv)
{
	int bit;

	chiplab_dmfe_wr(priv, CHIPLAB_CSR9, 0);		/* 释放总线（MDEN=0）*/
	udelay(1);
	chiplab_dmfe_wr(priv, CHIPLAB_CSR9, CSR9_MDC);	/* 上升沿，PHY 摆数据 */
	udelay(1);
	bit = !!(chiplab_dmfe_rd(priv, CHIPLAB_CSR9) & CSR9_MDI);
	chiplab_dmfe_wr(priv, CHIPLAB_CSR9, 0);		/* 下降沿 */
	udelay(1);

	return bit;
}

/*
 * 标准 IEEE 802.3 clause 22 帧：32 个前导 1 + ST=01 + OP(10 读/01 写)
 * + PHYAD(5) + REGAD(5) + TA + DATA(16)
 */
static int chiplab_dmfe_mdio_xfer(struct chiplab_dmfe_priv *priv, int phyad,
				  int regad, bool write, u16 *data)
{
	int i;
	u16 val = write ? *data : 0;

	/* 前导 + ST + OP + PHYAD + REGAD = 46 bit，压进 cmd 的高 46 位 */
	for (i = 0; i < 32; i++)
		chiplab_dmfe_mdio_drive(priv, 1);
	chiplab_dmfe_mdio_drive(priv, 0);	/* ST = 01 */
	chiplab_dmfe_mdio_drive(priv, 1);
	chiplab_dmfe_mdio_drive(priv, write ? 0 : 1);	/* OP = 01 / 10 */
	chiplab_dmfe_mdio_drive(priv, write ? 1 : 0);

	for (i = 4; i >= 0; i--)			/* PHYAD[4:0] */
		chiplab_dmfe_mdio_drive(priv, (phyad >> i) & 1);
	for (i = 4; i >= 0; i--)			/* REGAD[4:0] */
		chiplab_dmfe_mdio_drive(priv, (regad >> i) & 1);

	if (write) {
		chiplab_dmfe_mdio_drive(priv, 1);	/* TA = 10 */
		chiplab_dmfe_mdio_drive(priv, 0);
		for (i = 15; i >= 0; i--)
			chiplab_dmfe_mdio_drive(priv, (val >> i) & 1);
	} else {
		chiplab_dmfe_mdio_drive(priv, 0);	/* TA = Z0 */
		chiplab_dmfe_mdio_sample(priv);		/* 丢弃 TA 的第 2 拍 */
		for (i = 15; i >= 0; i--)
			val = (val << 1) | chiplab_dmfe_mdio_sample(priv);
		*data = val;
	}

	return 0;
}

static int chiplab_dmfe_mdio_read(struct mii_dev *bus, int phyad, int devad,
				  int regad)
{
	struct chiplab_dmfe_priv *priv = bus->priv;
	u16 val = 0;

	chiplab_dmfe_mdio_xfer(priv, phyad, regad, false, &val);

	return val;
}

static int chiplab_dmfe_mdio_write(struct mii_dev *bus, int phyad, int devad,
				   int regad, u16 val)
{
	struct chiplab_dmfe_priv *priv = bus->priv;

	chiplab_dmfe_mdio_xfer(priv, phyad, regad, true, &val);

	return 0;
}

static int chiplab_dmfe_mdio_init(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	struct mii_dev *bus;
	int ret;

	bus = mdio_alloc();
	if (!bus)
		return -ENOMEM;

	bus->read = chiplab_dmfe_mdio_read;
	bus->write = chiplab_dmfe_mdio_write;
	bus->priv = priv;

	ret = mdio_register_seq(bus, dev_seq(dev));
	if (ret) {
		free(bus);
		return ret;
	}

	priv->bus = bus;

	return 0;
}

/* ------------------------------------------------------------------ */
/* 描述符环                                                            */
/* ------------------------------------------------------------------ */
static void chiplab_dmfe_rx_ring_init(struct chiplab_dmfe_priv *priv)
{
	int i;

	for (i = 0; i < CHIPLAB_DMFE_RX_DESC; i++) {
		priv->rx_ring[i].status = DESC_OWN;
		priv->rx_ring[i].des1 = priv->rx_buf_size & RDES1_BS1_MASK;
		priv->rx_ring[i].buf = (u32)(priv->rx_buf_phys +
					     (phys_addr_t)i * priv->rx_buf_size);
		priv->rx_ring[i].next = (u32)(priv->rx_ring_phys +
					      (phys_addr_t)((i + 1) %
					      CHIPLAB_DMFE_RX_DESC) *
					      sizeof(struct chiplab_dmfe_desc));
	}

	/*
	 * RTL 的步进：rer（RDES1[25]）置位 ⇒ 下一描述符回到 CSR3 基址；
	 * 未置位 ⇒ 当前 + {dsl,2'b00}（rlsm.v:1035-1046）。本驱动用连续
	 * 16 B 步长（dsl=0），同时给出 next 链指针，两种语义都能工作。
	 */
	priv->rx_ring[CHIPLAB_DMFE_RX_DESC - 1].des1 |= RDES1_RER;
	priv->rx_new = 0;
}

static void chiplab_dmfe_tx_ring_init(struct chiplab_dmfe_priv *priv)
{
	int i;

	for (i = 0; i < CHIPLAB_DMFE_TX_DESC; i++) {
		priv->tx_ring[i].status = 0;
		priv->tx_ring[i].des1 = TDES1_IC | TDES1_LS | TDES1_FS |
					TDES1_TCH;
		priv->tx_ring[i].buf = (u32)(priv->tx_buf_phys +
					     (phys_addr_t)i * priv->tx_buf_size);
		priv->tx_ring[i].next = (u32)(priv->tx_ring_phys +
					      (phys_addr_t)((i + 1) %
					      CHIPLAB_DMFE_TX_DESC) *
					      sizeof(struct chiplab_dmfe_desc));
	}

	priv->tx_ring[CHIPLAB_DMFE_TX_DESC - 1].des1 |= TDES1_TER;
	priv->tx_new = 0;
}

static void chiplab_dmfe_ring_flush(struct chiplab_dmfe_priv *priv)
{
	ulong start = (ulong)priv->rx_ring & ~(ARCH_DMA_MINALIGN - 1);
	ulong end = (ulong)priv->tx_ring +
		    CHIPLAB_DMFE_TX_DESC * sizeof(struct chiplab_dmfe_desc);

	flush_dcache_range(start, roundup(end, ARCH_DMA_MINALIGN));
}

/*
 * SETUP 帧：TDES1.SET 置位的发送描述符不发送，而是把帧体按 16 bit 逐字写进
 * 64x16 地址过滤 RAM（tlsm.v:2107-2160：ifwe 时 faddr=ifaddr 递增、
 * fdata = DMA 数据字的低 16 位）。
 * rc.v:1454-1470 的比对顺序是：
 *   RAM[0] == DA[2]|DA[3]<<8 ; RAM[1] == DA[0]|DA[1]<<8 ;
 *   RAM[2] == DA[4]|DA[5]<<8
 * 因此 setup 帧体第 0/1/2 个字（字节 0..1 / 4..5 / 8..9）按上述顺序填。
 */
static void chiplab_dmfe_setup_frame(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);
	const u8 *a = pdata->enetaddr;
	struct chiplab_dmfe_desc *desc = &priv->tx_ring[priv->tx_new];
	int start;

	memset(priv->setup_frame, 0, CHIPLAB_DMFE_SETUP_LEN);
	priv->setup_frame[0] = a[2];
	priv->setup_frame[1] = a[3];
	priv->setup_frame[4] = a[0];
	priv->setup_frame[5] = a[1];
	priv->setup_frame[8] = a[4];
	priv->setup_frame[9] = a[5];

	start = get_timer(0);
	while (desc->status & DESC_OWN) {
		if (get_timer(start) > CHIPLAB_DMFE_TIMEOUT_MS) {
			dev_warn(dev, "setup: tx descriptor busy\n");
			return;
		}
		udelay(1);
	}

	desc->buf = (u32)priv->setup_frame_phys;
	desc->des1 = TDES1_TCH | TDES1_SET | CHIPLAB_DMFE_SETUP_LEN;
	desc->status = DESC_OWN;

	flush_dcache_range((ulong)priv->setup_frame & ~(ARCH_DMA_MINALIGN - 1),
			   roundup((ulong)priv->setup_frame +
				   CHIPLAB_DMFE_SETUP_LEN,
				   ARCH_DMA_MINALIGN));
	chiplab_dmfe_ring_flush(priv);

	chiplab_dmfe_wr(priv, CHIPLAB_CSR1, 1);	/* 发送轮询请求 */

	start = get_timer(0);
	while (desc->status & DESC_OWN) {
		if (get_timer(start) > CHIPLAB_DMFE_TIMEOUT_MS) {
			dev_warn(dev, "setup: mac did not consume the frame\n");
			break;
		}
		invalidate_dcache_range(
			(ulong)desc & ~(ARCH_DMA_MINALIGN - 1),
			roundup((ulong)desc + sizeof(*desc), ARCH_DMA_MINALIGN));
		udelay(1);
	}

	/* 复位该描述符，交回发送环 */
	desc->des1 = TDES1_IC | TDES1_LS | TDES1_FS | TDES1_TCH;
	desc->buf = (u32)(priv->tx_buf_phys +
			  (phys_addr_t)priv->tx_new * priv->tx_buf_size);
	priv->tx_new = (priv->tx_new + 1) % CHIPLAB_DMFE_TX_DESC;
	chiplab_dmfe_ring_flush(priv);

	debug("%s: filter RAM loaded from %p (DA %pM)\n", __func__,
	      priv->setup_frame, a);
}

/* ------------------------------------------------------------------ */
/* 设备操作                                                            */
/* ------------------------------------------------------------------ */
static int chiplab_dmfe_reset(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	u32 v;
	int start;

	/* CSR0.SWR 请求软复位（rstc.v：复位握手期间 csr0 读回 rst|swr） */
	v = chiplab_dmfe_rd(priv, CHIPLAB_CSR0);
	chiplab_dmfe_wr(priv, CHIPLAB_CSR0, v | CSR0_SWR);
	udelay(1000);

	/* 清 SWR + DSL=0：本 RTL 的 swr 位不自清，必须软件写回 0 */
	chiplab_dmfe_wr(priv, CHIPLAB_CSR0, 0);

	start = get_timer(0);
	for (;;) {
		v = chiplab_dmfe_rd(priv, CHIPLAB_CSR0);
		if (!(v & CSR0_SWR))
			break;
		if (get_timer(start) > CHIPLAB_DMFE_TIMEOUT_MS) {
			dev_err(dev, "reset timeout (CSR0=%08x)\n", v);
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	/* 复位后 TS/RS 状态必须是 0（csr5_ts/csr5_rs 由发送/接收状态机驱动） */
	start = get_timer(0);
	for (;;) {
		v = chiplab_dmfe_rd(priv, CHIPLAB_CSR5);
		if (!(v & (CSR5_TS_MASK | CSR5_RS_MASK)))
			break;
		if (get_timer(start) > CHIPLAB_DMFE_TIMEOUT_MS) {
			dev_err(dev, "controller not idle (CSR5=%08x)\n", v);
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	/* 轮询模式：关掉全部中断使能（中断脚本阶段不接） */
	chiplab_dmfe_wr(priv, CHIPLAB_CSR7, 0);

	/* Tulip 语义的端口选择位；本 RTL 忽略这两位（写路径无 PS/SDP） */
	chiplab_dmfe_wr(priv, CHIPLAB_CSR6, CSR6_SDP | CSR6_PS);

	return 0;
}

static int chiplab_dmfe_phy_config(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	u16 id1, id2, bmsr, bmcr;
	int start;

	id1 = chiplab_dmfe_mdio_read(priv->bus, CHIPLAB_DMFE_PHY_ADDR, 0, 2);
	id2 = chiplab_dmfe_mdio_read(priv->bus, CHIPLAB_DMFE_PHY_ADDR, 0, 3);
	/*
	 * PHY 型号未知（板卡无丝印资料）：只打印识别结果，不做 ID 匹配，
	 * 因此不阻塞任何一种通用 MII PHY。QEMU 模型按通用 MII PHY 建模。
	 */
	printf("chiplab-dmfe: PHYAD=%d PHYIDR=0x%04x%04x (generic MII assumed)\n",
	       CHIPLAB_DMFE_PHY_ADDR, id1, id2);

	/* 100 Mbps 全双工、关自协商（本板 MAC 侧只支持 MII 10/100） */
	bmcr = chiplab_dmfe_mdio_read(priv->bus, CHIPLAB_DMFE_PHY_ADDR, 0, 0);
	chiplab_dmfe_mdio_write(priv->bus, CHIPLAB_DMFE_PHY_ADDR, 0, 0,
				(bmcr & ~BIT(12)) | BIT(13) | BIT(8));

	start = get_timer(0);
	for (;;) {
		bmsr = chiplab_dmfe_mdio_read(priv->bus, CHIPLAB_DMFE_PHY_ADDR,
					      0, 1);
		if (bmsr & BIT(2))	/* link status */
			break;
		if (get_timer(start) > CHIPLAB_DMFE_LINK_TIMEOUT_MS) {
			dev_warn(dev, "PHY link down (BMSR=%04x), continue anyway\n",
				 bmsr);
			break;
		}
		udelay(1000);
	}

	printf("chiplab-dmfe: PHY link %s (BMSR=0x%04x)\n",
	       (bmsr & BIT(2)) ? "up" : "down", bmsr);

	return 0;
}

static int chiplab_dmfe_start(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	u32 csr6;
	int ret;

	ret = chiplab_dmfe_reset(dev);
	if (ret)
		return ret;

	chiplab_dmfe_phy_config(dev);

	chiplab_dmfe_rx_ring_init(priv);
	chiplab_dmfe_tx_ring_init(priv);
	chiplab_dmfe_ring_flush(priv);

	/* 环基址：CSR3 = 接收环，CSR4 = 发送环（低 5 位被 RTL 忽略） */
	chiplab_dmfe_wr(priv, CHIPLAB_CSR3, (u32)priv->rx_ring_phys);
	chiplab_dmfe_wr(priv, CHIPLAB_CSR4, (u32)priv->tx_ring_phys);
	udelay(100);

	/*
	 * 启动收发：ST(bit13)/SR(bit1) 是命令位（csr.v:912/920）。
	 *
	 * PR/PM/PB 三个过滤位都要置位，这不是"保险"，而是本 RTL 的硬性口径：
	 * rc.v:599/621/647 的收帧判定是
	 *     if ((pb) & (fsm == FSM_MATCH | ra | (pm & (dest[0]))))
	 * PR 让过滤状态机直接进 FSM_MATCH（rc.v:1241），但 PB（bit3）是**与**项：
	 * PB=0 时任何帧（含混杂模式下的单播）都会被判 RSM_BAD 丢掉。
	 * 标准 Tulip 里 PB 是"通过坏帧"，语义不同——这里必须按 RTL 置 1。
	 * SETUP 帧仍照发（对齐 Tulip 语义并把地址写进过滤 RAM）。
	 */
	csr6 = chiplab_dmfe_rd(priv, CHIPLAB_CSR6);
	csr6 |= CSR6_ST | CSR6_SR | CSR6_PB | CSR6_PR | CSR6_PM | CSR6_FD |
		CSR6_TTM | CSR6_SF;
	chiplab_dmfe_wr(priv, CHIPLAB_CSR6, csr6);

	chiplab_dmfe_setup_frame(dev);

	debug("%s: CSR0=%08x CSR5=%08x CSR6=%08x\n", __func__,
	      chiplab_dmfe_rd(priv, CHIPLAB_CSR0),
	      chiplab_dmfe_rd(priv, CHIPLAB_CSR5),
	      chiplab_dmfe_rd(priv, CHIPLAB_CSR6));

	return 0;
}

static int chiplab_dmfe_send(struct udevice *dev, void *packet, int length)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	struct chiplab_dmfe_desc *desc = &priv->tx_ring[priv->tx_new];
	ulong buf_start, buf_end;
	int start;

	if (length <= 0 || length > priv->tx_buf_size)
		return -EINVAL;

	/* 以太网最小帧 60 B（不含 FCS），RTL 不做补齐，由驱动保证 */
	if (length < 60)
		length = 60;

	start = get_timer(0);
	while (desc->status & DESC_OWN) {
		if (get_timer(start) > CHIPLAB_DMFE_TIMEOUT_MS) {
			dev_err(dev, "tx descriptor %d stuck (status=%08x)\n",
				priv->tx_new, desc->status);
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	memset(priv->tx_buf + (ulong)priv->tx_new * priv->tx_buf_size, 0,
	       length);
	memcpy(priv->tx_buf + (ulong)priv->tx_new * priv->tx_buf_size, packet,
	       length);

	buf_start = (ulong)desc->buf & ~(ARCH_DMA_MINALIGN - 1);
	buf_end = roundup((ulong)desc->buf + length, ARCH_DMA_MINALIGN);
	flush_dcache_range(buf_start, buf_end);

	desc->des1 = TDES1_IC | TDES1_LS | TDES1_FS | TDES1_TCH |
		     (length & TDES1_BS1_MASK);
	desc->status = DESC_OWN;
	chiplab_dmfe_ring_flush(priv);

	chiplab_dmfe_wr(priv, CHIPLAB_CSR1, 1);	/* 发送轮询请求（写 1 即可） */

	start = get_timer(0);
	for (;;) {
		invalidate_dcache_range(
			(ulong)desc & ~(ARCH_DMA_MINALIGN - 1),
			roundup((ulong)desc + sizeof(*desc), ARCH_DMA_MINALIGN));
		if (!(desc->status & DESC_OWN))
			break;
		if (get_timer(start) > CHIPLAB_DMFE_TIMEOUT_MS) {
			dev_err(dev, "tx timeout (status=%08x)\n", desc->status);
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	if (desc->status & BIT(15))	/* TDES0 覆写后的错误汇总位 */
		dev_dbg(dev, "tx completed with status %08x\n", desc->status);

	priv->tx_new = (priv->tx_new + 1) % CHIPLAB_DMFE_TX_DESC;

	/* 清发送完成中断位（本阶段纯轮询，顺手 W1C 保持状态干净） */
	chiplab_dmfe_wr(priv, CHIPLAB_CSR5, CSR5_TI | CSR5_NIS);

	return 0;
}

static int chiplab_dmfe_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	struct chiplab_dmfe_desc *desc;
	int i, length = -EAGAIN;

	for (i = 0; i < CHIPLAB_DMFE_RX_DESC; i++) {
		u32 status;

		desc = &priv->rx_ring[priv->rx_new];
		invalidate_dcache_range(
			(ulong)desc & ~(ARCH_DMA_MINALIGN - 1),
			roundup((ulong)desc + sizeof(*desc), ARCH_DMA_MINALIGN));
		status = desc->status;

		if (status & DESC_OWN)
			break;		/* MAC 还没写完这一项 */

		if ((status & RDES0_LS) && !(status & RDES0_ES)) {
			ulong buf_start = desc->buf &
					  ~(ARCH_DMA_MINALIGN - 1);
			ulong buf_end;

			length = (status >> RDES0_LEN_SHIFT) &
				 RDES0_LEN_MASK;
			if (length <= 0 || length > priv->rx_buf_size) {
				length = -EIO;
			} else {
				buf_end = roundup(desc->buf + length,
						  ARCH_DMA_MINALIGN);
				invalidate_dcache_range(buf_start, buf_end);
				*packetp = (uchar *)(ulong)desc->buf;
			}
		}

		/* 交回 MAC：OWN 置位（RTL 不看 RDES1，只要 OWN） */
		desc->status = DESC_OWN;
		desc->des1 = priv->rx_buf_size & RDES1_BS1_MASK;
		if (priv->rx_new == CHIPLAB_DMFE_RX_DESC - 1)
			desc->des1 |= RDES1_RER;
		flush_dcache_range(
			(ulong)desc & ~(ARCH_DMA_MINALIGN - 1),
			roundup((ulong)desc + sizeof(*desc), ARCH_DMA_MINALIGN));

		priv->rx_new = (priv->rx_new + 1) % CHIPLAB_DMFE_RX_DESC;

		if (length != -EAGAIN)
			break;
	}

	if (length >= 0)
		chiplab_dmfe_wr(priv, CHIPLAB_CSR5, CSR5_RI | CSR5_NIS);

	return length;
}

static int chiplab_dmfe_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	return 0;
}

static void chiplab_dmfe_stop(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	u32 csr6;

	csr6 = chiplab_dmfe_rd(priv, CHIPLAB_CSR6);
	chiplab_dmfe_wr(priv, CHIPLAB_CSR6, csr6 & ~(CSR6_ST | CSR6_SR));
}

static int chiplab_dmfe_write_hwaddr(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);

	debug("%s: %pM\n", __func__, pdata->enetaddr);

	return 0;
}

static const struct eth_ops chiplab_dmfe_ops = {
	.start		= chiplab_dmfe_start,
	.send		= chiplab_dmfe_send,
	.recv		= chiplab_dmfe_recv,
	.free_pkt	= chiplab_dmfe_free_pkt,
	.stop		= chiplab_dmfe_stop,
	.write_hwaddr	= chiplab_dmfe_write_hwaddr,
};

static int chiplab_dmfe_of_to_plat(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);

	pdata->iobase = dev_read_addr(dev);
	pdata->phy_interface = PHY_INTERFACE_MODE_MII;
	pdata->max_speed = 100;

	return 0;
}

static int chiplab_dmfe_probe(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);
	size_t ring_bytes = CHIPLAB_DMFE_RX_DESC *
			    sizeof(struct chiplab_dmfe_desc);
	size_t buf_bytes;
	int ret;

	priv->base = map_physmem(pdata->iobase, 0x1000, MAP_NOCACHE);
	if (!priv->base)
		return -ENOMEM;

	ring_bytes = roundup(ring_bytes, ARCH_DMA_MINALIGN);
	buf_bytes = roundup(CHIPLAB_DMFE_RX_DESC * CHIPLAB_DMFE_RX_BUF,
			    ARCH_DMA_MINALIGN);
	priv->rx_buf_size = CHIPLAB_DMFE_RX_BUF;
	priv->tx_buf_size = CHIPLAB_DMFE_TX_BUF;

	priv->rx_ring = memalign(ARCH_DMA_MINALIGN, ring_bytes);
	priv->tx_ring = memalign(ARCH_DMA_MINALIGN, ring_bytes);
	priv->rx_buf = memalign(ARCH_DMA_MINALIGN, buf_bytes);
	priv->tx_buf = memalign(ARCH_DMA_MINALIGN,
				roundup(CHIPLAB_DMFE_TX_DESC * priv->tx_buf_size,
					ARCH_DMA_MINALIGN));
	priv->setup_frame = memalign(ARCH_DMA_MINALIGN,
				     roundup(CHIPLAB_DMFE_SETUP_LEN,
					     ARCH_DMA_MINALIGN));
	if (!priv->rx_ring || !priv->tx_ring || !priv->rx_buf ||
	    !priv->tx_buf || !priv->setup_frame) {
		ret = -ENOMEM;
		goto err_free;
	}

	/* 本板 U-Boot 无 MMU：虚拟地址 == 物理地址 */
	priv->rx_ring_phys = (phys_addr_t)(ulong)priv->rx_ring;
	priv->tx_ring_phys = (phys_addr_t)(ulong)priv->tx_ring;
	priv->rx_buf_phys = (phys_addr_t)(ulong)priv->rx_buf;
	priv->tx_buf_phys = (phys_addr_t)(ulong)priv->tx_buf;
	priv->setup_frame_phys = (phys_addr_t)(ulong)priv->setup_frame;

	memset(priv->rx_ring, 0, ring_bytes);
	memset(priv->tx_ring, 0, ring_bytes);

	ret = chiplab_dmfe_mdio_init(dev);
	if (ret)
		goto err_free;

	return 0;

err_free:
	free(priv->rx_ring);
	free(priv->tx_ring);
	free(priv->rx_buf);
	free(priv->tx_buf);
	free(priv->setup_frame);

	return ret;
}

static int chiplab_dmfe_remove(struct udevice *dev)
{
	struct chiplab_dmfe_priv *priv = dev_get_priv(dev);

	if (priv->bus) {
		mdio_unregister(priv->bus);
		mdio_free(priv->bus);
	}

	free(priv->rx_ring);
	free(priv->tx_ring);
	free(priv->rx_buf);
	free(priv->tx_buf);
	free(priv->setup_frame);

	return 0;
}

static const struct udevice_id chiplab_dmfe_ids[] = {
	{ .compatible = "loongson,chiplab-dmfe" },
	{ }
};

U_BOOT_DRIVER(chiplab_dmfe) = {
	.name		= "chiplab_dmfe",
	.id		= UCLASS_ETH,
	.of_match	= chiplab_dmfe_ids,
	.of_to_plat	= chiplab_dmfe_of_to_plat,
	.probe		= chiplab_dmfe_probe,
	.remove		= chiplab_dmfe_remove,
	.ops		= &chiplab_dmfe_ops,
	.priv_auto	= sizeof(struct chiplab_dmfe_priv),
	.plat_auto	= sizeof(struct eth_pdata),
};
