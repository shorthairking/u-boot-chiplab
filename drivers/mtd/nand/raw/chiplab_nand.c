// SPDX-License-Identifier: GPL-2.0+
/*
 * Chiplab (Loongson Artix-7 实验箱) APB NAND 控制器驱动
 *
 * 控制器：chiplab/IP/APB_DEV/NAND/nand.v（模块 NAND_top，1430 行）
 * 平台 DMA：chiplab/IP/DMA/dma.v（描述符驱动；order 寄存器在 confreg
 *           `define ORDER_REG_ADDR 16'h1160 → 0x1FD0_1160）
 * 芯片：K9F1G08U0C（128 MiB，页 2048+64 B，块 128 KiB = 64 页，1024 块）
 *
 * 分层：
 *   本文件 = 控制器层（寄存器读写、DMA 搬运、NAND 命令序列）+ 框架注册；
 *   chiplab_nand.h = 纯逻辑层（寄存器表、几何、ECC 编解码、BBT、地址换算、
 *   DMA 描述符打包），不依赖 U-Boot，可被宿主机单元测试
 *   board/loongson/chiplab/nand-unittest/ 直接编译。
 *
 * 字节流契约（与 U-Boot RAW NAND 框架的 nand_read_page_raw /
 * nand_read_oob_std 完全对齐）：
 *   cmdfunc(NAND_CMD_READ0) 触发一次「主区 + 备用区」搬运（2112 B），
 *   数据存于 DMA 缓冲区；随后框架按顺序调用 read_buf(writesize) 与
 *   read_buf(oobsize)，本驱动用内部游标顺序吐出，**不重复搬运**。
 *   写侧对称：SEQIN 复位游标，write_buf(writesize) 与 write_buf(oobsize)
 *   依次填入缓冲区，PAGEPROG 一次性提交。
 *
 * 运行期验证状态：本机无硬件、无 qemu ⇒ 只做到**构建级 + 逻辑级**验证；
 * 上板验证项列在 board/loongson/chiplab/README「NAND 驱动」节的遗留清单。
 */

#include <config.h>
#include <cpu_func.h>
#include <errno.h>
#include <init.h>
#include <log.h>
#include <malloc.h>
#include <nand.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/nand.h>
#include <linux/sizes.h>

#include "chiplab_nand.h"

/* 控制器基址：0x1FE7_8000（platform-facts.md §2.5；ls1a_nand.c:86 同口径） */
#define CHIPLAB_NAND_BASE		0x1fe78000UL

/* DMA 缓冲区：32 B 描述符 + 2112 B 数据（32 字节对齐 + 余量） */
#define CHIPLAB_NAND_DMA_BUF_SIZE	4096u
#define CHIPLAB_NAND_DMA_DATA_OFF	CHIPLAB_DMA_DESC_BYTES

#define CHIPLAB_NAND_TIMEOUT_MS		500u

/* 设备状态位（NAND 标准） */
#define CHIPLAB_NAND_STATUS_FAIL	0x01u
#define CHIPLAB_NAND_STATUS_READY	0x40u
#define CHIPLAB_NAND_STATUS_WP		0x80u

struct chiplab_nand_priv {
	void __iomem *base;
	u8 *dma_buf;			/* 描述符 + 数据；无 MMU ⇒ 虚拟 == 物理 */
	ulong dma_buf_phys;
	u32 param;			/* 最近一次读回的 nand_parameter */
	unsigned cursor;		/* read_buf/write_buf 的字节游标 */
	unsigned frame_len;		/* 当前命令帧的总字节数 */
	u8 status;			/* 最近一次读到的状态字节 */
};

static inline u32 cnand_rd(struct chiplab_nand_priv *p, u32 off)
{
	return readl(p->base + off);
}

static inline void cnand_wr(struct chiplab_nand_priv *p, u32 off, u32 v)
{
	writel(v, p->base + off);
}

static struct chiplab_nand_priv *to_priv(struct mtd_info *mtd)
{
	return mtd_to_nand(mtd)->priv;
}

/* ------------------------------------------------------------------ */
/* 底层操作序列                                                        */
/* ------------------------------------------------------------------ */

/*
 * 写列/行地址。
 *   列地址 → 0x04（取低 14 位，nand.v:184）
 *   行地址 → 0x08（取低 25 位，nand.v:185）；控制器再按 nand_size 展开成
 *   地址周期（nand.v:225-282 / 733-757）。
 */
static void cnand_set_addr(struct chiplab_nand_priv *p, u32 page, u32 column)
{
	cnand_wr(p, CHIPLAB_NAND_REG_ADDRL, chiplab_nand_column(column));
	cnand_wr(p, CHIPLAB_NAND_REG_ADDRH, chiplab_nand_row(page));
}

/* 等待控制器 DONE（命令寄存器 bit10，nand.v:635/952/1143） */
static int cnand_wait_done(struct chiplab_nand_priv *p, unsigned timeout_ms)
{
	ulong start = get_timer(0);
	u32 cmd;

	do {
		cmd = cnand_rd(p, CHIPLAB_NAND_REG_CMD);
		if (cmd & CHIPLAB_CMD_DONE)
			return 0;
		udelay(1);
	} while (get_timer(start) < timeout_ms);

	printf("chiplab-nand: timeout waiting for DONE (cmd=%08x)\n", cmd);
	return -ETIMEDOUT;
}

/*
 * 数据搬运：平台 DMA 引擎（chiplab/IP/DMA/dma.v）。
 *
 * 方向语义（按 RTL 核实）：
 *   DMA 引擎对设备侧恒为「经门铃 0x1FE7_8040 读写」；cmd[12]=dma_r_w
 *   只决定 DDR 侧方向（dma.v:572）。读页用 dma_r_w=0（设备→内存），
 *   写页用 1（内存→设备）。
 *
 * 完成判定：order 寄存器 bit3（dma_start）由硬件在取完描述符后清零
 * （confreg_syn.v:329），bit2（ask_valid）在搬运结束后清零
 * （confreg_syn.v:328），两者皆 0 即结束；随后再等控制器 DONE。
 */
static int cnand_dma(struct chiplab_nand_priv *p, unsigned len, int to_device)
{
	u32 *desc = (u32 *)p->dma_buf;
	u32 data_phys = (u32)(p->dma_buf_phys + CHIPLAB_NAND_DMA_DATA_OFF);
	ulong start;
	int ret;

	ret = chiplab_nand_dma_desc_pack(desc, CHIPLAB_DMA_DESC_WORDS, 0,
					 data_phys, CHIPLAB_DMA_NAND_DEV_ADDR,
					 len, to_device);
	if (ret)
		return -EINVAL;

	/* cache 一致性：描述符/数据都必须先落到 DDR，DMA 直接读物理内存 */
	flush_dcache_range(p->dma_buf_phys,
			   p->dma_buf_phys + CHIPLAB_NAND_DMA_BUF_SIZE);

	cnand_wr(p, CHIPLAB_NAND_REG_DMA_ACK, 0);
	writel(chiplab_nand_dma_order_start((u32)p->dma_buf_phys),
	       (void __iomem *)CHIPLAB_DMA_ORDER_REG);

	start = get_timer(0);
	for (;;) {
		u32 order = readl((void __iomem *)CHIPLAB_DMA_ORDER_REG);

		if (!(order & CHIPLAB_DMA_ORDER_START) &&
		    !(order & CHIPLAB_DMA_ORDER_ASK))
			break;
		if (get_timer(start) >= CHIPLAB_NAND_TIMEOUT_MS) {
			printf("chiplab-nand: DMA timeout (order=%08x len=%u)\n",
			       order, len);
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	invalidate_dcache_range(p->dma_buf_phys,
				p->dma_buf_phys + CHIPLAB_NAND_DMA_BUF_SIZE);

	return cnand_wait_done(p, CHIPLAB_NAND_TIMEOUT_MS);
}

/*
 * 页读：主区 + 备用区一次搬运（2112 B）。
 * Linux 侧同口径：ls1a_nand.c:676 `buf_count = oobsize + writesize`，
 * 命令位 op_main | op_spare（nand.v:206-207 的 main_op/spare_op）。
 */
static int cnand_page_read(struct chiplab_nand_priv *p, u32 page)
{
	cnand_set_addr(p, page, 0);
	cnand_wr(p, CHIPLAB_NAND_REG_OP_NUM, CHIPLAB_NAND_PAGE_SPARE);
	cnand_wr(p, CHIPLAB_NAND_REG_CMD,
		 CHIPLAB_CMD_READ | CHIPLAB_CMD_OP_MAIN | CHIPLAB_CMD_OP_SPARE |
		 CHIPLAB_CMD_VALID);
	return cnand_dma(p, CHIPLAB_NAND_PAGE_SPARE, 0);
}

/* 页写：主区 + 备用区一次搬运，命令位含 WRITE（nand.v:494-519） */
static int cnand_page_write(struct chiplab_nand_priv *p, u32 page)
{
	cnand_set_addr(p, page, 0);
	cnand_wr(p, CHIPLAB_NAND_REG_OP_NUM, CHIPLAB_NAND_PAGE_SPARE);
	cnand_wr(p, CHIPLAB_NAND_REG_CMD,
		 CHIPLAB_CMD_WRITE | CHIPLAB_CMD_OP_MAIN | CHIPLAB_CMD_OP_SPARE |
		 CHIPLAB_CMD_VALID);
	return cnand_dma(p, CHIPLAB_NAND_PAGE_SPARE, 1);
}

static int cnand_block_erase(struct chiplab_nand_priv *p, u32 page)
{
	cnand_set_addr(p, page, 0);
	cnand_wr(p, CHIPLAB_NAND_REG_OP_NUM, 1);
	cnand_wr(p, CHIPLAB_NAND_REG_CMD,
		 CHIPLAB_CMD_ERASE | CHIPLAB_CMD_VALID);
	return cnand_wait_done(p, CHIPLAB_NAND_TIMEOUT_MS * 4);
}

/* 读状态字节：0x14 = {status[7:0], ID_INFORM[47:32]}（nand.v:347,1216） */
static int cnand_read_status(struct chiplab_nand_priv *p)
{
	int ret;

	cnand_wr(p, CHIPLAB_NAND_REG_CMD,
		 CHIPLAB_CMD_READ_STATUS | CHIPLAB_CMD_VALID);
	ret = cnand_wait_done(p, CHIPLAB_NAND_TIMEOUT_MS);
	if (ret)
		return ret;
	p->status = (u8)(cnand_rd(p, CHIPLAB_NAND_REG_STATUS_IDH) >> 16);
	return 0;
}

/* ------------------------------------------------------------------ */
/* nand_chip 回调                                                      */
/* ------------------------------------------------------------------ */

static void cnand_cmdfunc(struct mtd_info *mtd, unsigned command, int column,
			  int page_addr)
{
	struct chiplab_nand_priv *p = to_priv(mtd);

	p->cursor = 0;
	p->frame_len = 0;

	switch (command) {
	case NAND_CMD_RESET:
		cnand_wr(p, CHIPLAB_NAND_REG_CMD,
			 CHIPLAB_CMD_RESET | CHIPLAB_CMD_VALID);
		(void)cnand_wait_done(p, CHIPLAB_NAND_TIMEOUT_MS);
		break;

	case NAND_CMD_STATUS:
		p->frame_len = 1;
		(void)cnand_read_status(p);
		break;

	case NAND_CMD_READID:
		/* 控制器把整段 ID 读进 ID_INFORM（nand.v:1332-1396，按 nand_id_num） */
		cnand_wr(p, CHIPLAB_NAND_REG_PARAM,
			 chiplab_nand_param_encode(CHIPLAB_NAND_PAGE_SIZE,
						   CHIPLAB_NAND_ID_BYTES,
						   CHIPLAB_NAND_SIZE_1GBIT));
		cnand_wr(p, CHIPLAB_NAND_REG_CMD,
			 CHIPLAB_CMD_READ_ID | CHIPLAB_CMD_VALID);
		(void)cnand_wait_done(p, CHIPLAB_NAND_TIMEOUT_MS);
		p->frame_len = CHIPLAB_NAND_ID_BYTES;
		break;

	case NAND_CMD_ERASE1:
		/* 控制器自串 0x60 → 地址 → 0xD0 → 0x70（nand.v:1243-1331） */
		(void)cnand_block_erase(p, (u32)page_addr);
		p->frame_len = 1;	/* 让后续读状态拿到 status */
		break;

	case NAND_CMD_ERASE2:
		break;

	case NAND_CMD_READ0:
	case NAND_CMD_READOOB:
	{
		u32 page = (u32)page_addr;

		if (column >= (int)CHIPLAB_NAND_PAGE_SIZE)
			page += (u32)(column / CHIPLAB_NAND_PAGE_SIZE);
		if (cnand_page_read(p, page)) {
			/* 失败时也把 frame 置满，避免框架读到越界数据 */
			p->cursor = CHIPLAB_NAND_PAGE_SPARE;
			p->frame_len = CHIPLAB_NAND_PAGE_SPARE;
			break;
		}
		p->frame_len = CHIPLAB_NAND_PAGE_SPARE;
		break;
	}

	case NAND_CMD_SEQIN:
		/* 清空缓冲区，等待 write_buf 填入（PAGEPROG 才真正提交） */
		memset(p->dma_buf + CHIPLAB_NAND_DMA_DATA_OFF, 0xff,
		       CHIPLAB_NAND_PAGE_SPARE);
		p->frame_len = CHIPLAB_NAND_PAGE_SPARE;
		break;

	case NAND_CMD_PAGEPROG:
		(void)cnand_page_write(p, (u32)page_addr);
		break;

	case NAND_CMD_READ1:
	case NAND_CMD_RNDOUT:
		break;

	default:
		debug("chiplab-nand: unsupported command 0x%x\n", command);
		break;
	}
}

static int cnand_dev_ready(struct mtd_info *mtd)
{
	struct chiplab_nand_priv *p = to_priv(mtd);

	return !!(cnand_rd(p, CHIPLAB_NAND_REG_CMD) & CHIPLAB_CMD_DONE);
}

/*
 * nand_wait() 的返回值语义（见 nand_base.c 的 waitfunc 用法）：
 *   负值 → 超时/错误；0 → 成功；非 0 → 位翻转计数或失败。
 * 这里返回状态字节本身（0 = 失败、0x40/0xC0 = 就绪），与多数驱动一致。
 */
static int cnand_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct chiplab_nand_priv *p = to_priv(mtd);
	int ret;

	ret = cnand_wait_done(p, CHIPLAB_NAND_TIMEOUT_MS * 2);
	if (ret)
		return ret;
	return p->status;
}

static uint8_t cnand_read_byte(struct mtd_info *mtd)
{
	struct chiplab_nand_priv *p = to_priv(mtd);
	u8 v = 0xff;

	switch (p->frame_len) {
	case 1:		/* NAND_CMD_STATUS */
		v = p->status;
		break;
	case CHIPLAB_NAND_ID_BYTES:	/* NAND_CMD_READID：ID_INFORM 低 5 字节 */
	{
		u32 idl = cnand_rd(p, CHIPLAB_NAND_REG_IDL);
		u32 sth = cnand_rd(p, CHIPLAB_NAND_REG_STATUS_IDH);

		/* ID_INFORM[39:0]：IDL[39:8] 不可得，故按 HIT4/HIT5 拼接 */
		switch (p->cursor) {
		case 0:
			v = (u8)(idl >> 0);
			break;
		case 1:
			v = (u8)(idl >> 8);
			break;
		case 2:
			v = (u8)(idl >> 16);
			break;
		case 3:
			v = (u8)(idl >> 24);
			break;
		default:
			v = (u8)(sth >> (((p->cursor - 4) * 8) + 8));
			break;
		}
		break;
	}
	default:
		break;
	}
	p->cursor++;
	return v;
}

static void cnand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct chiplab_nand_priv *p = to_priv(mtd);
	unsigned avail;

	if (p->cursor >= p->frame_len) {
		memset(buf, 0xff, (unsigned)len);
		return;
	}
	avail = p->frame_len - p->cursor;
	if ((unsigned)len > avail)
		len = (int)avail;
	memcpy(buf, p->dma_buf + CHIPLAB_NAND_DMA_DATA_OFF + p->cursor, (unsigned)len);
	p->cursor += (unsigned)len;
}

static void cnand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct chiplab_nand_priv *p = to_priv(mtd);
	unsigned room = CHIPLAB_NAND_PAGE_SPARE - p->cursor;

	if ((unsigned)len > room)
		len = (int)room;
	if (len > 0) {
		memcpy(p->dma_buf + CHIPLAB_NAND_DMA_DATA_OFF + p->cursor, buf,
		       (unsigned)len);
		p->cursor += (unsigned)len;
	}
}

/* 本驱动的 BCH 上下文（表由 chiplab_bch_init() 在 probe 时构造） */
static struct chiplab_bch chiplab_ecc_bch;

/*
 * ECC 计算/纠正钩子。框架调用约定（nand_read_page_swecc / nand_write_page_swecc）：
 *   calculate(dat[512], ecc_code[7])
 *   correct(dat[512], read_ecc[7], calc_ecc[7])
 * 与 chiplab_nand.h 的 chiplab_bch_encode / chiplab_bch_decode 一一对应。
 */
static int cnand_ecc_calculate(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	return chiplab_bch_encode(&chiplab_ecc_bch, dat, chip->ecc.size,
				  ecc_code, CHIPLAB_NAND_ECC_BYTES_PER_STEP);
}

static int cnand_ecc_correct(struct mtd_info *mtd, u_char *dat, u_char *read_ecc,
			     u_char *calc_ecc)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	int ret;

	/* 翻转位数由解码器返回；负值表示超出纠正能力（fail-closed） */
	ret = chiplab_bch_decode(&chiplab_ecc_bch, dat, chip->ecc.size,
				 read_ecc, CHIPLAB_NAND_ECC_BYTES_PER_STEP);
	if (ret < 0) {
		debug("chiplab-nand: ECC uncorrectable (%d)\n", ret);
		return ret;
	}
	return ret;	/* 0..t：纠正的 bit 数 */
}

static void cnand_select_chip(struct mtd_info *mtd, int chip)
{
	(void)mtd;
	(void)chip;
}

/* ------------------------------------------------------------------ */
/* 探测与注册                                                          */
/* ------------------------------------------------------------------ */

/*
 * 实读器件 ID 与几何参数（04-nand-driver.md §3.2 的验收第 1 项）。
 * 字节拼接口径见 chiplab_nand.h 寄存器表：
 *   HIT4 = ID_INFORM[31:0]，HIT5 = {status[7:0], ID_INFORM[47:32]}
 */
static void cnand_print_ids(struct chiplab_nand_priv *p)
{
	u32 idl, sth, param;
	u8 id[6];

	cnand_wr(p, CHIPLAB_NAND_REG_CMD,
		 CHIPLAB_CMD_READ_ID | CHIPLAB_CMD_VALID);
	if (cnand_wait_done(p, CHIPLAB_NAND_TIMEOUT_MS)) {
		printf("chiplab-nand: READ_ID timeout\n");
		return;
	}

	idl = cnand_rd(p, CHIPLAB_NAND_REG_IDL);
	sth = cnand_rd(p, CHIPLAB_NAND_REG_STATUS_IDH);
	param = cnand_rd(p, CHIPLAB_NAND_REG_PARAM);
	p->param = param;

	id[0] = (u8)(idl >> 0 * 8);
	id[1] = (u8)(idl >> 1 * 8);
	id[2] = (u8)(idl >> 2 * 8);
	id[3] = (u8)(idl >> 3 * 8);
	id[4] = (u8)(sth >> 1 * 8);
	id[5] = (u8)(sth >> 2 * 8);

	printf("chiplab-nand: ID_INFORM = %02x %02x %02x %02x %02x %02x "
	       "(expect %02x %02x ..)\n",
	       id[0], id[1], id[2], id[3], id[4], id[5],
	       CHIPLAB_NAND_MFR_ID, CHIPLAB_NAND_DEV_ID);
	printf("chiplab-nand: IDL=%08x STATUS_IDH=%08x PARAM=%08x "
	       "(op_scope=%u id_num=%u size=%u)\n",
	       idl, sth, param,
	       chiplab_nand_param_op_scope(param),
	       chiplab_nand_param_id_num(param),
	       chiplab_nand_param_size(param));
}

/* 与 04-nand-driver.md §1.2 的裁决口径对账（关掉几何矛盾那条不确定项） */
static int cnand_check_geometry(struct mtd_info *mtd)
{
	if (mtd->writesize != CHIPLAB_NAND_PAGE_SIZE) {
		printf("chiplab-nand: writesize=%u, expect %u\n",
		       mtd->writesize, CHIPLAB_NAND_PAGE_SIZE);
		return -EINVAL;
	}
	if (mtd->oobsize != CHIPLAB_NAND_OOBSIZE) {
		printf("chiplab-nand: oobsize=%u, expect %u\n",
		       mtd->oobsize, CHIPLAB_NAND_OOBSIZE);
		return -EINVAL;
	}
	if (mtd->erasesize != CHIPLAB_NAND_BLOCK_SIZE) {
		printf("chiplab-nand: erasesize=%u, expect %u\n",
		       mtd->erasesize, CHIPLAB_NAND_BLOCK_SIZE);
		return -EINVAL;
	}
	if (mtd->size != (u64)CHIPLAB_NAND_TOTAL_SIZE) {
		printf("chiplab-nand: size=%llu, expect %u\n",
		       (unsigned long long)mtd->size, CHIPLAB_NAND_TOTAL_SIZE);
		return -EINVAL;
	}
	return 0;
}

void board_nand_init(void)
{
	struct mtd_info *mtd;
	struct nand_chip *chip;
	struct chiplab_nand_priv *p;
	int ret;

	p = calloc(1, sizeof(*p));
	if (!p) {
		printf("chiplab-nand: out of memory\n");
		return;
	}
	chip = calloc(1, sizeof(*chip));
	if (!chip)
		goto err_p;
	mtd = calloc(1, sizeof(*mtd));
	if (!mtd)
		goto err_chip;

	p->base = map_physmem(CHIPLAB_NAND_BASE, SZ_4K, MAP_NOCACHE);
	if (!p->base) {
		printf("chiplab-nand: cannot map controller @%lx\n",
		       CHIPLAB_NAND_BASE);
		goto err_mtd;
	}
	p->dma_buf = memalign(CHIPLAB_DMA_ORDER_ALIGN, CHIPLAB_NAND_DMA_BUF_SIZE);
	if (!p->dma_buf) {
		printf("chiplab-nand: cannot allocate DMA buffer\n");
		goto err_mtd;
	}
	p->dma_buf_phys = (ulong)p->dma_buf;	/* 无 MMU：虚拟 == 物理 */

	/* --- nand_chip 接线 --- */
	chip->priv = p;
	nand_set_controller_data(chip, p);
	chip->IO_ADDR_R = p->base;
	chip->IO_ADDR_W = p->base;

	chip->cmdfunc = cnand_cmdfunc;
	chip->waitfunc = cnand_waitfunc;
	chip->dev_ready = cnand_dev_ready;
	chip->select_chip = cnand_select_chip;
	chip->read_byte = cnand_read_byte;
	chip->read_buf = cnand_read_buf;
	chip->write_buf = cnand_write_buf;
	chip->chip_delay = 20;
	chip->options = NAND_NO_SUBPAGE_WRITE;
	chip->bbt_options = NAND_BBT_USE_FLASH;
	chip->badblockpos = (int)CHIPLAB_NAND_BBT_FACTORY_OFF;

	/* --- 控制器初始化（寄存器口径见 chiplab_nand.h 顶部全表） --- */
	/* 时序：RTL 对写值有下限钳位（t[7:0] ≥ 5、t[15:8] ≥ 2，nand.v:186-189） */
	cnand_wr(p, CHIPLAB_NAND_REG_TIMING, CHIPLAB_NAND_TIMING_RESET);
	/* op_scope = 页容量（nand.v:203）、id_num = ID 字节数（:204）、
	 * size = 1 Gbit 几何编码（nand.v:160 缺省 0x0800_5000 → size=0；
	 * Linux 侧写 0x08005300 → size=3，ls1a_nand.c:910）。 */
	cnand_wr(p, CHIPLAB_NAND_REG_PARAM,
		 chiplab_nand_param_encode(CHIPLAB_NAND_PAGE_SIZE,
					   CHIPLAB_NAND_ID_BYTES,
					   CHIPLAB_NAND_SIZE_1GBIT));
	/* CE/RDY 映射：Linux 同值（ls1a_nand.c:925） */
	cnand_wr(p, CHIPLAB_NAND_REG_CE_MAP0, 0x88442200u);
	cnand_wr(p, CHIPLAB_NAND_REG_RDY_MAP0, 0x88442200u);

	cnand_print_ids(p);

	mtd->name = "chiplab-nand";
	mtd->priv = chip;

	/*
	 * ECC：控制器**无硬件 ECC**（04-nand-driver.md §4.2 的只读分析结论：
	 * nand.v 全文无 ECC 引擎），故用软件 BCH，4 bit / 512 B 段、4 段/页，
	 * 布局常量见 chiplab_nand.h §3（备用区尾部 28 B；标记在 offset 0/1）。
	 */
	/*
	 * ECC：控制器**无硬件 ECC**（04-nand-driver.md §4.2 的只读分析结论：
	 * nand.v 全文无 ECC 引擎）。这里用 NAND_ECC_SOFT 的通用软件 ECC 框架
	 * （nand_read_page_swecc / nand_write_page_swecc），但把 calculate/correct
	 * 换成 chiplab_nand.h 里的自包含 BCH —— 这样 ECC 布局与位序由**同一份
	 * 头文件**唯一定义，将来 Linux 侧照搬即可，不受框架默认布局影响。
	 * 布局：4 段 × 7 B = 28 B，由 nand_bch_init() 自动放在备用区尾部
	 * （offset 36..63，备用区 64 B），坏块标记在 offset 0/1，互不重叠。
	 */
	if (chiplab_bch_init(&chiplab_ecc_bch)) {
		printf("chiplab-nand: BCH init failed\n");
		goto err_buf;
	}
	chip->ecc.mode = NAND_ECC_SOFT;
	chip->ecc.size = CHIPLAB_NAND_ECC_STEP_SIZE;
	chip->ecc.steps = CHIPLAB_NAND_ECC_STEPS;
	chip->ecc.bytes = CHIPLAB_NAND_ECC_BYTES_PER_STEP;
	chip->ecc.strength = CONFIG_NAND_CHIPLAB_ECC_STRENGTH;
	chip->ecc.calculate = cnand_ecc_calculate;
	chip->ecc.correct = cnand_ecc_correct;

	ret = nand_scan(mtd, 1);
	if (ret) {
		printf("chiplab-nand: nand_scan failed (%d)\n", ret);
		goto err_buf;
	}
	if (cnand_check_geometry(mtd)) {
		nand_unregister(mtd);
		goto err_buf;
	}

	nand_register(0, mtd);
	printf("chiplab-nand: registered %llu MiB, page %u+%u B, erase %u B, ECC BCH-%d/%uB\n",
	       (unsigned long long)mtd->size / (1024 * 1024),
	       mtd->writesize, mtd->oobsize, mtd->erasesize,
	       chip->ecc.strength, chip->ecc.size);
	return;

err_buf:
	free(p->dma_buf);
err_mtd:
	free(mtd);
err_chip:
	free(chip);
err_p:
	free(p);
}
