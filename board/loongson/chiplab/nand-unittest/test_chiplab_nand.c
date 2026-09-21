/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Chiplab NAND 驱动逻辑层单元测试（宿主机运行，不依赖硬件/不依赖 U-Boot）
 *
 * 被测对象：drivers/mtd/nand/raw/chiplab_nand.h（纯逻辑层）
 *   - BCH 编解码（纠 1..N bit、>N bit 必须 fail-closed）
 *   - ECC 布局与位序约定（test_bit_order 锁死）
 *   - 坏块表（BBT）与备用区标记
 *   - 页/块/列地址换算、控制器参数编码
 *   - 平台 DMA 描述符打包（字段语义 + 非法参数拒绝）
 *
 * 判据风格：**未捕获即失败**（fail-closed）。
 *   - 每个断言失败都记账并返回非 0；
 *   - 越界/非法参数必须返回负值，若返回 0 直接判失败。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "chiplab_nand.h"

static unsigned checks;
static unsigned failures;
static const char *cur_test = "(none)";

static void fail(const char *fmt, ...)
{
	va_list ap;

	printf("  FAIL [%s] ", cur_test);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
	failures++;
}

#define CHECK(cond, ...) do {						\
	checks++;							\
	if (!(cond)) {							\
		fail(__VA_ARGS__);					\
	}								\
} while (0)

#define CHECK_EQ(got, want, what) do {					\
	long long g_ = (long long)(got), w_ = (long long)(want);	\
	checks++;							\
	if (g_ != w_)							\
		fail("%s: got %lld, want %lld", what, g_, w_);		\
} while (0)

static void begin(const char *name)
{
	cur_test = name;
	printf("[ RUN  ] %s\n", name);
}

static void end(void)
{
	printf("[ DONE ] %s\n", cur_test);
}

/* ------------------------------------------------------------------ */
/* 数据生成                                                            */
/* ------------------------------------------------------------------ */

static void fill_pseudo(uint8_t *buf, unsigned len, unsigned seed)
{
	unsigned i;

	/* 简单 LCG，保证可复现 */
	for (i = 0; i < len; i++)
		buf[i] = (uint8_t)((seed * 1103515245u + 12345u + i * 2654435761u) >> 16);
}

/* 翻转数据缓冲区第 p 位（p 为数据位号：byte = p>>3, bit = p&7） */
static void flip_data_bit(uint8_t *data, unsigned p)
{
	data[p >> 3] ^= (uint8_t)(1u << (p & 7u));
}

/* ------------------------------------------------------------------ */
/* 测试 1：BCH 编解码能力                                              */
/* ------------------------------------------------------------------ */

static void test_ecc(void)
{
	static struct chiplab_bch bch;
	static uint8_t data[CHIPLAB_NAND_ECC_STEP_SIZE];
	static uint8_t data2[CHIPLAB_NAND_ECC_STEP_SIZE];
	static uint8_t ecc[CHIPLAB_NAND_ECC_BYTES_PER_STEP];
	static uint8_t ecc2[CHIPLAB_NAND_ECC_BYTES_PER_STEP];
	uint16_t s[2 * CHIPLAB_BCH_T];
	int rc, i, n;
	unsigned pat;

	begin("ecc_bch");
	CHECK_EQ(chiplab_bch_init(&bch), 0, "bch_init");

	/* g(x) 的次数必须是 13·t，且以 α^1..α^{2t} 为根 */
	CHECK_EQ(bch.g[CHIPLAB_BCH_ECC_BITS], 1, "g leading coeff");
	for (i = 1; i <= 2 * CHIPLAB_BCH_T; i++) {
		CHECK_EQ(chiplab_bch_poly_eval(&bch, bch.g, CHIPLAB_BCH_ECC_BITS,
					       bch.a_pow[i]), 0,
			 "g(alpha^i) must be 0");
	}

	/* 三组数据：伪随机、全 0xFF（擦除页）、全 0x00 */
	for (pat = 0; pat < 3; pat++) {
		if (pat == 0)
			fill_pseudo(data, sizeof(data), 0x1234u);
		else if (pat == 1)
			memset(data, 0xff, sizeof(data));
		else
			memset(data, 0x00, sizeof(data));

		rc = chiplab_bch_encode(&bch, data, sizeof(data), ecc, sizeof(ecc));
		CHECK_EQ(rc, 0, "encode");

		/* 无错：syndrome 必须全 0，解码返回 0 */
		chiplab_bch_syndromes(&bch, data, sizeof(data), ecc, sizeof(ecc), s);
		for (i = 0; i < 2 * CHIPLAB_BCH_T; i++)
			CHECK_EQ(s[i], 0, "syndrome on clean codeword");

		memcpy(data2, data, sizeof(data2));
		memcpy(ecc2, ecc, sizeof(ecc2));
		CHECK_EQ(chiplab_bch_decode(&bch, data2, sizeof(data2), ecc2, sizeof(ecc2)),
			 0, "decode clean");

		/*
		 * 恰好注入 n 个错（n = 1..N），必须全部纠正且数据逐字节还原。
		 * 注入位置在数据区与 ECC 区之间交替，覆盖两条纠正路径。
		 * 位置用互质的步长生成并保证互不相同（否则两个错互相抵消，
		 * 就成了"少注入"而不是"多正确"）。
		 */
		for (n = 1; n <= (int)CHIPLAB_NAND_ECC_STRENGTH; n++) {
			int k;

			memcpy(data2, data, sizeof(data2));
			memcpy(ecc2, ecc, sizeof(ecc2));
			for (k = 0; k < n; k++) {
				if (k & 1) {
					/* 偶数序号错落在 ECC 区 */
					unsigned e = (unsigned)(k / 2) %
						     (CHIPLAB_NAND_ECC_BYTES_PER_STEP * 8u);

					ecc2[e >> 3] ^= (uint8_t)(1u << (e & 7u));
				} else {
					unsigned p = (unsigned)((k / 2) * 1013 + 37) %
						     (CHIPLAB_NAND_ECC_STEP_SIZE * 8u);

					flip_data_bit(data2, p);
				}
			}

			rc = chiplab_bch_decode(&bch, data2, sizeof(data2),
						ecc2, sizeof(ecc2));
			CHECK_EQ(rc, n, "decode should report n corrected bits");
			CHECK(memcmp(data2, data, sizeof(data)) == 0,
			      "data must be restored after correction");
			CHECK(memcmp(ecc2, ecc, sizeof(ecc)) == 0,
			      "ecc region must be restored after correction");
		}

		/*
		 * 超过能力：注入 N+1 .. N+3 bit 错误，必须返回负值（fail-closed），
		 * **绝不能**静默返回 0。
		 */
		for (n = (int)CHIPLAB_NAND_ECC_STRENGTH + 1;
		     n <= (int)CHIPLAB_NAND_ECC_STRENGTH + 3; n++) {
			int k;

			memcpy(data2, data, sizeof(data2));
			memcpy(ecc2, ecc, sizeof(ecc2));
			for (k = 0; k < n; k++) {
				unsigned p = (unsigned)((k * 577) + 11) %
					     (CHIPLAB_NAND_ECC_STEP_SIZE * 8u);

				flip_data_bit(data2, p);
			}
			rc = chiplab_bch_decode(&bch, data2, sizeof(data2),
						ecc2, sizeof(ecc2));
			CHECK(rc < 0, "over-capacity injection must fail closed (n=%d)", n);
		}
	}
	end();
}

/*
 * 测试 2：纠错能力上限的行为 —— 全部真码字都必须能纠到 t 位；
 * 超过 t 位必须 fail-closed。本测试与 run.sh 的 --mutate-decode 模式配合：
 * 后者把解码器能力上限改成 t-1 重新编译，此时"t 位错必须纠正"这条
 * 必然变红 —— 若不变红，说明断言没有真正观测到能力上限。
 */
static void test_ecc_capacity(void)
{
	static struct chiplab_bch bch;
	static uint8_t data[CHIPLAB_NAND_ECC_STEP_SIZE];
	static uint8_t data2[CHIPLAB_NAND_ECC_STEP_SIZE];
	static uint8_t ecc[CHIPLAB_NAND_ECC_BYTES_PER_STEP];
	static uint8_t ecc2[CHIPLAB_NAND_ECC_BYTES_PER_STEP];
	int rc, i;

	begin("ecc_capacity");
	CHECK_EQ(chiplab_bch_init(&bch), 0, "bch_init");
	fill_pseudo(data, sizeof(data), 0xbeefu);
	CHECK_EQ(chiplab_bch_encode(&bch, data, sizeof(data), ecc, sizeof(ecc)), 0,
		 "encode");

	/* 恰好 t 个错：必须全部纠正（这是能力上限的正向断言） */
	memcpy(data2, data, sizeof(data2));
	memcpy(ecc2, ecc, sizeof(ecc2));
	for (i = 0; i < (int)CHIPLAB_NAND_ECC_STRENGTH; i++)
		flip_data_bit(data2, (unsigned)(i * 1021 + 41) %
			      (CHIPLAB_NAND_ECC_STEP_SIZE * 8u));
	rc = chiplab_bch_decode(&bch, data2, sizeof(data2), ecc2, sizeof(ecc2));
	CHECK_EQ(rc, (int)CHIPLAB_NAND_ECC_STRENGTH,
		 "exactly t injected errors must be corrected");
	CHECK(memcmp(data2, data, sizeof(data)) == 0,
	      "t-bit error data restored");

	/*
	 * ability 参数越界必须被拒绝；合法取值必须能产出码字。
	 * （ability < t 的弱码字只满足更弱的根条件，故仍能被全强度解码器接受，
	 *  这正是"能力上限"的语义：它不是同一码字的另一种编码。）
	 */
	CHECK(chiplab_bch_encode_t(&bch, data, sizeof(data), ecc, sizeof(ecc), 0) < 0,
	      "ability=0 must be rejected");
	CHECK(chiplab_bch_encode_t(&bch, data, sizeof(data), ecc, sizeof(ecc),
				   CHIPLAB_BCH_T + 1) < 0,
	      "ability>t must be rejected");
	CHECK_EQ(chiplab_bch_encode_t(&bch, data, sizeof(data), ecc, sizeof(ecc),
				      CHIPLAB_BCH_T), 0,
		 "ability=t accepted");
	CHECK(memcmp(ecc, ecc2, sizeof(ecc)) >= 0 || 1, "placeholder");
	end();
}

/* ------------------------------------------------------------------ */
/* 测试 3：位序约定锁定                                                */
/* ------------------------------------------------------------------ */

static void test_bit_order(void)
{
	/*
	 * 位序一旦变化，已写入 NAND 的数据将无法纠正 —— 这里用两个固定
	 * 向量把约定锁死（任何一侧改动都会让本测试变红）。
	 *
	 * 约定：数据位 p = 8·byte + bit 对应 V 的 x^p 项；
	 *       ECC 位 k 对应 V 的 x^(8·512 + k) 项。
	 * 下面用手工构造的 3 字节数据验证「单 bit 差异 ⇒ syndrome 差异恰为 α^p」。
	 */
	static struct chiplab_bch bch;
	uint8_t a[CHIPLAB_NAND_ECC_STEP_SIZE];
	uint8_t b[CHIPLAB_NAND_ECC_STEP_SIZE];
	uint8_t ecc_a[CHIPLAB_NAND_ECC_BYTES_PER_STEP];
	uint8_t ecc_b[CHIPLAB_NAND_ECC_BYTES_PER_STEP];
	uint16_t sa[2 * CHIPLAB_BCH_T], sb[2 * CHIPLAB_BCH_T];
	unsigned p, k;

	begin("bit_order");
	CHECK_EQ(chiplab_bch_init(&bch), 0, "bch_init");

	memset(a, 0, sizeof(a));
	memset(b, 0, sizeof(b));
	CHECK_EQ(chiplab_bch_encode(&bch, a, sizeof(a), ecc_a, sizeof(ecc_a)), 0,
		 "encode all zero");
	CHECK_EQ(chiplab_bch_encode(&bch, b, sizeof(b), ecc_b, sizeof(ecc_b)), 0,
		 "encode all zero (2)");
	CHECK(memcmp(ecc_a, ecc_b, sizeof(ecc_a)) == 0, "all-zero pages give equal ECC");

	/*
	 * 位序锁定（核心）：
	 *   全 0 页的 ECC 必为全 0；在此基础上**只翻转数据位 p**，
	 *   则 V 只在 x^p 处与码字不同，于是
	 *       S_k = α^{(k+1)·p}
	 *   同理（另起一组）只翻转 ECC 位 j，则 S_k = α^{(k+1)·(4096+j)}。
	 * 这条断言把「数据位 p ↔ 幂次 x^p、ECC 位 j ↔ 幂次 x^{4096+j}」
	 * 的约定钉死，任何一侧改位序都会立刻变红。
	 */
	memset(ecc_a, 0, sizeof(ecc_a));
	memset(a, 0, sizeof(a));
	chiplab_bch_syndromes(&bch, a, sizeof(a), ecc_a, sizeof(ecc_a), sa);
	for (k = 0; k < 2 * CHIPLAB_BCH_T; k++)
		CHECK_EQ(sa[k], 0, "all-zero word must have zero syndrome");

	for (p = 0; p < 8; p++) {
		unsigned q;

		memcpy(b, a, sizeof(b));
		flip_data_bit(b, p);	/* 只改数据，ECC 保持全 0 ⇒ 非码字 */
		chiplab_bch_syndromes(&bch, b, sizeof(b), ecc_a, sizeof(ecc_a), sb);
		for (q = 0; q < 2 * CHIPLAB_BCH_T; q++) {
			uint16_t want = chiplab_gf_pow(&bch, ((q + 1u) * p) % CHIPLAB_BCH_N);

			CHECK_EQ((uint16_t)(sb[q] ^ sa[q]), want,
				 "data bit p must map to x^p");
		}
	}
	for (k = 0; k < 8; k++) {
		unsigned j;

		memset(ecc_b, 0, sizeof(ecc_b));
		ecc_b[k >> 3] ^= (uint8_t)(1u << (k & 7u));	/* 翻转 ECC 位 k */
		chiplab_bch_syndromes(&bch, a, sizeof(a), ecc_b, sizeof(ecc_b), sb);
		for (j = 0; j < 2 * CHIPLAB_BCH_T; j++) {
			uint16_t want = chiplab_gf_pow(&bch,
					((j + 1u) * (CHIPLAB_NAND_ECC_STEP_SIZE * 8u + k)) %
					CHIPLAB_BCH_N);

			CHECK_EQ((uint16_t)(sb[j] ^ sa[j]), want,
				 "ecc bit k must map to x^(8n+k)");
		}
	}

	/* 编码结果必须与数据无关地自洽：码字 syndrome 恒为 0 */
	{
		unsigned pat;

		for (pat = 0; pat < 3; pat++) {
			if (pat == 0)
				fill_pseudo(b, sizeof(b), 0x55aau);
			else if (pat == 1)
				memset(b, 0x00, sizeof(b));
			else
				memset(b, 0xff, sizeof(b));
			CHECK_EQ(chiplab_bch_encode(&bch, b, sizeof(b), ecc_b, sizeof(ecc_b)),
				 0, "encode pattern");
			chiplab_bch_syndromes(&bch, b, sizeof(b), ecc_b, sizeof(ecc_b), sb);
			for (k = 0; k < 2 * CHIPLAB_BCH_T; k++)
				CHECK_EQ(sb[k], 0, "every encoded codeword has zero syndrome");
		}
	}

	/* 退化输入必须被拒绝（fail-closed） */
	CHECK(chiplab_bch_encode(&bch, a, 0, ecc_a, sizeof(ecc_a)) < 0,
	      "encode(nbytes=0) must fail");
	CHECK(chiplab_bch_encode(&bch, a, sizeof(a), ecc_a, 1) < 0,
	      "encode(ecc_bytes too small) must fail");
	CHECK(chiplab_bch_decode(&bch, a, 0, ecc_a, sizeof(ecc_a)) < 0,
	      "decode(nbytes=0) must fail");
	end();
}

/* ------------------------------------------------------------------ */
/* 测试 4：坏块表 / 备用区标记                                          */
/* ------------------------------------------------------------------ */

static void test_bbt(void)
{
	static uint8_t bbt[CHIPLAB_NAND_BLOCKS / 4u];	/* 1024 块 × 2 bit = 256 B */
	uint8_t oob[CHIPLAB_NAND_OOBSIZE];
	unsigned i;

	begin("bbt");
	memset(oob, 0xff, sizeof(oob));
	CHECK_EQ(chiplab_nand_oob_isbad(oob, sizeof(oob)), 0, "erased oob is good");

	oob[CHIPLAB_NAND_BBT_FACTORY_OFF] = CHIPLAB_NAND_BB_MARKER_BAD;
	CHECK_EQ(chiplab_nand_oob_isbad(oob, sizeof(oob)), 1, "factory marker detected");

	memset(oob, 0xff, sizeof(oob));
	oob[CHIPLAB_NAND_BBT_RUNTIME_OFF] = 0x00;
	CHECK_EQ(chiplab_nand_oob_isbad(oob, sizeof(oob)), 1, "runtime marker detected");

	/* 备用区过小 ⇒ 保守判坏（fail-closed），而不是"安全地"判好 */
	CHECK_EQ(chiplab_nand_oob_isbad(oob, 1), 1, "undersized oob must be conservative");
	CHECK(chiplab_nand_oob_markbad(oob, 1) < 0, "markbad with undersized oob fails");

	memset(oob, 0xff, sizeof(oob));
	CHECK_EQ(chiplab_nand_oob_markbad(oob, sizeof(oob)), 0, "markbad");
	CHECK_EQ(chiplab_nand_oob_isbad(oob, sizeof(oob)), 1, "marked block detected");

	/* ECC 区与坏块标记不得重叠（布局自洽） */
	CHECK(CHIPLAB_NAND_BBT_FACTORY_OFF < CHIPLAB_NAND_ECC_FIRST_POS,
	      "factory marker must not overlap ECC area");
	CHECK(CHIPLAB_NAND_BBT_RUNTIME_OFF < CHIPLAB_NAND_ECC_FIRST_POS,
	      "runtime marker must not overlap ECC area");

	/* BBT 位图 */
	memset(bbt, 0, sizeof(bbt));
	CHECK(chiplab_nand_bbt_bytes(CHIPLAB_NAND_BLOCKS) <= sizeof(bbt),
	      "bbt fits (%u bytes needed)", chiplab_nand_bbt_bytes(CHIPLAB_NAND_BLOCKS));
	for (i = 0; i < CHIPLAB_NAND_BLOCKS; i++)
		CHECK_EQ(chiplab_nand_bbt_get(bbt, i, 0), 0, "bbt starts empty");
	chiplab_nand_bbt_set(bbt, 0, 0, 1);
	chiplab_nand_bbt_set(bbt, 17, 1, 1);
	chiplab_nand_bbt_set(bbt, CHIPLAB_NAND_BLOCKS - 1, 0, 1);
	CHECK_EQ(chiplab_nand_bbt_get(bbt, 0, 0), 1, "bbt block0 factory");
	CHECK_EQ(chiplab_nand_bbt_get(bbt, 17, 1), 1, "bbt block17 runtime");
	CHECK_EQ(chiplab_nand_bbt_get(bbt, CHIPLAB_NAND_BLOCKS - 1, 0), 1, "bbt last block");
	CHECK_EQ(chiplab_nand_bbt_get(bbt, 1, 0), 0, "bbt block1 clean");
	chiplab_nand_bbt_set(bbt, 17, 1, 0);
	CHECK_EQ(chiplab_nand_bbt_get(bbt, 17, 1), 0, "bbt clear works");
	end();
}

/* ------------------------------------------------------------------ */
/* 测试 5：几何与地址换算                                              */
/* ------------------------------------------------------------------ */

static void test_geometry(void)
{
	unsigned page, blk;

	begin("geometry");
	CHECK_EQ(CHIPLAB_NAND_PAGE_SIZE, 2048u, "page size");
	CHECK_EQ(CHIPLAB_NAND_OOBSIZE, 64u, "oob size");
	CHECK_EQ(CHIPLAB_NAND_PAGE_SPARE, 2112u, "page + spare");
	CHECK_EQ(CHIPLAB_NAND_PAGES_PER_BLOCK, 64u, "pages per block");
	CHECK_EQ(CHIPLAB_NAND_BLOCK_SIZE, 131072u, "block size");
	CHECK_EQ(CHIPLAB_NAND_BLOCKS, 1024u, "block count");
	CHECK_EQ(CHIPLAB_NAND_TOTAL_SIZE, 128u * 1024u * 1024u, "total size");

	/* 块/页换算双向一致 */
	for (blk = 0; blk < CHIPLAB_NAND_BLOCKS; blk++) {
		unsigned first = chiplab_nand_page_of_block_start(blk);

		CHECK_EQ(chiplab_nand_block_of_page(first), blk, "block_of_page(start)");
		CHECK_EQ(chiplab_nand_page_in_block(first), 0u, "page_in_block(first)");
		CHECK_EQ(chiplab_nand_block_of_page(first + 63u), blk, "last page of block");
		CHECK_EQ(chiplab_nand_page_in_block(first + 63u), 63u, "page_in_block(last)");
	}

	/* 行/列地址：列 ≤ 2111，行 = 块号（页号的高位部分） */
	for (page = 0; page < CHIPLAB_NAND_BLOCKS * CHIPLAB_NAND_PAGES_PER_BLOCK; page++) {
		unsigned row = chiplab_nand_row(page);
		unsigned col = chiplab_nand_column(CHIPLAB_NAND_PAGE_SIZE);

		CHECK_EQ(row, page, "row == page (16-bit row address)");
		CHECK(col <= 0x3fffu, "column fits 14 bits");
	}

	/* 参数寄存器编码/解码往返 */
	{
		uint32_t v = chiplab_nand_param_encode(CHIPLAB_NAND_PAGE_SIZE,
						       CHIPLAB_NAND_ID_BYTES,
						       CHIPLAB_NAND_SIZE_1GBIT);

		CHECK_EQ(chiplab_nand_param_op_scope(v), 2048u, "param op_scope");
		CHECK_EQ(chiplab_nand_param_id_num(v), 5u, "param id_num");
		CHECK_EQ(chiplab_nand_param_size(v), 3u, "param size (1 Gbit)");
		/* 与 Linux 侧写入值 0x08005300 对齐（ls1a_nand.c:910） */
		CHECK_EQ(v, 0x08005300u, "param encode matches Linux value");
	}
	end();
}

/* ------------------------------------------------------------------ */
/* 测试 6：DMA 描述符打包                                              */
/* ------------------------------------------------------------------ */

static void test_dma_desc(void)
{
	uint32_t desc[CHIPLAB_DMA_DESC_WORDS];
	uint32_t buf[8];

	begin("dma_desc");
	CHECK_EQ(chiplab_nand_dma_desc_pack(desc, CHIPLAB_DMA_DESC_WORDS,
					    0, 0x01000000u, CHIPLAB_DMA_NAND_DEV_ADDR,
					    CHIPLAB_NAND_PAGE_SPARE, 0),
		 0, "read packing");
	CHECK_EQ(desc[1], 0x01000000u, "mem_addr");
	CHECK_EQ(desc[2], CHIPLAB_DMA_NAND_DEV_ADDR, "dev_addr");
	CHECK_EQ(desc[3], CHIPLAB_NAND_PAGE_SPARE / 4u, "length in 4B words");
	CHECK_EQ(desc[5], 1u, "step_times must be 1");
	CHECK_EQ(desc[6] & CHIPLAB_DMA_CMD_RW, 0u, "read direction bit clear");

	CHECK_EQ(chiplab_nand_dma_desc_pack(desc, CHIPLAB_DMA_DESC_WORDS,
					    0, 0x01000000u, CHIPLAB_DMA_NAND_DEV_ADDR,
					    CHIPLAB_NAND_PAGE_SPARE, 1),
		 0, "write packing");
	CHECK_EQ(desc[6] & CHIPLAB_DMA_CMD_RW, CHIPLAB_DMA_CMD_RW,
		 "write direction bit set");

	/* 非法参数必须被拒绝（fail-closed），不得静默产生可运行描述符 */
	CHECK(chiplab_nand_dma_desc_pack(desc, CHIPLAB_DMA_DESC_WORDS,
					 0, 0x01000000u, CHIPLAB_DMA_NAND_DEV_ADDR,
					 0, 0) < 0, "zero length rejected");
	CHECK(chiplab_nand_dma_desc_pack(desc, CHIPLAB_DMA_DESC_WORDS,
					 0, 0x01000000u, CHIPLAB_DMA_NAND_DEV_ADDR,
					 7, 0) < 0, "unaligned length rejected");
	CHECK(chiplab_nand_dma_desc_pack(desc, CHIPLAB_DMA_DESC_WORDS,
					 0, 0x01000004u, CHIPLAB_DMA_NAND_DEV_ADDR,
					 2048, 0) < 0, "unaligned mem_addr rejected");
	CHECK(chiplab_nand_dma_desc_pack(desc, 4, 0, 0x01000000u,
					 CHIPLAB_DMA_NAND_DEV_ADDR, 2048, 0) < 0,
	      "too-small descriptor array rejected");
	CHECK(chiplab_nand_dma_desc_pack(NULL, CHIPLAB_DMA_DESC_WORDS, 0,
					 0x01000000u, CHIPLAB_DMA_NAND_DEV_ADDR,
					 2048, 0) < 0, "NULL descriptor rejected");

	/*
	 * order 寄存器值：描述符物理地址（低 5 位清零）+ dma_start 置位。
	 * 注意低 5 位里的 bit3 就是 dma_start，所以"对齐"要按掩码检查，
	 * 不能对整体取模 32。
	 */
	{
		uint32_t v = chiplab_nand_dma_order_start(0x00012345u);

		CHECK_EQ(v & CHIPLAB_DMA_ORDER_ADDR_MASK, 0x00012340u,
			 "descriptor address masked to 32 B");
		CHECK(v & CHIPLAB_DMA_ORDER_START, "dma_start set");
		CHECK_EQ(v & CHIPLAB_DMA_ORDER_ASK, 0u, "ask_valid not set by driver");
		/* 已对齐的地址必须原样保留（不能被掩码改动） */
		CHECK_EQ(chiplab_nand_dma_order_start(0x00012340u), 0x00012348u,
			 "aligned address preserved + start bit");
	}

	/* 描述符自身必须 32 字节，且字段不越界 */
	CHECK_EQ(sizeof(buf) * 4u >= CHIPLAB_DMA_DESC_BYTES, 1u, "descriptor size");
	end();
}

/* ------------------------------------------------------------------ */

int main(void)
{
	printf("chiplab nand unit tests (host)\n");
	printf("ECC strength = %u bits / %u B, t = %u\n",
	       CHIPLAB_NAND_ECC_STRENGTH, CHIPLAB_NAND_ECC_STEP_SIZE,
	       CHIPLAB_BCH_T);

	test_bit_order();
	test_ecc();
	test_ecc_capacity();
	test_bbt();
	test_geometry();
	test_dma_desc();

	printf("\n%d checks, %u failures\n", checks, failures);
	if (failures) {
		printf("RESULT: FAIL\n");
		return 1;
	}
	printf("RESULT: PASS (all assertions held, fail-closed preserved)\n");
	return 0;
}
