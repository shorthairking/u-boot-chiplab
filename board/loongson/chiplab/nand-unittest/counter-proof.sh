#!/bin/sh
# SPDX-License-Identifier: GPL-2.0+
#
# 反证（mutation test）：把 ECC 的纠错能力上限改小，注入超限错误必须 FAIL。
#
# 纪律（任务书强制）：
#   * **严禁** git checkout / stash / reset；
#   * 只对 `drivers/mtd/nand/raw/chiplab_nand.h` 的**一处常量**做
#     cp 备份 → sed 改写 → 跑测试 → 从备份恢复；
#   * 恢复后必须再跑一次正常模式确认全绿（证明恢复干净）。
#
# 用法：board/loongson/chiplab/nand-unittest/counter-proof.sh
# 退出码：0 = 反证成立且恢复干净；非 0 = 反证失败或恢复不干净（都必须红）。

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
UBOOT=$(cd "$HERE/../../../.." && pwd)
HDR="$UBOOT/drivers/mtd/nand/raw/chiplab_nand.h"
RUN="$HERE/run.sh"
BAK="${TMPDIR:-/tmp}/chiplab_nand.h.counterproof.$$"

[ -f "$HDR" ] || { echo "missing $HDR" >&2; exit 1; }

cleanup() {
	# 无论怎么退出，都必须把原文件放回去
	if [ -f "$BAK" ]; then
		cp "$BAK" "$HDR"
		rm -f "$BAK"
	fi
}
trap cleanup EXIT INT TERM

echo "== 反证：把解码侧纠错能力上限由 4 bit 改为 3 bit =="
echo "备份 $HDR -> $BAK"
cp "$HDR" "$BAK"

# 原始行必须存在，否则说明头文件结构变了，反证脚本要跟着改
grep -q '^#define CHIPLAB_ECC_STRENGTH_DEFAULT	4$' "$HDR" || {
	echo "anchor not found: CHIPLAB_ECC_STRENGTH_DEFAULT 4" >&2
	exit 1
}

# 只改解码侧能力（编码保持 4）—— 正是"把 BCH-4 当 BCH-3 用"的场景
sed -i 's|^#define CHIPLAB_ECC_DECODE_STRENGTH\tCHIPLAB_ECC_STRENGTH_DEFAULT$|#define CHIPLAB_ECC_DECODE_STRENGTH\t3|' "$HDR"

if ! grep -q '^#define CHIPLAB_ECC_DECODE_STRENGTH	3$' "$HDR"; then
	echo "mutation not applied (sed anchor mismatch)" >&2
	exit 1
fi
echo "已把 CHIPLAB_ECC_DECODE_STRENGTH 改为 3"

set +e
"$RUN" --mutate-decode 3
rc=$?
set -e
if [ $rc -ne 0 ]; then
	echo "COUNTER-PROOF FAILED: 能力调小后测试没有按预期变红（rc=$rc）" >&2
	exit 1
fi

echo
echo "== 恢复 =="
cleanup
trap - EXIT INT TERM
grep -q '^#define CHIPLAB_ECC_STRENGTH_DEFAULT	4$' "$HDR" || {
	echo "restore check failed" >&2
	exit 1
}
grep -q '^#define CHIPLAB_ECC_DECODE_STRENGTH	CHIPLAB_ECC_STRENGTH_DEFAULT$' "$HDR" || {
	echo "restore check failed: decode strength not restored" >&2
	exit 1
}
echo "已从备份恢复（无 git checkout/stash/reset 参与）"

echo
echo "== 恢复后正常模式复核 =="
"$RUN" >/dev/null
echo "COUNTER-PROOF OK: 反证成立，且恢复后单元测试全绿"
