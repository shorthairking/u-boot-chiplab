#!/bin/sh
# SPDX-License-Identifier: GPL-2.0+
#
# Chiplab NAND 驱动 —— 纯 C 单元自测入口（宿主机运行，不依赖硬件 / 不依赖 U-Boot）
#
# 用法：
#   board/loongson/chiplab/nand-unittest/run.sh
#
# 判据：**未捕获即失败**。任何断言失败、编译失败、或"构建通过但没跑起来"
# 都返回非 0；全绿时打印的兜底文案**不含 FAIL/PASS 之外的歧义**。
#
# 反证（mutation test）：把纠错能力上限调小（BCH-4 当 BCH-3/N 用），
# 注入超限错误必须 FAIL —— 见本合同内的 --negative 模式与
# board/loongson/chiplab/README「NAND 驱动」节的记录。

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
UBOOT=$(cd "$HERE/../../../.." && pwd)
HDR="$UBOOT/drivers/mtd/nand/raw/chiplab_nand.h"
SRC="$HERE/test_chiplab_nand.c"

for f in "$HDR" "$SRC"; do
	[ -f "$f" ] || { echo "missing file: $f" >&2; exit 1; }
done

CC=${CC:-cc}
HOSTCFLAGS=${HOSTCFLAGS:--O2 -g -Wall -Wextra -Wno-unused-function \
	-Wno-unused-parameter -Wno-array-bounds}

mode=normal
strength=""
outdir=${TMPDIR:-/tmp}/chiplab-nand-unittest.$$
mkdir -p "$outdir"
trap 'rm -rf "$outdir"' EXIT INT TERM

while [ $# -gt 0 ]; do
	case "$1" in
	--negative)	mode=negative; shift ;;
	--strength)	strength=$2; shift 2 ;;
	--mutate-decode)	mode=mutate; strength=$2; shift 2 ;;
	*)		echo "unknown option: $1" >&2; exit 2 ;;
	esac
done

# 把 -I<dir_of_header> 传给编译器，测试直接 #include "chiplab_nand.h"
INCDIR="$UBOOT/drivers/mtd/nand/raw"

extra=""
case "$mode" in
mutate)
	# 只把**解码侧**能力上限调小（模拟"BCH-4 当 BCH-3 用"），
	# 编码侧仍是完整强度 —— 于是"t 位错必须纠正"这条断言必然变红。
	extra="-DCHIPLAB_ECC_DECODE_STRENGTH=$strength"
	;;
*)
	if [ -n "$strength" ]; then
		extra="-DCHIPLAB_ECC_STRENGTH_DEFAULT=$strength"
	fi
	;;
esac

echo "== chiplab NAND unit test =="
echo "header : $HDR"
echo "source : $SRC"
echo "mode   : $mode${strength:+ (strength=$strength)}"

bin="$outdir/t"
set +e
$CC $HOSTCFLAGS $extra -I"$INCDIR" -I"$HERE" -o "$bin" "$SRC" 2>"$outdir/cc.log"
cc_rc=$?
set -e
if [ $cc_rc -ne 0 ]; then
	echo "COMPILE FAILED (rc=$cc_rc)" >&2
	cat "$outdir/cc.log" >&2
	exit 1
fi

set +e
"$bin" >"$outdir/run.log" 2>&1
run_rc=$?
set -e
cat "$outdir/run.log"

if [ "$mode" = mutate ]; then
	# 变异模式：断言**必须**失败，证明测试真的在观测纠错能力上限。
	if [ $run_rc -eq 0 ]; then
		echo "MUTATION NOT DETECTED: 解码能力上限被调到 $strength 后测试仍全绿，" >&2
		echo "说明断言没有真正观测到能力上限（反证失败）" >&2
		exit 1
	fi
	grep -q "exactly t injected errors must be corrected" "$outdir/run.log" || {
		echo "mutation was detected, but not by the capability assertion" >&2
		exit 1
	}
	echo "MUTATION DETECTED: 解码能力上限=$strength 时能力断言如预期变红 (rc=$run_rc)"
	exit 0
fi

if [ "$mode" = negative ]; then
	# 负向模式：**必须**失败（用于证明测试确实在观测纠错能力上限）
	if [ $run_rc -eq 0 ]; then
		echo "NEGATIVE MODE FAILED: 能力被调小后测试仍然全绿，说明断言没有观测到纠错能力上限" >&2
		exit 1
	fi
	echo "NEGATIVE MODE OK: 能力调小后测试如预期变红 (rc=$run_rc)"
	exit 0
fi

if [ $run_rc -ne 0 ]; then
	echo "UNIT TEST FAILED (rc=$run_rc)" >&2
	exit $run_rc
fi

# 兜底：确认真的跑了断言（防止"编译出空程序也算通过"）
n=$(sed -n 's/^\([0-9][0-9]*\) checks, \([0-9][0-9]*\) failures$/\1 \2/p' "$outdir/run.log")
[ -n "$n" ] || { echo "no assertion summary found in output" >&2; exit 1; }
set -- $n
[ "$1" -ge 50 ] || { echo "too few checks run ($1)" >&2; exit 1; }
[ "$2" -eq 0 ] || { echo "$2 failures reported" >&2; exit 1; }

echo "ALL CHECKS PASSED ($1 assertions, 0 failures)"
