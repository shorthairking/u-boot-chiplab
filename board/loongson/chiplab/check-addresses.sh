#!/bin/sh
# SPDX-License-Identifier: GPL-2.0+
#
# 地址口径机器核对：chiplab-rv32 板级文件 vs 平台事实台账
#
#   用法: board/loongson/chiplab/check-addresses.sh [dts] [facts]
#   退出码: 0 = 全部命中; 1 = 有任一条未命中; 2 = 输入缺失 / dtc 失败
#           （未捕获即失败：兜底文案不含 PASS）
#
# 核对对象：
#   arch/riscv/dts/chiplab-rv32.dts          (DTS 地址，先用 dtc 规范化，再做节点级检查)
#   configs/chiplab_rv32_defconfig           (U-Boot 侧同一批地址/时钟/分区口径)
#   rv32gc-cpu/docs/kb/platform-facts.md     (平台事实唯一真源)
#
# 为什么先 dtc 规范化：直接在源码文本里 grep 地址会被**注释**命中（把 reg 改错、
# 注释没改也能“通过”）。本脚本把 DTS 编成 DTB 再反解成规范 DTS，从而只检查
# 真实的 reg/property 值，并且按“节点名 -> reg/属性”逐条定位。
#
# 反证（两条，改完必须回红；见 README）：
#   A. 只改 DTS 里 UART 的 reg 地址（注释/node 名不动）-> §1 必红
#   B. 只改 defconfig 里 CONFIG_DEBUG_UART_BASE        -> §3 必红

set -u

here=$(dirname "$0")
top=$(git -C "$here" rev-parse --show-toplevel 2>/dev/null) || top=$(cd "$here/../../.." && pwd)

DTS=${1:-$top/arch/riscv/dts/chiplab-rv32.dts}
DEFCONFIG=${DEFCONFIG:-$top/configs/chiplab_rv32_defconfig}
FACTS=${2:-${PLATFORM_FACTS:-$top/../rv32gc-cpu/docs/kb/platform-facts.md}}

for f in "$DTS" "$DEFCONFIG" "$FACTS"; do
	if [ ! -f "$f" ]; then
		echo "MISSING-INPUT: $f"
		exit 2
	fi
done
command -v dtc >/dev/null 2>&1 || { echo "MISSING-INPUT: dtc 不在 PATH"; exit 2; }

tmp=$(mktemp -d) || exit 2
trap 'rm -rf "$tmp"' EXIT
canon=$tmp/canon.dts

if ! dtc -I dts -O dtb -o "$tmp/b.dtb" "$DTS" > "$tmp/dtc.log" 2>&1; then
	echo "dtc 编译失败（$DTS）："
	cat "$tmp/dtc.log"
	exit 2
fi
if ! dtc -I dtb -O dts -o "$canon" "$tmp/b.dtb" > "$tmp/dtc2.log" 2>&1; then
	echo "dtc 反解失败："
	cat "$tmp/dtc2.log"
	exit 2
fi

# 平台事实归一：大写→小写 + 十六进制字面量内部下划线删除（0x1FE0_01E0 -> 0x1fe001e0）
facts_n=$tmp/facts.norm
tr 'A-Z' 'a-z' < "$FACTS" | sed -E 's/0x([0-9a-f]+)_([0-9a-f]+)/0x\1\2/g' > "$facts_n"

fail=0
ok()  { echo "MATCH  : $1"; }
bad() { echo "MISSING: $1"; fail=1; }

# 节点名支持“前缀”写法（memory@ / serial@ / cpu@）：匹配 "<前缀><unit> {"
# 只在节点块内取第一条 reg / 指定属性，注释与同名文本无法伪造。
node_awk() { # node_awk <文件> <节点名前缀> [<属性名>]
	if [ $# -ge 3 ]; then
		awk -v n="$2" -v p="$3" '
			$0 ~ ("^[ \t]*" n "[^ \t]* \\{") { f = 1; next }
			f && index($0, p" = ") { print; exit }
		' "$1"
	else
		awk -v n="$2" '
			$0 ~ ("^[ \t]*" n "[^ \t]* \\{") { f = 1; next }
			f && /reg = </ { print; exit }
		' "$1"
	fi
}

# 数值统一成十进制（dash 的 test 不认 0x 字面量）
num() {
	case ${1:-} in
	"") echo "" ;;
	0x*|0X*) printf '%d' "$(( $1 ))" 2>/dev/null || echo "" ;;
	*) printf '%s' "$1" ;;
	esac
}
cell1() { sed -n 's/.*<\(0x[0-9a-f]*\).*/\1/p'; }
only_cell() { sed -n 's/.*<\(0x[0-9a-f]*\)>.*/\1/p'; }

exp_reg() { # exp_reg <节点名前缀> <reg 期望字面量> <说明>
	_line=$(node_awk "$canon" "$1")
	if [ -n "$_line" ] && printf '%s' "$_line" | grep -qF "<$2>"; then
		ok "$3 [$1*] reg = <$2>"
	else
		bad "$3 [$1*] 期望 reg = <$2>；实得：${_line:-<节点不存在>}"
	fi
}

exp_prop() { # exp_prop <节点名前缀> <属性> <期望子串> <说明>
	_line=$(node_awk "$canon" "$1" "$2")
	if [ -n "$_line" ] && printf '%s' "$_line" | grep -qF "$3"; then
		ok "$4 [$1*] $2 含 $3"
	else
		bad "$4 [$1*] 期望 $2 含 $3；实得：${_line:-<无>}"
	fi
}

getk() { sed -n "s/^CONFIG_$1=\(.*\)$/\1/p" "$DEFCONFIG" | head -1; }

echo "== 1. DTS（dtc 规范化后）逐条地址口径 =="
exp_reg memory@          "0x00 0x8000000"       "DDR3 128 MiB @ 0x0"
exp_reg clint@1f000000   "0x1f000000 0x10000"   "CLINT 基址"
exp_reg plic@1f100000    "0x1f100000 0x400000"  "PLIC 基址"
exp_reg serial@1fe001e0  "0x1fe001e0 0x100"     "UART 基址"
exp_reg spi-xip@1c000000 "0x1c000000 0x1000000" "SPI XIP 基址"
exp_reg nand@1fe78000    "0x1fe78000 0x100"     "NAND 基址（预留）"
exp_prop serial@1fe001e0 clock-frequency    "<0x1f78a40>" "UART 时钟 = 33 MHz"
exp_prop serial@1fe001e0 current-speed      "<0x1c200>"   "UART 波特率 = 115200"
exp_prop serial@1fe001e0 reg-shift          "<0x00>"      "UART reg-shift = 0（逐字节）"
exp_prop serial@1fe001e0 reg-io-width       "<0x01>"      "UART reg-io-width = 1"
exp_prop cpus            timebase-frequency "<0x1f78a40>" "timebase = 33 MHz"
exp_prop cpu@            mmu-type           "\"riscv,sv32\"" "MMU = Sv32"
exp_prop cpu@            riscv,isa          "rv32imafdc_zicsr_zifencei_zicntr_zicbom" "ISA 字符串"

echo "== 2. platform-facts 台账同口径 + 派生校验 =="
for lit in 0x1fe001e0 0x1f000000 0x1f100000 0x1c000000 0x1fe78000 33000000 0x07ffffff; do
	if grep -qF -- "$lit" "$facts_n"; then
		ok "platform-facts 含 $lit"
	else
		bad "platform-facts 缺 $lit"
	fi
done

# 派生：DTS /memory 的 size 必须等于台账上界 0x07FF_FFFF + 1
facts_top=$(num "$(grep -oF '0x07ffffff' "$facts_n" | head -1)")
mem_line=$(node_awk "$canon" "memory@")
mem_size=$(printf '%s' "$mem_line" | sed -n 's/.*<0x[0-9a-f]* \(0x[0-9a-f]*\)>.*/\1/p')
if [ -n "$facts_top" ] && [ -n "$mem_size" ] && \
   [ "$(num "$mem_size")" -eq "$(( facts_top + 1 ))" ]; then
	ok "DDR3 size($mem_size) = 台账上界($facts_top)+1 = 128 MiB"
else
	bad "DDR3 size(${mem_size:-<无>}) != 台账上界(${facts_top:-<无>})+1"
fi

echo "== 3. defconfig <-> DTS 交叉一致（同一口径两侧不许分叉） =="
uart_dts=$(num "$(node_awk "$canon" "serial@" | cell1)")
uart_def=$(num "$(getk DEBUG_UART_BASE)")
if [ -n "$uart_dts" ] && [ "$uart_dts" = "$uart_def" ]; then
	ok "DEBUG_UART_BASE($(getk DEBUG_UART_BASE)) == DTS UART(0x$(printf '%x' "$uart_dts"))"
else
	bad "DEBUG_UART_BASE(${uart_def:-<无>}) != DTS UART(${uart_dts:-<无>})"
fi

clk_dts=$(num "$(node_awk "$canon" "serial@" clock-frequency | only_cell)")
clk_def=$(num "$(getk DEBUG_UART_CLOCK)")
if [ -n "$clk_dts" ] && [ "$clk_dts" = "$clk_def" ] && [ "$clk_dts" -eq 33000000 ]; then
	ok "DEBUG_UART_CLOCK($(getk DEBUG_UART_CLOCK)) == DTS UART 时钟($clk_dts) == 33 MHz"
else
	bad "DEBUG_UART_CLOCK(${clk_def:-<无>}) != DTS UART 时钟(${clk_dts:-<无>})/33 MHz"
fi

baud_dts=$(num "$(node_awk "$canon" "serial@" current-speed | only_cell)")
baud_def=$(num "$(getk BAUDRATE)")
if [ -n "$baud_dts" ] && [ "$baud_dts" = "$baud_def" ] && [ "$baud_dts" -eq 115200 ]; then
	ok "BAUDRATE($(getk BAUDRATE)) == DTS current-speed($baud_dts) == 115200"
else
	bad "BAUDRATE(${baud_def:-<无>}) != DTS current-speed(${baud_dts:-<无>})/115200"
fi

# env 区：256 KiB = 262144 B = 2 x 128 KiB(131072 B) 块，对齐分区表 256K(env)
env_range=$(num "$(getk ENV_RANGE)")
if [ -n "$env_range" ] && [ "$env_range" -eq 262144 ] && [ $(( env_range / 131072 )) -eq 2 ]; then
	ok "ENV_RANGE($(getk ENV_RANGE)) = 256 KiB = 2 个 128 KiB NAND 块（对齐 256K(env) 分区）"
else
	bad "ENV_RANGE($(getk ENV_RANGE)) 不等于 256 KiB / 2 个 128 KiB 块"
fi

tb_dts=$(num "$(node_awk "$canon" "cpus" timebase-frequency | only_cell)")
if [ -n "$tb_dts" ] && [ "$tb_dts" -eq 33000000 ]; then
	ok "timebase-frequency($tb_dts) = 33 MHz（S 模式定时器走 SBI，频率口径同平台）"
else
	bad "timebase-frequency(${tb_dts:-<无>}) != 33 MHz"
fi

# TEXT_BASE 的取值源是板级 Kconfig 的 default（defconfig 里通常不重复写）
BOARD_KCONFIG=${BOARD_KCONFIG:-$top/board/loongson/chiplab/Kconfig}
kconfig_default() {
	awk -v s="config $1" '
		$0 ~ ("^" s "$") { f = 1; next }
		/^config |^if |^endif/ { f = 0 }
		f && /^[ \t]*default / { print $2; exit }
	' "$BOARD_KCONFIG"
}
text_base=$(num "$(getk TEXT_BASE)")
[ -n "$text_base" ] || text_base=$(num "$(kconfig_default TEXT_BASE)")
sp_addr=$(num "$(getk CUSTOM_SYS_INIT_SP_ADDR)")
if [ -n "$text_base" ] && [ -n "$(num "$mem_size")" ] && \
   [ "$text_base" -gt 0 ] && [ "$text_base" -lt "$(num "$mem_size")" ] && \
   [ -n "$sp_addr" ] && [ "$sp_addr" -lt "$text_base" ]; then
	ok "TEXT_BASE(0x$(printf '%x' "$text_base")) 在 DDR3 内，且重定位前栈(0x$(printf '%x' "$sp_addr")) 在其下方"
else
	bad "TEXT_BASE(${text_base:-<无>}) / CUSTOM_SYS_INIT_SP_ADDR(${sp_addr:-<无>}) 与 DDR3 窗口(${mem_size:-<无>}) 关系不成立"
fi

echo "== 4. 禁用符号 / 命名口径 =="
if grep -q "riscv_m_mode" "$canon" "$DEFCONFIG"; then
	bad "出现旧的 RISCV_M_MODE 符号名（本树不存在该符号）"
else
	ok "未出现 RISCV_M_MODE"
fi
if grep -q "sys_text_base" "$DEFCONFIG"; then
	bad "出现 2019.07 时代的 CONFIG_SYS_TEXT_BASE（本树为 CONFIG_TEXT_BASE）"
else
	ok "未出现 CONFIG_SYS_TEXT_BASE"
fi
if grep -q "^CONFIG_RISCV_SMODE=y" "$DEFCONFIG" && ! grep -q "^CONFIG_RISCV_MMODE=y" "$DEFCONFIG"; then
	ok "运行模式 = RISCV_SMODE（M 模式由 OpenSBI 负责）"
else
	bad "运行模式不是纯 RISCV_SMODE"
fi

if [ "$fail" -ne 0 ]; then
	echo "RESULT : 地址口径核对未通过"
	exit 1
fi
echo "RESULT : 地址口径核对全部命中"
exit 0
