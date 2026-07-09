#!/bin/bash
# FPGA MMIO 诊断脚本 — 测试 /dev/mem、UIO、物理地址映射
# 用法: bash diagnose_mmio.sh

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass()  { echo -e "${GREEN}[PASS]${NC} $*"; }
fail()  { echo -e "${RED}[FAIL]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
info()  { echo -e "       $*"; }

CAPE_BASE=0x41280000
REG_OFFSET=0x80
PIXEL_PROC_ADDR=$((CAPE_BASE + REG_OFFSET))
PAGE_SIZE=$(getconf PAGESIZE)
MAP_BASE=$((CAPE_BASE & ~(PAGE_SIZE - 1)))

echo "=============================================="
echo " FPGA MMIO 诊断"
echo " CAPE_BASE  = 0x${CAPE_BASE:x}"
echo " Page Size  = $PAGE_SIZE"
echo " MAP_BASE   = 0x$(printf '%x' $MAP_BASE)"
echo "=============================================="
echo

# ====== 1. 运行权限 ======
echo "--- 1. 权限检查 ---"
if [ "$(id -u)" -eq 0 ]; then
    pass "以 root 运行"
else
    fail "非 root 用户 (uid=$(id -u)) — /dev/mem 需要 root"
    info "sudo bash $0 重试，或加入 dialout/uio 组"
fi
echo

# ====== 2. /dev/mem 是否可用 ======
echo "--- 2. /dev/mem 检查 ---"
if [ -e /dev/mem ]; then
    info "/dev/mem 存在"
else
    fail "/dev/mem 不存在"
fi

# 尝试打开 /dev/mem
if dd if=/dev/mem of=/dev/null bs=1 count=1 skip=$PIXEL_PROC_ADDR 2>/dev/null; then
    pass "可以读取 /dev/mem 偏移 0x$(printf '%x' $PIXEL_PROC_ADDR)"
else
    fail "无法读取 /dev/mem — 权限不足或 CONFIG_STRICT_DEVMEM 已启用"
    info "检查: cat /boot/config-$(uname -r) | grep STRICT_DEVMEM"
fi
echo

# ====== 3. UIO 设备 ======
echo "--- 3. UIO 设备检查 ---"
UIO_COUNT=0
if ls /dev/uio* 2>/dev/null | grep -q uio; then
    for uio in /dev/uio*; do
        uio_num="${uio##*/uio}"
        uio_name=$(cat "/sys/class/uio/${uio_num}/name" 2>/dev/null || echo "?")
        uio_addr=$(cat "/sys/class/uio/${uio_num}/maps/map0/addr" 2>/dev/null || echo "?")
        uio_size=$(cat "/sys/class/uio/${uio_num}/maps/map0/size" 2>/dev/null || echo "?")
        echo "  $uio  name=\"$uio_name\"  addr=$uio_addr  size=$uio_size"
        UIO_COUNT=$((UIO_COUNT + 1))
    done
    pass "找到 $UIO_COUNT 个 UIO 设备"
else
    warn "未找到 /dev/uio* 设备"
    info "检查设备树 overlay 是否已加载"
    info "ls /sys/class/uio/"
fi

# 检查是否匹配 pixel-proc
if grep -q "pixel.proc" /sys/class/uio/*/name 2>/dev/null; then
    pass "找到 pixel-proc UIO 设备"
    UIO_DEV=$(grep -l "pixel.proc" /sys/class/uio/*/name | head -1 | xargs dirname | xargs basename)
    info "UIO 设备: /dev/$UIO_DEV"
else
    warn "未找到名为 pixel-proc 的 UIO 设备"
    info "可用的 UIO 名称:"
    cat /sys/class/uio/*/name 2>/dev/null | sed 's/^/       /' || echo "       (无)"
fi
echo

# ====== 4. 编译 C 程序测试 mmap ======
echo "--- 4. 物理地址 mmap 测试 ---"
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

cat > "$TMPDIR/test_mmap.c" << 'EOF'
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

#define CAPE_BASE    0x41280000
#define DBG_OFFSET   0x98    // A_DBG: 应返回 0xDEADBEEF
#define CTRL_OFFSET  0x80    // A_CTRL
#define STAT_OFFSET  0x84    // A_STAT

int main() {
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0) {
        fprintf(stderr, "FAIL: sysconf(_SC_PAGESIZE) = %ld\n", page_size);
        return 1;
    }
    printf("  page_size = %ld\n", page_size);

    off_t map_base = CAPE_BASE & ~(page_size - 1);
    size_t map_delta = CAPE_BASE - map_base;
    size_t map_bytes = ((0x100 + map_delta + page_size - 1) / page_size) * page_size;

    // ---- 测试 /dev/mem ----
    printf("  [1/2] open /dev/mem ... ");
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("FAIL");
        printf("  [2/2] skip (no /dev/mem)\n");
        return 1;
    }
    printf("OK (fd=%d)\n", fd);

    printf("  [2/2] mmap /dev/mem @ 0x%lx ... ", (unsigned long)map_base);
    volatile uint32_t *base = mmap(NULL, map_bytes,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd, map_base);
    if (base == MAP_FAILED) {
        perror("FAIL");
        close(fd);
        return 1;
    }
    printf("OK\n");

    volatile uint32_t *regs = (volatile uint32_t*)((uint8_t*)base + map_delta);
    uint32_t dbg_offset_w = DBG_OFFSET / 4;
    uint32_t ctrl_offset_w = CTRL_OFFSET / 4;
    uint32_t stat_offset_w = STAT_OFFSET / 4;

    // 读 A_DBG (0x98) — 应返回 0xDEADBEEF
    uint32_t dbg = regs[dbg_offset_w];
    printf("  A_DBG  (0x%02X)  = 0x%08X", DBG_OFFSET, dbg);
    if (dbg == 0xDEADBEEF) {
        printf("  [OK: DEADBEEF]\n");
    } else if (dbg == 0 || dbg == 0xFFFFFFFF) {
        printf("  [WARN: 总线无响应 (%s)]\n",
               dbg == 0 ? "全0" : "全F");
    } else {
        printf("  [WARN: 非预期值]\n");
    }

    // 读 A_STAT (0x84)
    uint32_t stat = regs[stat_offset_w];
    printf("  A_STAT (0x%02X)  = 0x%08X  [busy=%d irq=%d done=%d]\n",
           STAT_OFFSET, stat,
           (stat>>0)&1, (stat>>1)&1, (stat>>2)&1);

    // 读 A_CTRL (0x80)
    uint32_t ctrl = regs[ctrl_offset_w];
    printf("  A_CTRL (0x%02X)  = 0x%08X  [start=%d ack=%d]\n",
           CTRL_OFFSET, ctrl, (ctrl>>0)&1, (ctrl>>7)&1);

    // 读取 A_RES (0x94) 试试
    uint32_t res = regs[0x94/4];
    printf("  A_RES  (0x94)  = 0x%08X\n", res);

    munmap((void*)base, map_bytes);
    close(fd);
    printf("  [DONE] /dev/mem + mmap 成功\n");
    return 0;
}
EOF

if gcc -o "$TMPDIR/test_mmap" "$TMPDIR/test_mmap.c" 2>/dev/null; then
    info "编译测试程序成功，开始测试 mmap..."
    if "$TMPDIR/test_mmap"; then
        pass "/dev/mem mmap 成功，硬件可访问"
    else
        fail "/dev/mem mmap 失败 — 物理地址 0x41280000 不可访问"
        info "可能原因: 地址不在总线范围、页表未映射、无硬件"
    fi
else
    warn "无法编译 C 测试程序 (需要 gcc)，跳过 mmap 测试"
fi
echo

# ====== 5. 内核相关检查 ======
echo "--- 5. 内核配置检查 ---"
if [ -f "/boot/config-$(uname -r)" ]; then
    if grep -q "CONFIG_STRICT_DEVMEM=y" "/boot/config-$(uname -r)" 2>/dev/null; then
        warn "CONFIG_STRICT_DEVMEM=y — /dev/mem 受限，建议用 UIO"
    else
        info "CONFIG_STRICT_DEVMEM 未启用 (or n)"
    fi
    if grep -q "CONFIG_UIO=y\|CONFIG_UIO=m" "/boot/config-$(uname -r)" 2>/dev/null; then
        pass "CONFIG_UIO 已启用"
    else
        fail "CONFIG_UIO 未启用 — 无法使用 /dev/uio"
    fi
else
    warn "未找到 /boot/config-$(uname -r)"
fi

# 检查 /proc/iomem 中是否有我们的地址范围
echo
echo "--- 6. /proc/iomem 地址范围 ---"
if grep -i "41280000\|pixel" /proc/iomem 2>/dev/null; then
    pass "地址 0x41280000 在 /proc/iomem 中已注册"
else
    warn "地址 0x41280000 未在 /proc/iomem 中找到"
    info "这通常意味着设备树 overlay 未加载，或硬件不在总线"
fi
echo

echo "=============================================="
echo " 诊断完成"
echo "=============================================="
