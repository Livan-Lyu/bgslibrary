#!/bin/bash
# FPGA MMIO 诊断脚本 — 测试 UIO、/dev/mem、物理地址映射
# 用法: sudo bash diagnose_mmio.sh
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
UIO_DEV=""

echo "=============================================="
echo " FPGA MMIO 诊断"
echo " CAPE_BASE  = 0x$(printf '%X' $CAPE_BASE)"
echo "=============================================="
echo

# ====== 1. 运行权限 ======
echo "--- 1. 权限检查 ---"
if [ "$(id -u)" -eq 0 ]; then
    pass "以 root 运行"
else
    fail "非 root 用户 (uid=$(id -u))"
    info "用 sudo bash $0 重试"
fi
echo

# ====== 2. UIO 设备 ======
echo "--- 2. UIO 设备检查 ---"
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
fi

# 匹配 pixel-proc
if grep -q "pixel.proc" /sys/class/uio/*/name 2>/dev/null; then
    pass "找到 pixel-proc UIO 设备"
    UIO_DEV=$(grep -l "pixel.proc" /sys/class/uio/*/name | head -1 | xargs dirname | xargs basename)
    UIO_DEV="/dev/$UIO_DEV"
    info "设备路径: $UIO_DEV"
else
    warn "未找到名为 pixel-proc 的 UIO 设备"
    info "可用的 UIO 名称:"
    cat /sys/class/uio/*/name 2>/dev/null | sed 's/^/       /' || echo "       (无)"
fi
echo

# ====== 3. /dev/mem 检查 ======
echo "--- 3. /dev/mem 检查 ---"
if [ -e /dev/mem ]; then
    info "/dev/mem 存在"
else
    fail "/dev/mem 不存在"
fi
# 尝试读取
PIXEL_PROC_ADDR=$((CAPE_BASE + 0x80))
if dd if=/dev/mem of=/dev/null bs=1 count=1 skip=$PIXEL_PROC_ADDR 2>/dev/null; then
    pass "可以读取 /dev/mem (0x$(printf '%X' $PIXEL_PROC_ADDR))"
else
    fail "无法读取 /dev/mem — 权限不足或 CONFIG_STRICT_DEVMEM"
fi
echo

# ====== 4. 编译 C 程序测试 UIO 和 /dev/mem mmap ======
echo "--- 4. 寄存器读写测试 ---"
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

cat > "$TMPDIR/test_mmap.c" << 'CEOF'
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

#define CAPE_BASE      0x41280000
#define REG_CONTROL    0x80
#define REG_STATUS     0x84
#define REG_SRC_ADDR_LO 0x88
#define REG_SRC_ADDR_HI 0x8C
#define REG_PIXEL_COUNT 0x90
#define REG_RESULT     0x94
#define REG_DEBUG      0x98

static const char* reg_name(uint32_t off) {
    switch (off) {
        case REG_CONTROL:    return "A_CTRL";
        case REG_STATUS:     return "A_STAT";
        case REG_SRC_ADDR_LO:return "A_SRC_LO";
        case REG_SRC_ADDR_HI:return "A_SRC_HI";
        case REG_PIXEL_COUNT:return "A_CNT";
        case REG_RESULT:     return "A_RES";
        case REG_DEBUG:      return "A_DBG";
        default: return "?";
    }
}

static uint32_t reg_read(volatile uint32_t *regs, uint32_t offset) {
    return regs[offset >> 2];
}

static void reg_write(volatile uint32_t *regs, uint32_t offset, uint32_t val) {
    regs[offset >> 2] = val;
}

static int test_regs(volatile uint32_t *regs, const char *label) {
    printf("  [%s]\n", label);

    uint32_t offsets[] = {REG_DEBUG, REG_STATUS, REG_CONTROL, REG_RESULT,
                           REG_SRC_ADDR_LO, REG_SRC_ADDR_HI, REG_PIXEL_COUNT};
    int all_zero = 1;
    for (int i = 0; i < 7; i++) {
        uint32_t off = offsets[i];
        uint32_t val = reg_read(regs, off);
        printf("    %-10s (0x%02X) = 0x%08X", reg_name(off), off, val);
        if (off == REG_DEBUG && val == 0xDEADBEEF)
            printf("  [OK: magic]");
        if (val != 0) all_zero = 0;
        printf("\n");
    }

    if (all_zero) {
        printf("    [WARN] 全部寄存器返回 0 — 总线无响应\n");
        return 1;
    }

    // Quick smoke test: write SRC_ADDR_LO, read back
    printf("    [TEST] A_SRC_LO 写/读回 ... ");
    reg_write(regs, REG_SRC_ADDR_LO, 0x12345678);
    uint32_t rb = reg_read(regs, REG_SRC_ADDR_LO);
    if (rb == 0x12345678) {
        printf("OK (0x%08X 匹配)\n", rb);
    } else if (rb == 0) {
        printf("FAIL (读回 0，寄存器不可写)\n");
    } else {
        printf("WARN (写 0x12345678, 读回 0x%08X)\n", rb);
    }

    return 0;
}

int main(int argc, char **argv) {
    const char *uio_path = (argc > 1) ? argv[1] : NULL;
    int uio_ok = 0, mem_ok = 0;

    // ---- UIO path ----
    if (uio_path && *uio_path) {
        printf("  [UIO] open %s ... ", uio_path);
        int fd = open(uio_path, O_RDWR | O_SYNC);
        if (fd < 0) { perror("FAIL"); }
        else {
            printf("OK (fd=%d)\n", fd);
            printf("  [UIO] mmap (offset=0, size=0x1000) ... ");
            volatile uint32_t *regs = mmap(NULL, 0x1000,
                PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
            if (regs == MAP_FAILED) { perror("FAIL"); close(fd); }
            else {
                printf("OK\n");
                uio_ok = (test_regs(regs, "UIO") == 0);
                munmap((void*)regs, 0x1000);
            }
            close(fd);
        }
    } else {
        printf("  [UIO] 未指定设备，跳过\n");
    }

    // ---- /dev/mem fallback ----
    {
        printf("  [/dev/mem] open ... ");
        int fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (fd < 0) { perror("FAIL"); }
        else {
            printf("OK (fd=%d)\n", fd);
            long ps = sysconf(_SC_PAGESIZE);
            off_t map_base = CAPE_BASE & ~(ps - 1);
            size_t map_delta = CAPE_BASE - map_base;
            size_t map_bytes = ((0x100 + map_delta + ps - 1) / ps) * ps;
            printf("  [/dev/mem] mmap @ 0x%lx, bytes=0x%zx ... ",
                   (unsigned long)map_base, map_bytes);
            volatile uint32_t *base = mmap(NULL, map_bytes,
                PROT_READ|PROT_WRITE, MAP_SHARED, fd, map_base);
            if (base == MAP_FAILED) { perror("FAIL"); close(fd); }
            else {
                printf("OK\n");
                volatile uint32_t *regs = (volatile uint32_t*)((uint8_t*)base + map_delta);
                mem_ok = (test_regs(regs, "/dev/mem") == 0);
                munmap((void*)base, map_bytes);
            }
            close(fd);
        }
    }

    printf("\n");
    if (uio_ok) { printf("  [RESULT] UIO 路径可用 ✓\n"); return 0; }
    if (mem_ok) { printf("  [RESULT] /dev/mem 路径可用 (UIO 不可用)\n"); return 0; }
    printf("  [RESULT] 两条路径均不可用 ✗\n");
    return 1;
}
CEOF

if gcc -o "$TMPDIR/test_mmap" "$TMPDIR/test_mmap.c" 2>/dev/null; then
    info "编译测试程序成功，开始测试..."
    if [ -n "$UIO_DEV" ]; then
        "$TMPDIR/test_mmap" "$UIO_DEV"
    else
        "$TMPDIR/test_mmap" ""
    fi
else
    warn "无法编译 C 测试程序 (需要 gcc)"
fi
echo

# ====== 5. 内核配置 ======
echo "--- 5. 内核配置检查 ---"
KCONFIG="/boot/config-$(uname -r)"
if [ -f "$KCONFIG" ]; then
    if grep -q "CONFIG_STRICT_DEVMEM=y" "$KCONFIG" 2>/dev/null; then
        warn "CONFIG_STRICT_DEVMEM=y — /dev/mem 受限，必须用 UIO"
    else
        info "CONFIG_STRICT_DEVMEM 未强制启用"
    fi
    if grep -qE "CONFIG_UIO=y|CONFIG_UIO=m" "$KCONFIG" 2>/dev/null; then
        pass "CONFIG_UIO 已启用"
    else
        fail "CONFIG_UIO 未启用"
    fi
else
    warn "未找到 $KCONFIG"
fi
echo

# ====== 6. /proc/iomem ======
echo "--- 6. /proc/iomem ---"
if grep -qi "41280000\|pixel" /proc/iomem 2>/dev/null; then
    pass "地址 0x41280000 在 /proc/iomem 中已注册"
else
    warn "地址 0x41280000 未在 /proc/iomem 中找到"
    info "设备树 overlay 可能未加载"
fi
echo

echo "=============================================="
echo " 诊断完成"
echo "=============================================="
