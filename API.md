软件接口完整总结
一、地址映射
物理地址	模块	说明
0x41100000	CAPE (APB slot)	apb_ctrl_status (0x00-0x7F) + pixel_proc (0x80-0xFF)
二、pixel_proc 寄存器（基址 0x41100080）
偏移	字索引	名称	访问	位段	说明
0x80	32	CONTROL	RW	[0] START<br>[7] ACK	写 0x01 启动；写 0x80 确认已读结果
0x84	33	STATUS	RO	[0] BUSY<br>[1] READY<br>[2] DONE	READY=1 时可读 RESULT
0x88	34	SRC_ADDR_LO	RW	[31:0]	DDR 缓冲区物理地址低 32bit
0x8C	35	SRC_ADDR_HI	RW	[31:0]	DDR 缓冲区物理地址高 32bit
0x90	36	PIXEL_COUNT	RW	[31:0]	总像素数 N
0x94	37	RESULT	RO	[31:0]	打包 32bit 前景结果，[0]=第1个像素
0x98	38	DEBUG	RO	[31:0]	固定返回 0xDEADBEEF
注意：字索引 = 偏移值 / 4。用 /dev/mem 访问时：regs[偏移/4]。

三、DDR 数据排布
CPU 写入 DDR 缓冲区的格式（pixel_proc 通过 AXI 直接读取）：


偏移 (字节)      内容
───────────────  ─────────────────────────────
+0x0000_0000     pixel_0 当前帧    [R(7:0), G(15:8), B(23:16), 0x00]
+0x0000_0004     pixel_0 历史帧0   [R, G, B, 0x00]
+0x0000_0008     pixel_0 历史帧1   [R, G, B, 0x00]
    ⋮
+0x0000_005C     pixel_0 历史帧22  [R, G, B, 0x00]   ← 像素0 共 24条目×4B=96B
+0x0000_0060     pixel_1 当前帧    [R, G, B, 0x00]
    ⋮
每像素 = 1 current + 23 history = 24 条目 × 4 字节 = 96 字节
每条目 = 32bit word：[23:16]=B, [15:8]=G, [7:0]=R, [31:24]=0x00
总大小 = N × 96 字节
AXI 读取：每 beat 64bit = 2 条目，每像素 12 beats
四、操作流程

1. CPU 分配连续物理内存，写入 N 个像素数据（按上述格式）
   获取物理地址 ddr_phys

2. mmap /dev/mem (0x41100000, 0x100) 或 mmap /dev/uioX

3. 写 SRC_ADDR_LO  = (uint32_t)(ddr_phys & 0xFFFFFFFF)
   写 SRC_ADDR_HI  = (uint32_t)(ddr_phys >> 32)
   写 PIXEL_COUNT  = N
   写 CONTROL      = 0x01       // START

4. 循环处理结果：
   while (像素未处理完) {
       while ((STATUS & 0x02) == 0);   // 等 READY
       uint32_t r = read(RESULT);
       write(CONTROL, 0x80);           // ACK
       for (i = 0; i < 32 && pixels_left > 0; i++, pixels_left--)
           foreground[i] = (r >> i) & 1;
   }

5. while ((STATUS & 0x04) == 0);      // 等 DONE
   write(CONTROL, 0x00);               // 清除 START

6. munmap, 关闭文件
五、中断
项目	值
硬件 IRQ	pixel_proc_0_irq → CAPE:INT[0] → MSS_INT_F2M[8]
Linux IRQ	取决于设备树中断控制器映射
触发条件	32 个前景 bit 打包完成 (READY=1)
清除条件	CPU 写 CONTROL[7]=1 (ACK)
可以用轮询 STATUS[1] 替代中断，更简单。

六、Device Tree

// verilog-cape.dtso
&{/fabric-bus@40000000} {
    pixel_proc: pixel-proc@41100080 {
        compatible = "generic-uio";
        reg = <0x0 0x41100080 0x0 0x20>;
        status = "okay";
        linux,uio-name = "pixel-proc";
    };
};
七、C 示例代码

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#define CAPE_BASE    0x41100000
#define PIXEL_OFFSET 0x80
#define REG(r)       ((volatile uint32_t *)(base + (r)))

int main() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    volatile uint32_t *base = mmap(NULL, 0x100,
        PROT_READ|PROT_WRITE, MAP_SHARED, fd, CAPE_BASE);

    uint64_t ddr_phys = /* 你的 DDR 缓冲区物理地址 */;
    uint32_t N = /* 总像素数 */;

    REG(0x88) = (uint32_t)(ddr_phys & 0xFFFFFFFF);   // SRC_ADDR_LO
    REG(0x8C) = (uint32_t)(ddr_phys >> 32);           // SRC_ADDR_HI
    REG(0x90) = N;                                     // PIXEL_COUNT
    REG(0x80) = 0x01;                                  // START

    uint32_t done = 0, left = N;
    while (!done) {
        while (!(REG(0x84) & 0x02));          // wait READY
        uint32_t r = REG(0x94);               // read RESULT
        REG(0x80) = 0x80;                     // ACK

        for (int i = 0; i < 32 && left > 0; i++, left--) {
            uint8_t fg = (r >> i) & 1;
            printf("pixel %u: %s\n", N - left, fg ? "FG" : "BG");
        }
        if (REG(0x84) & 0x04) done = 1;       // DONE
    }
    REG(0x80) = 0x00;                          // clear START

    munmap((void*)base, 0x100);
    close(fd);
    return 0;
}
八、固定硬件参数（不可配置）
参数	值	说明
阈值 4.5×T	90	diff_sum × 2 ≤ 90，即 diff_sum ≤ 45
每像素条目	24	1 current + 23 history
前景判定	match_count > 2	需 ≥3 个历史帧匹配
结果粒度	32 bit/批	每 32 像素触发一次 READY
AXI 数据宽	64-bit	每 beat 2 条目
AXI 地址宽	38-bit	匹配 FIC_0
AXI ID 宽	4-bit	匹配 COREAXI4INTERCONNECT
