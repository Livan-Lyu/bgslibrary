软件接口总结
一、寄存器映射
物理基地址：0x41100000（CAPE APB slot）

地址	模块	关键寄存器
0x41100000	apb_ctrl_status	CONTROL_0 (RO, 0xDEADBEEF), CONTROL_1 (RW, 0x10)
0x41100080	pixel_proc	见下表
pixel_proc 寄存器（偏移 0x80 起，paddr[7]=1 访问）：

偏移	名称	访问	位段	说明
0x80	CONTROL	RW	[0] START<br>[7] ACK	写 0x01 启动；写 0x80 通知 CPU 已取走结果
0x84	STATUS	RO	[0] BUSY<br>[1] READY<br>[2] DONE	硬件自动更新
0x8C	PIXEL_DATA	WO	[23:0] RGB	PDMA 连续写入此地址推送像素数据
0x90	PIXEL_COUNT	RW	[31:0]	总像素数 N
0x94	RESULT	RO	[31:0]	打包 32 个 foreground bit（[0]=第一个像素）
0x98	DEBUG	RO	[31:0]	固定返回 0xDEADBEEF
二、DDR 数据排布（CPU 写入）

每像素 = 1 当前 + 23 历史 = 24 条目 × 4 字节 = 96 字节

地址 +0x00: pixel0_curr    [R(7:0), G(15:8), B(23:16), 0x00]
地址 +0x04: pixel0_hist0   [R, G, B, 0x00]
地址 +0x08: pixel0_hist1   ...
地址 +0x5C: pixel0_hist22
地址 +0x60: pixel1_curr    ...
...
总大小 = N × 96 字节
三、CPU 操作流程

1. 分配 DDR 缓冲区，按格式写入 N 个像素数据
2. mmap /dev/mem (0x41100000) 或 /dev/uioX

3. 写 PIXEL_COUNT    (offset 0x90/4 = 36)  = N
4. 写 CONTROL        (offset 0x80/4 = 32)  = 0x01    // START

5. 配置并启动 PDMA:
   - src_addr     = DDR 缓冲区物理地址
   - dst_addr     = 0x4110008C (PIXEL_DATA, fixed)
   - src_inc      = 1 (递增)
   - dst_inc      = 0 (固定)
   - transfer_len = N × 96 字节

6. 循环处理结果:
   while (像素未处理完) {
       while ((STATUS & 0x02) == 0);   // 等 READY
       uint32_t result = read(RESULT);  // 偏移 0x94
       write(CONTROL, 0x80);            // ACK
       write(CONTROL, 0x00);            // 清除 ACK
       for (i = 0; i < 32; i++)
           处理 result 的第 i 个 bit
   }
7. 等 STATUS & 0x04 (DONE)
8. write(CONTROL, 0x00)                // 清除 START
四、中断
pixel_proc 的 IRQ 通过 MSS_INT_F2M[8] 到达 CPU（Linux IRQ 号取决于设备树和中断控制器映射）。

触发条件：32 个结果 bit 打包完成（READY=1）
清除条件：CPU 写 CONTROL[7]=1 (ACK)
也可不用中断，纯轮询 STATUS[1]
五、Device Tree Overlay
pixel_proc UIO 节点：


pixel_proc: pixel-proc@41100080 {
    compatible = "generic-uio";
    reg = <0x0 0x41100080 0x0 0x20>;
    status = "okay";
    linux,uio-name = "pixel-proc";
};
六、C 伪代码

volatile uint32_t *regs = mmap(NULL, 0x100, PROT_RW, MAP_SHARED, fd, 0x41100000);
// regs[0] = CONTROL     (word offset 0x80/4 = 32)
// regs[1] = STATUS      (word offset 0x84/4 = 33)
// regs[3] = PIXEL_DATA  (word offset 0x8C/4 = 35)
// regs[4] = PIXEL_COUNT (word offset 0x90/4 = 36)
// regs[5] = RESULT      (word offset 0x94/4 = 37)

regs[36] = total_pixels;      // PIXEL_COUNT
regs[32] = 0x01;              // START

// 配置 PDMA: dst=0x4110008C, src=ddr_buf, len=N*96
pdma_start(...);

uint32_t done = 0;
while (!done) {
    while (!(regs[33] & 0x02));   // wait READY
    uint32_t r = regs[37];        // read RESULT
    regs[32] = 0x80;              // ACK
    regs[32] = 0x00;
    for (int i = 0; i < 32; i++) {
        if ((r >> i) & 1) { /* pixel i 是前景 */ }
    }
    if (regs[33] & 0x04) done = 1; // DONE
}
regs[32] = 0x00;                 // clear START
七、固定参数（硬件硬编码）
参数	值
阈值 4.5×T	90（即 diff_sum ≤ 45）
每像素条目数	24（1 current + 23 history）
前景判定	match_count > 2
结果打包粒度	32 bit/批
每条目大小	4 字节（24-bit RGB + 8-bit padding）
