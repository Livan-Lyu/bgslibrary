# BGS Cape Register & Address Map

## FIC3 Fabric Address Space

```
FIC3 base: 0x41000000 (FIC3_INITIATOR CoreAPB3, MADDR_BITS=28, UPR=6)
Slot size: 1 MB

FIC3 Slot  Offset      Bus Address   Connected To
───────────────────────────────────────────────────
  0        0x000000    0x41000000     (disabled)
  1        0x100000    0x41100000     CAPE_APB_MTARGET → CAPE (CoreAPB3_CAPE)
  2        0x200000    0x41200000     (disabled — no CSI hardware)
  3–15      …          0x41300000+    (disabled)
```

## CAPE Internal Address Map (CoreAPB3_CAPE)

```
CoreAPB3_CAPE: MADDR_BITS=24, UPR_NIBBLE_POSN=5
Each slot: 2^(24-5) = 512 KB

Slot  Offset      Bus Address   Peripheral
────────────────────────────────────────────────
  0    0x000000    0x41100000     PWM_0
  1    0x080000    0x41180000     P8_GPIO_UPPER   (P8 pins 31–46, 16 GPIOs)
  2    0x100000    0x41200000     P9_GPIO         (P9 pins 11–42, 20 GPIOs)
  3    0x180000    0x41280000     cape_regs ★     (custom ctrl/status + pixel_proc)
  4    0x200000    0x41300000     PWM_1
  5    0x280000    0x41380000     PWM_2
```

## cape_regs Internal Decoding (slot 3, paddr[7] selects sub-module)

```
paddr[7]  Offset      Bus Address   Module
────────────────────────────────────────────────
  0       0x00–0x7F   0x412800xx    apb_ctrl_status
  1       0x80–0xFF   0x4128008x    pixel_proc
```

---

## apb_ctrl_status Registers

**Base: 0x41280000**

| Offset | Name       | Access | Reset      | Description                    |
|--------|------------|--------|------------|--------------------------------|
| 0x00   | CONTROL_0  | RO     | 0xDEADBEEF | Magic number / alive check     |
| 0x10   | CONTROL_1  | RW     | 0x00000000 | General-purpose control output |
| 0x20   | STATUS     | RO     | —          | External status input          |

### Control Signals (Verilog ports)

| Signal   | Width | Direction | Description                     |
|----------|-------|-----------|---------------------------------|
| control  | 32    | output    | Drives CONTROL_1 value to fabric |
| status   | 32    | input     | Read back via STATUS register   |

---

## pixel_proc Registers

**Base: 0x41280080**

| Offset | Name    | Access | Reset      | Description                                           |
|--------|---------|--------|------------|-------------------------------------------------------|
| 0x80   | A_CTRL  | RW     | 0x00000000 | bit[0]=start engine, bit[7]=ack IRQ (write 1 to clear) |
| 0x84   | A_STAT  | RO     | 0x00000000 | bit[0]=busy, bit[1]=irq, bit[2]=done                 |
| 0x88   | A_SRC_LO| RW     | 0x00000000 | Source address lower 32 bits                          |
| 0x8C   | A_SRC_HI| RW     | 0x00000000 | Source address upper 32 bits                          |
| 0x90   | A_CNT   | RW     | 0x00000000 | Total pixel-pair count to process                     |
| 0x94   | A_RES   | RO     | 0x00000000 | Result bitmask (32 pixels per interrupt batch)        |
| 0x98   | A_DBG   | RO     | 0xDEADBEEF | Debug / alive check                                   |

### pixel_proc Interrupt

| Signal | Source Domain | Description                      |
|--------|---------------|----------------------------------|
| irq    | ACLK→PCLK sync| Asserted when 32-pixel batch completes. Write A_CTRL[7]=1 to acknowledge. |

### pixel_proc Operation Flow

```
1. Write A_SRC_LO, A_SRC_HI → set source address in DDR
2. Write A_CNT → set number of pixel pairs to process
3. Write A_CTRL[0]=1 → start DMA read engine
4. Poll A_STAT[0] or wait for IRQ
5. Read A_RES → 32-bit mask (1=foreground pixel)
6. Write A_CTRL[7]=1 → acknowledge, continue next batch
7. Repeat until A_STAT[2]=1 (done) or A_CTRL[0]=0 (stop)
```

### pixel_proc Algorithm Summary

- Reads 12 pixel pairs (PPR=12) per AXI burst from DDR
- Color distance threshold: THR=90 (9-bit sum-of-differences)
- Majority threshold: MTH=2 (≥2 neighbor pixels must be foreground)
- Output: 32-bit bitmask, 1 bit per pixel-pair processed

---

## AXI4 DMA Address Range

```
AXI Crossbar (COREAXI4INTERCONNECT): 1 initiator, 1 target

cape_regs:AXI4_INITIATOR → XBAR_0:AXI4minitiator0
XBAR_0:AXI4mtarget0       → CAPE:AXI4mtarget0 → FIC_0_AXI4_TARGET

Target address range: 0x80000000 – 0xDFFFFFFF (1536 MB DDR)
AXI data width: 64-bit, ID width: 4-bit, address width: 38-bit
```

---

## Linux Device Tree

### cape-gpios.dtso
```
fabric-bus@40000000 {
    gpio@41180000 { compatible="microchip,coregpio-rtl-v3"; … };  // P8 upper
    gpio@41200000 { compatible="microchip,coregpio-rtl-v3"; … };  // P9
};
```

### verilog-cape.dtso
```
fabric-bus@40000000 {
    pixel-proc@41280000 {
        compatible = "generic-uio";
        reg = <0x0 0x41280000 0x0 0x1000>;
        linux,uio-name = "pixel-proc";
    };
};
```

Userspace access via `/dev/uio*` — mmap the 4KB region at 0x41280000 to access all cape_regs.
