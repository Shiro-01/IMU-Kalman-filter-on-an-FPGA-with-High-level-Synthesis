# ce_pulse_gen — Product Guide

**Free-Running Clock-Enable Pulse Generator**
Target device: Xilinx Basys 3 | VHDL-2008 | Vivado 2026.1  
Author: Abdelrahman Hewala (Shiro)

---

## 1. Overview

`ce_pulse_gen` is a free-running clock-enable pulse generator, derived from the
system clock by integer division. In this project.

---

## 2. Interface

### 2.1 Generics

| Generic       | Type    | Default     | Description |
|----------------|---------|-------------|-------------|
| `CLK_FREQ_HZ`  | natural | 100,000,000 | System clock frequency in Hz. |
| `TARGET_HZ`    | natural | 48,000      | Desired pulse rate in Hz. |

### 2.2 Ports

| Port    | Dir | Width | Description |
|---------|-----|-------|-------------|
| `clk`   | in  | 1     | System clock. |
| `rst_n` | in  | 1     | Active-low synchronous reset. |
| `ce`    | out | 1     | Single-cycle pulse at approximately `TARGET_HZ`. |

---

## 3. Achieved Rate

```
DIV_MAX = (CLK_FREQ_HZ / TARGET_HZ) - 1
```

`CLK_FREQ_HZ / TARGET_HZ` uses integer division, so the achieved rate is subject
to whatever rounding this leaves behind. At the default generics (100 MHz,
48 kHz requested):

```
achieved_rate = 48.007 kHz
```

---

## 4. Known Limitations

**Rate accuracy.** The achieved rate depends on integer division and can differ
from `TARGET_HZ`. Not suitable where an exact frequency is required.

---

## 5. Revision

| Rev  | Date       | Notes |
|------|------------|-------|
| 0.01 | 2026-08-06 | Initial implementation. |