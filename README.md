# 7-Segment Display Control Library

Direct hardware control for the BD0027 7-segment displays, based on firmware reverse engineering.

## Overview

This library provides low-level control over the two 7-segment LED digits on the BD0027 board. It allows custom firmware (shellcode) to display arbitrary numbers and patterns without relying on the original firmware's display routines.

## Hardware Details

**Microcontroller:** PY32F030K28 (ARM Cortex-M0+)
**Display:** Two 7-segment digits with multiplexed common cathode
**Refresh Method:** Time-multiplexing via software (no timer interrupts required)

### Pin Mapping

Based on schematic analysis and firmware reverse engineering:

| Segment | GPIO Pin | Function |
|---------|----------|----------|
| A | PB8 | Top horizontal |
| B | PB7 | Top-right vertical |
| C | PB6 | Bottom-right vertical |
| D | PB5 | Bottom horizontal |
| E | PB3 | Bottom-left vertical |
| F | PA15 | Top-left vertical |
| G | PA12 | Middle horizontal |
| DP | PA11 | Decimal point |

**Digit Select (Multiplexing):**
- PA10 (TOP_LEDS) - Enables top digit
- PA9 (BOT_LEDS) - Enables bottom digit

## Quick Start

### 1. Simple Number Display

```c
#include "display_7seg.h"

void main(void) {
    display_7seg_init();
    display_7seg_number(42, 5000);  // Show "42" for 5 seconds
}
```

### 2. Continuous Display (Shellcode)

```c
#include "display_7seg.h"

void shellcode_main(void) {
    display_7seg_init();

    while (1) {
        // Display "69" continuously
        display_7seg_digit(6, true, false);   // Top: 6
        for (volatile uint32_t i = 0; i < 1000; i++);

        display_7seg_digit(9, false, false);  // Bottom: 9
        for (volatile uint32_t i = 0; i < 1000; i++);
    }
}
```

## Files

- `display_7seg.h` - Header file with API declarations
- `display_7seg.c` - Implementation
- `example_usage.c` - Complete examples
- `README.md` - This file

## API Reference

See comments in `display_7seg.h` for full API documentation.

Key functions:
- `display_7seg_init()` - Initialize GPIO (call first!)
- `display_7seg_number(num, duration)` - Display 0-99 with multiplexing
- `display_7seg_digit(digit, pos, dp)` - Display single digit
- `display_7seg_clear()` - Turn off display

## Examples

See `example_usage.c` for complete working examples.

## Building

```bash
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Os \
    display_7seg.c your_code.c -o output.elf
```

## References

- ../bd0027-decompiled/DISPLAY_ANALYSIS_COMPLETE.md - Full analysis
- ../DISPLAY_MAPPING_COMPLETE.md - Pin mappings
