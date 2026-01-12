# 7-Segment Display Pin Mapping

## Overview
This document describes the correct pin mapping for the 4-digit 7-segment display based on hardware research.

## Display Layout
```
[Position 0] [Position 1]    (Top row)
[Position 2] [Position 3]    (Bottom row)
```

## Digit Cathode Control Pins

The display uses **cathode multiplexing** where each digit position has a dedicated cathode pin. To enable a digit, pull its cathode **LOW** (active LOW). All inactive digits should have their cathodes **HIGH**.

| Position | Location       | Cathode Pin | GPIO   | Active State |
|----------|----------------|-------------|--------|--------------|
| 0        | Top-left       | PB0         | GPIOB  | LOW          |
| 1        | Top-right      | PB1         | GPIOB  | LOW          |
| 2        | Bottom-left    | PB2         | GPIOB  | LOW          |
| 3        | Bottom-right   | PA8         | GPIOA  | LOW          |

## Segment Pins (Shared Across All Digits)

These pins control which segments light up. The segments are **common cathode** (active HIGH).

| Segment | Description         | Pin   | GPIO   | Active State |
|---------|---------------------|-------|--------|--------------|
| A       | Top horizontal      | PB8   | GPIOB  | HIGH         |
| B       | Top-right vertical  | PB7   | GPIOB  | HIGH         |
| C       | Bottom-right vert.  | PB6   | GPIOB  | HIGH         |
| D       | Bottom horizontal   | PB5   | GPIOB  | HIGH         |
| E       | Bottom-left vert.   | PB3   | GPIOB  | HIGH         |
| F       | Top-left vertical   | PA15  | GPIOA  | HIGH         |
| G       | Middle horizontal   | PA12  | GPIOA  | HIGH         |

```
     A
    ───
  F│   │B
    ─G─
  E│   │C
    ───
     D
```

## Leading "1" Digit (Special)

The display has a special **leading "1"** that can be displayed on either the top or bottom row. This is controlled by an anode pin (PA11) shared with the cathode multiplexing system.

| Function      | Pin  | GPIO   | Control Method |
|---------------|------|--------|----------------|
| Leading "1" anode | PA11 | GPIOA  | HIGH to enable |

**How it works:**
- The leading "1" is wired to span across an **entire row** (both digit positions)
- Set PA11 **HIGH** to enable the leading "1" anode
- Pull **BOTH cathodes in the target row LOW** to display it:
  - **Top row (positions 0-1):** PB0 LOW + PB1 LOW
  - **Bottom row (positions 2-3):** PB2 LOW + PA8 LOW
- This allows displaying numbers like "123" or "1.99" with the leading "1" as a hundreds digit

### Examples:
- **Show "1" on top row:**     PA11=HIGH, PB0=LOW, PB1=LOW (all other cathodes HIGH)
- **Show "1" on bottom row:**  PA11=HIGH, PB2=LOW, PA8=LOW (all other cathodes HIGH)

## Multiplexing Strategy

To display multiple digits simultaneously, use time-multiplexing:

1. Display digit 0 (top-left): segments + PB0=LOW, all others HIGH
2. Wait ~5ms
3. Display digit 1 (top-right): segments + PB1=LOW, all others HIGH
4. Wait ~5ms
5. Display digit 2 (bottom-left): segments + PB2=LOW, all others HIGH
6. Wait ~5ms
7. Display digit 3 (bottom-right): segments + PA8=LOW, all others HIGH
8. Wait ~5ms
9. Repeat from step 1

This creates a 50Hz refresh rate (20ms per full cycle) which appears solid to the human eye.

## Example: Display "1234"

### Multiplexed Display Loop:
```c
// Cycle 1: Show "1" in position 0
GPIOA_ODR = PA15 | PA12;  // Segments F + G (digit "1")
GPIOB_ODR = PB7 | PB6 | PB1 | PB2 | PA8_mask;  // Segments B,C + disable pos 1,2,3
GPIOB_ODR &= ~PB0;  // Enable position 0
delay(5ms);

// Cycle 2: Show "2" in position 1
GPIOA_ODR = PA12;  // Segment G
GPIOB_ODR = PB8 | PB7 | PB5 | PB3 | PB0 | PB2 | PA8_mask;  // Segments A,B,D,E,G + disable pos 0,2,3
GPIOB_ODR &= ~PB1;  // Enable position 1
delay(5ms);

// Cycle 3: Show "3" in position 2
GPIOA_ODR = PA12;  // Segment G
GPIOB_ODR = PB8 | PB7 | PB6 | PB5 | PB0 | PB1 | PA8_mask;  // Segments A,B,C,D,G + disable pos 0,1,3
GPIOB_ODR &= ~PB2;  // Enable position 2
delay(5ms);

// Cycle 4: Show "4" in position 3
GPIOA_ODR = PA15 | PA12;  // Segments F,G
GPIOB_ODR = PB7 | PB6 | PB0 | PB1 | PB2;  // Segments B,C + disable pos 0,1,2
GPIOA_ODR &= ~PA8;  // Enable position 3
delay(5ms);

// Repeat...
```

## Code Reference

See `display_7seg.c` for complete implementation with helper functions:

- `display_7seg_digit(digit, position, decimal_point)` - Show one digit
- `display_7seg_all_digits(digits[], decimal_points[], duration)` - Multiplex all 4
- `display_7seg_number(number, duration)` - Show 2-digit number on bottom row
- `display_leading_one(position, show)` - Control the leading "1"

## Testing

Use these test functions to verify pin mapping:

```c
// Test digit positions (shows "0123")
display_7seg_position_test(5000);

// Test leading "1" in all positions
display_leading_one_test(1000);

// Test all segments on position 2
display_7seg_segment_test(2);
```

## GPIO Configuration

All display pins should be configured as:
- **Mode:** Output (0b01)
- **Type:** Push-pull (0)
- **Speed:** High (0b11)
- **Pull-up/down:** None (0b00)

### Initialization State:
- All segment pins: **LOW** (segments off)
- All cathode pins: **HIGH** (all digits disabled)
- Leading "1" anode (PA11): **LOW** (off)

## PY32-Specific Hardware Notes

### BRR Register (Important!)

The PY32F0xx series has a **separate BRR (Bit Reset Register)** at offset `0x28` instead of using BSRR[31:16] for clearing pins.

**What this means:**
- ✅ **BSRR (0x18):** Only bits [15:0] work - sets pins HIGH
- ❌ **BSRR[31:16]:** Does **nothing** on PY32 (hardware limitation)
- ✅ **BRR (0x28):** Use this to set pins LOW atomically

**Example:**
```c
// Set PA8 HIGH - use BSRR
GPIOA_BSRR = (1 << 8);  // Works!

// Set PA8 LOW - DON'T use BSRR upper bits
GPIOA_BSRR = (1 << (8 + 16));  // Does NOTHING on PY32!

// Set PA8 LOW - use BRR instead
GPIOA_BRR = (1 << 8);  // Works!

// Or use read-modify-write on ODR
uint32_t odr = GPIOA_ODR;
odr &= ~(1 << 8);
GPIOA_ODR = odr;  // Also works!
```

**Why it matters:**
- This is different from STM32 where BSRR[31:16] is supported
- Code written for STM32 may not work correctly on PY32
- Always use BRR or read-modify-write for clearing pins

**Our implementation:**
The display code uses **read-modify-write on ODR** which works correctly on PY32 and preserves other pin states.
