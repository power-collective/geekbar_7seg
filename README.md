# 7-Segment Display Shellcode Generator

<img src="geekbar_demo.jpg" width="400" alt="GeekBar demo board with 7-segment display">

Python tool that generates, compiles, and injects ARM Thumb shellcode to directly control the 4-digit 7-segment display on the BD0027 board via GDB/Black Magic Probe.

## Hardware Details

**Board:** BD0027 ([schematic/RE available here](https://github.com/schlae/VapeRE/tree/main/BD0027))
**Microcontroller:** PY32F030K28 (ARM Cortex-M0+)
**Display:** 4 digit positions across two dual 7-segment displays, cathode multiplexed
**Refresh Method:** Time-multiplexing via software (no timer interrupts required)

### Display Layout

```
[0] [1]    (Top row)
[2] [3]    (Bottom row)
```

An optional **leading "1"** can be displayed on either row via a dedicated anode pin (PA11).

### Pin Mapping

**Digit Cathode Pins (Active LOW):**

| Position | Location     | Pin  |
|----------|--------------|------|
| 0        | Top-left     | PB0  |
| 1        | Top-right    | PB1  |
| 2        | Bottom-left  | PB2  |
| 3        | Bottom-right | PA8  |

**Segment Pins (Active HIGH, shared across all digits):**

| Segment | Function            | Pin  |
|---------|---------------------|------|
| A       | Top horizontal      | PB8  |
| B       | Top-right vertical  | PB7  |
| C       | Bottom-right vert.  | PB6  |
| D       | Bottom horizontal   | PB5  |
| E       | Bottom-left vert.   | PB3  |
| F       | Top-left vertical   | PA15 |
| G       | Middle horizontal   | PA12 |

**Leading "1" anode:** PA11 (HIGH to enable; pull both cathodes in target row LOW to display)

See `PIN_MAPPING.md` for full details including PY32-specific BRR register notes.

## Usage

Connect a Black Magic Probe (or compatible GDB server) to the target, then:

```bash
# Display a 2-digit number on the bottom row for 5 seconds
python3 display_shellcode_generator.py number 42 bottom

# Display arbitrary digits at all 4 positions
python3 display_shellcode_generator.py digits 1 2 3 4

# Display digits with a leading "1" at a given position (0-3)
python3 display_shellcode_generator.py leading1 0 2 3
```

The script generates C from the template, compiles it to ARM Thumb binary, injects it into MCU RAM, and sets the PC to execute it.

## Requirements

- Python 3
- `arm-none-eabi-gcc` (for ARM Thumb compilation)
- Black Magic Probe or TCP GDB server (e.g. `localhost:2331`)

## Files

| File | Description |
|------|-------------|
| `display_shellcode_generator.py` | Main script: generate, compile, inject, run |
| `display_shellcode_gpio_template.c` | C template compiled into shellcode |
| `display_shellcode_template.h` | Helper macros and RAM buffer addresses |
| `pyrsp_py32f003.py` | MCU communication library (GDB RSP) |
| `inject_only.py` | Inject a pre-compiled `.bin` file |
| `test_inject_only.py` | Test injection script |
| `PIN_MAPPING.md` | Detailed pin reference |

## References

- [BD0027 schematic reverse engineering (schlae/VapeRE)](https://github.com/schlae/VapeRE/tree/main/BD0027)
