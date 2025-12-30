# GPIO Blink Shellcode Generator

**Dynamic ARM Thumb code generation for PY32F030 GPIO control**

This tool generates custom ARM Thumb machine code to blink GPIO pins, injects it into RAM, and executes it on the microcontroller. Perfect for testing hardware connections, creating temporary blink patterns, or overriding firmware behavior temporarily.

## Features

- **Runtime code generation** - Specify GPIO pins and behavior at runtime
- **C-based generation** - Uses C code compiled to ARM Thumb for maintainability
- **Automatic injection** - Writes to RAM and jumps to execute
- **Self-resetting** - Automatically resets MCU to restore firmware after completion
- **Precise timing** - 1 Hz blink rate with configurable system clock
- **No firmware modification** - Runs entirely from RAM

## Quick Start

### Generate Shellcode

```python
from utils.gpio_blink_generator import generate_gpio_shellcode

# Generate code to blink PA5 three times, then set PB1 low for 10s
shellcode = generate_gpio_shellcode(
    blink_pin="PA5",
    final_pin="PB1",
    num_blinks=3
)

print(f"Generated {len(shellcode)} bytes of ARM code")
```

### Inject and Run

```python
from pyrsp_py32f003 import PY32F003
from utils.gpio_blink_generator import generate_gpio_shellcode, inject_and_run_blink

shellcode = generate_gpio_shellcode("PA5", "PB1", num_blinks=3)

with PY32F003() as mcu:
    mcu.prepare_for_interactive_debug(reset_first=True)
    inject_and_run_blink(mcu, shellcode)
```

### Command Line Usage

```bash
# Generate and save shellcode
python utils/gpio_blink_generator.py PA5 PB1 --blinks 5 --save-output ./build

# See all options
python utils/gpio_blink_generator.py --help
```

## How It Works

### 1. **C Code Generation**

The script generates optimized C code based on your parameters:

```c
void shellcode_entry(void) {
    // Enable GPIO clocks
    RCC->IOPENR |= (1 << port_bit);

    // Configure pins as outputs
    GPIOx->MODER = ...;

    // Blink loop
    for (int i = 0; i < NUM_BLINKS; i++) {
        GPIOx->BSRR = (1 << pin);     // Set high
        delay_ms(500);
        GPIOx->BSRR = (1 << (pin+16)); // Set low
        delay_ms(500);
    }

    // Set final pin low
    GPIOy->BSRR = (1 << (final_pin+16));
    delay_ms(10000);

    // Reset MCU
    SCB->AIRCR = 0x05FA0004;
}
```

### 2. **Compilation**

Uses `arm-none-eabi-gcc` to compile to ARM Thumb code:

```bash
arm-none-eabi-gcc \
  -mcpu=cortex-m0plus \
  -mthumb \
  -O2 \
  -nostdlib \
  -T shellcode.ld \
  shellcode.c \
  -o shellcode.elf
```

### 3. **Binary Extraction**

Extracts raw binary from ELF:

```bash
arm-none-eabi-objcopy -O binary shellcode.elf shellcode.bin
```

### 4. **Injection**

Writes shellcode to RAM and sets PC:

```python
mcu.write(0x20000A00, shellcode)  # Write to RAM
mcu.set_pc(0x20000A01)             # Set PC (Thumb mode)
mcu.resume()                        # Execute
```

## Parameters

### `generate_gpio_shellcode()`

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `blink_pin` | str | GPIO pin to blink (e.g., "PA5") | Required |
| `final_pin` | str | Pin to set low after blinking | Required |
| `num_blinks` | int | Number of blink cycles | 3 |
| `ram_address` | int | Target RAM address | 0x20000A00 |
| `sysclk_hz` | int | System clock frequency | 24000000 |
| `save_output` | str | Directory to save build files | None |

### GPIO Pin Format

- **Format**: `P<PORT><NUMBER>` (case insensitive)
- **Examples**: `PA5`, `pb12`, `PC0`
- **Valid ports**: A, B, C, D, F
- **Valid pins**: 0-15

## Behavior Sequence

1. **Enable GPIO clocks** for both ports
2. **Configure pins** as outputs
3. **Blink loop**:
   - Set blink pin HIGH
   - Delay 500ms
   - Set blink pin LOW
   - Delay 500ms
   - Repeat N times
4. **Set final pin LOW**
5. **Delay 10 seconds**
6. **Software reset** MCU (returns to firmware)

## Examples

### Basic Blink

```python
# Blink PA5 3 times
shellcode = generate_gpio_shellcode("PA5", "PB1", num_blinks=3)
```

**Timeline**:
- 0.0s: PA5 HIGH
- 0.5s: PA5 LOW
- 1.0s: PA5 HIGH
- 1.5s: PA5 LOW
- 2.0s: PA5 HIGH
- 2.5s: PA5 LOW
- 3.0s: PB1 LOW
- 13.0s: MCU reset

### Fast Indicator

```python
# Quick 10-blink test pattern
shellcode = generate_gpio_shellcode("PA0", "PA1", num_blinks=10)
```

### Hardware Test Sequence

```python
# Test multiple GPIO pins
pins_to_test = ["PA0", "PA1", "PA2", "PA3", "PA4", "PA5"]

for pin in pins_to_test:
    print(f"Testing {pin}...")
    shellcode = generate_gpio_shellcode(pin, "PB0", num_blinks=2)

    with PY32F003() as mcu:
        mcu.prepare_for_interactive_debug(reset_first=True)
        inject_and_run_blink(mcu, shellcode)
        time.sleep(15)  # Wait for completion
```

### Custom System Clock

```python
# If you've changed HSI to 8 MHz
shellcode = generate_gpio_shellcode(
    "PA5", "PB1",
    num_blinks=3,
    sysclk_hz=8_000_000  # 8 MHz instead of default 24 MHz
)
```

## Directory Structure

```
utils/
├── gpio_blink_generator.py    # Main generator script
└── pyrsp_py32f003.py           # MCU interface

examples/
└── gpio_blink_demo.py          # Usage examples

build/                          # Generated artifacts (optional)
├── shellcode.c                 # Generated C code
├── shellcode.ld                # Linker script
├── shellcode.elf               # ELF binary
└── shellcode.bin               # Raw binary
```

## Requirements

### Software

- Python 3.7+
- `arm-none-eabi-gcc` toolchain
- pyOCD or Black Magic Probe (for MCU connection)

### Installation

```bash
# macOS
brew install arm-none-eabi-gcc

# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi

# Arch Linux
sudo pacman -S arm-none-eabi-gcc
```

### Python Dependencies

```bash
pip install pyserial  # For GDB remote protocol
```

## RAM Usage

- **Default address**: `0x20000A00` (near top of 8KB SRAM)
- **Typical size**: 200-400 bytes depending on configuration
- **Safe range**: `0x20000A00` - `0x20001F00` (avoids stack/heap)

## Timing Accuracy

Delay loops are calibrated for the specified system clock. Accuracy depends on:

- **Compiler optimization** - `-O2` provides good balance
- **System clock accuracy** - HSI is ±1% on PY32F030
- **Interrupt latency** - Minimal (no interrupts in shellcode)

Expected accuracy: **±2%** at default 24 MHz HSI

## Debugging

### View Generated Code

```python
shellcode = generate_gpio_shellcode(
    "PA5", "PB1", 3,
    save_output="./build/debug"
)

# Check build/debug/shellcode.c for C source
# Check build/debug/shellcode.elf with arm-none-eabi-objdump
```

### Disassemble Binary

```bash
arm-none-eabi-objdump -D -m arm -b binary shellcode.bin
```

### Check Execution

```python
with PY32F003() as mcu:
    inject_and_run_blink(mcu, shellcode)

    # Halt and check PC
    time.sleep(1)
    mcu.halt()

    # Read PC register
    all_regs = mcu.rsp.fetch(b'g')
    reg_bytes = bytes.fromhex(all_regs.decode('ascii'))
    pc = int.from_bytes(reg_bytes[60:64], byteorder='little')

    print(f"Current PC: 0x{pc:08X}")
    mcu.resume()
```

## Troubleshooting

### Compilation Fails

**Error**: `arm-none-eabi-gcc: command not found`

**Solution**: Install ARM GCC toolchain (see Requirements)

### Wrong Blink Timing

**Problem**: Blinks too fast or too slow

**Solution**: Verify system clock frequency and adjust `sysclk_hz` parameter

### MCU Doesn't Reset

**Problem**: Shellcode runs but MCU stays in code

**Solution**: Check that final delay completes. May need to manually reset:

```python
mcu.reset()
```

### GPIO Doesn't Toggle

**Problem**: Pin configured but no voltage change

**Checklist**:
- Verify pin name (PA5, not A5)
- Check if pin is available (not used by SWD, etc.)
- Confirm GPIO clock is enabled (handled by shellcode)
- Check pin isn't held by external hardware

## Advanced Usage

### Inject to Flash

```python
# Write to flash instead of RAM (requires flash unlock)
FLASH_ADDRESS = 0x08007000  # End of firmware

shellcode = generate_gpio_shellcode("PA5", "PB1", 3, ram_address=FLASH_ADDRESS)

# Unlock flash, write, lock
# ... (flash programming code)
```

### Chain Multiple Shellcodes

```python
# Generate multiple shellcodes and call sequentially
code1 = generate_gpio_shellcode("PA0", "PA1", 2, ram_address=0x20000A00)
code2 = generate_gpio_shellcode("PA2", "PA3", 2, ram_address=0x20000B00)

# Modify code1 to jump to code2 instead of reset
# ... (requires assembly modification)
```

### Call from Firmware

```C
// From your own firmware, call the shellcode
typedef void (*shellcode_func_t)(void);
shellcode_func_t shellcode = (shellcode_func_t)0x20000A01;  // +1 for Thumb

shellcode();  // Execute
```

## Safety Considerations

1. **Watchdog**: Shellcode runs long enough to trigger IWDG. May need to disable watchdog first.

2. **Interrupts**: Shellcode doesn't disable interrupts. Firmware ISRs may interfere.

3. **Clock Changes**: If firmware changes system clock, timing will be wrong.

4. **Stack**: Shellcode uses very little stack (<64 bytes). Safe in most cases.

5. **Reset**: Software reset via AIRCR is safe and equivalent to hardware reset.

## License

This tool is part of the py32-board-tools project.

## See Also

- `utils/inject_busy_loop.py` - Simple infinite loop injection
- `utils/inject_loop_breakpoint.py` - Loop with breakpoint for debugging
- PY32F030 reference manual - GPIO and RCC details
