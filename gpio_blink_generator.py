#!/usr/bin/env python3
"""
GPIO Blink Code Generator for PY32F030

Generates ARM Thumb shellcode that:
1. Blinks a specified GPIO pin N times (1 second on, 1 second off)
2. Sets another GPIO pin low for 10 seconds
3. Resets the MCU to return to firmware

The code is generated as C, compiled with arm-none-eabi-gcc, and can be
injected into RAM or flash for execution.

Usage:
    from utils.gpio_blink_generator import generate_gpio_shellcode, inject_and_run_blink

    # Generate code to blink PA5 3 times, then set PB1 low for 10s
    shellcode = generate_gpio_shellcode("PA5", "PB1", num_blinks=3)

    # Inject and run
    inject_and_run_blink(mcu, shellcode)
"""

import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Tuple, Optional


# =============================================================================
# Hardware Definitions for PY32F030
# =============================================================================

# RCC (Reset and Clock Control)
RCC_BASE = 0x40021000
RCC_IOPENR = RCC_BASE + 0x34  # GPIO clock enable register

# GPIO Bases
GPIO_BASES = {
    'A': 0x50000000,
    'B': 0x50000400,
    'C': 0x50000800,
    'D': 0x50000C00,
    'F': 0x50001400,
}

# System Control Block (for reset)
SCB_AIRCR = 0xE000ED0C  # Application Interrupt and Reset Control Register

# Default system clock (HSI)
DEFAULT_SYSCLK_HZ = 24_000_000  # 24 MHz HSI


# =============================================================================
# GPIO Pin Parsing
# =============================================================================

def parse_gpio_pin(pin_spec: str) -> Tuple[str, int]:
    """
    Parse GPIO pin specification like "PA5" or "PB12"

    Args:
        pin_spec: Pin name (e.g., "PA5", "PB1")

    Returns:
        Tuple of (port_letter, pin_number)

    Example:
        >>> parse_gpio_pin("PA5")
        ('A', 5)
    """
    match = re.match(r'P([A-F])(\d+)', pin_spec.upper())
    if not match:
        raise ValueError(f"Invalid GPIO pin: {pin_spec}. Expected format: PA5, PB12, etc.")

    port = match.group(1)
    pin = int(match.group(2))

    if port not in GPIO_BASES:
        raise ValueError(f"Invalid GPIO port: {port}. Valid ports: {list(GPIO_BASES.keys())}")

    if pin > 15:
        raise ValueError(f"Invalid pin number: {pin}. Must be 0-15")

    return (port, pin)


# =============================================================================
# C Code Generator
# =============================================================================

def generate_c_code(blink_port: str, blink_pin: int,
                   final_port: str, final_pin: int,
                   num_blinks: int,
                   sysclk_hz: int = DEFAULT_SYSCLK_HZ) -> str:
    """
    Generate C code for GPIO blinking shellcode

    Args:
        blink_port: Port letter for blinking pin ('A', 'B', etc.)
        blink_pin: Pin number (0-15)
        final_port: Port letter for final pin
        final_pin: Pin number (0-15)
        num_blinks: Number of times to blink
        sysclk_hz: System clock frequency in Hz

    Returns:
        C source code as string
    """

    blink_gpio_base = GPIO_BASES[blink_port]
    final_gpio_base = GPIO_BASES[final_port]

    # Calculate bit positions for RCC clock enable
    blink_clock_bit = ord(blink_port) - ord('A')
    final_clock_bit = ord(final_port) - ord('A')

    # Calculate delay loop iterations for timing
    # Rough estimate: 4 cycles per loop iteration at -O2
    cycles_per_ms = sysclk_hz // 1000
    iterations_per_ms = cycles_per_ms // 4

    c_code = f"""
/*
 * Auto-generated GPIO Blink Shellcode
 *
 * Blink: P{blink_port}{blink_pin}
 * Final: P{final_port}{final_pin}
 * Blinks: {num_blinks}
 */

#include <stdint.h>

// Hardware register addresses
#define RCC_BASE        0x{RCC_BASE:08X}U
#define RCC_IOPENR      (RCC_BASE + 0x34)

#define GPIO{blink_port}_BASE     0x{blink_gpio_base:08X}U
#define GPIO{final_port}_BASE     0x{final_gpio_base:08X}U

#define SCB_AIRCR       0x{SCB_AIRCR:08X}U

// GPIO register offsets
#define GPIO_MODER_OFFSET   0x00
#define GPIO_ODR_OFFSET     0x14
#define GPIO_BSRR_OFFSET    0x18

// Pin definitions
#define BLINK_PIN       {blink_pin}
#define FINAL_PIN       {final_pin}
#define NUM_BLINKS      {num_blinks}

// Timing (iterations for delay loops)
#define ITERATIONS_PER_MS    {iterations_per_ms}

// Register access macros
#define REG32(addr) (*(volatile uint32_t*)(addr))

// Delay function (busy loop)
static inline void delay_ms(uint32_t ms) __attribute__((always_inline));
static inline void delay_ms(uint32_t ms) {{
    volatile uint32_t count = ms * ITERATIONS_PER_MS;
    while (count--) {{
        __asm__ volatile ("nop");
    }}
}}

// Main shellcode entry point
__attribute__((naked, noreturn, section(".text.entry")))
void shellcode_entry(void) {{
    // Enable GPIO clocks
    uint32_t rcc_iopenr = REG32(RCC_IOPENR);
    rcc_iopenr |= (1U << {blink_clock_bit});  // Enable GPIO{blink_port}
    rcc_iopenr |= (1U << {final_clock_bit});  // Enable GPIO{final_port}
    REG32(RCC_IOPENR) = rcc_iopenr;

    // Small delay for clock to stabilize
    delay_ms(1);

    // Configure blink pin as output (MODER = 01 for output)
    volatile uint32_t *blink_moder = (volatile uint32_t*)(GPIO{blink_port}_BASE + GPIO_MODER_OFFSET);
    uint32_t moder_val = *blink_moder;
    moder_val &= ~(3U << (BLINK_PIN * 2));  // Clear mode bits
    moder_val |= (1U << (BLINK_PIN * 2));   // Set as output
    *blink_moder = moder_val;

    // Configure final pin as output
    volatile uint32_t *final_moder = (volatile uint32_t*)(GPIO{final_port}_BASE + GPIO_MODER_OFFSET);
    moder_val = *final_moder;
    moder_val &= ~(3U << (FINAL_PIN * 2));  // Clear mode bits
    moder_val |= (1U << (FINAL_PIN * 2));   // Set as output
    *final_moder = moder_val;

    // BSRR register for atomic bit set/reset
    volatile uint32_t *blink_bsrr = (volatile uint32_t*)(GPIO{blink_port}_BASE + GPIO_BSRR_OFFSET);
    volatile uint32_t *final_bsrr = (volatile uint32_t*)(GPIO{final_port}_BASE + GPIO_BSRR_OFFSET);

    // Blink loop
    for (int i = 0; i < NUM_BLINKS; i++) {{
        // Set pin high (BS - bit set)
        *blink_bsrr = (1U << BLINK_PIN);
        delay_ms(500);  // 0.5 second on

        // Set pin low (BR - bit reset, upper 16 bits)
        *blink_bsrr = (1U << (BLINK_PIN + 16));
        delay_ms(500);  // 0.5 second off
    }}

    // Set final pin low
    *final_bsrr = (1U << (FINAL_PIN + 16));

    // Wait 10 seconds
    delay_ms(10000);

    // Software reset via NVIC
    // AIRCR key (0x05FA) + SYSRESETREQ bit
    REG32(SCB_AIRCR) = 0x05FA0004;

    // Should never reach here, but infinite loop just in case
    while (1) {{
        __asm__ volatile ("nop");
    }}
}}
"""

    return c_code


# =============================================================================
# Linker Script Generator
# =============================================================================

def generate_linker_script(ram_address: int = 0x20000A00) -> str:
    """
    Generate linker script to place code at specific RAM address

    Args:
        ram_address: Target RAM address for shellcode

    Returns:
        Linker script as string
    """

    linker_script = f"""
/* Linker script for GPIO shellcode */

MEMORY
{{
    RAM (rwx) : ORIGIN = 0x{ram_address:08X}, LENGTH = 1K
}}

SECTIONS
{{
    .text : {{
        *(.text.entry)
        *(.text*)
        *(.rodata*)
    }} > RAM

    .data : {{
        *(.data*)
    }} > RAM

    .bss : {{
        *(.bss*)
        *(COMMON)
    }} > RAM

    /DISCARD/ : {{
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }}
}}

ENTRY(shellcode_entry)
"""

    return linker_script


# =============================================================================
# Compiler Invocation
# =============================================================================

def compile_shellcode(c_code: str, linker_script: str,
                     output_path: Optional[Path] = None) -> bytes:
    """
    Compile C code to ARM Thumb binary shellcode

    Args:
        c_code: C source code
        linker_script: Linker script
        output_path: Optional path to save .elf and .bin files

    Returns:
        Binary shellcode as bytes

    Raises:
        RuntimeError: If compilation fails
    """

    # Check for compiler
    try:
        result = subprocess.run(['arm-none-eabi-gcc', '--version'],
                              capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        raise RuntimeError(
            "arm-none-eabi-gcc not found. Please install ARM GCC toolchain:\n"
            "  macOS: brew install arm-none-eabi-gcc\n"
            "  Linux: sudo apt-get install gcc-arm-none-eabi"
        )

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir_path = Path(tmpdir)

        # Write source files
        c_file = tmpdir_path / "shellcode.c"
        ld_file = tmpdir_path / "shellcode.ld"
        elf_file = tmpdir_path / "shellcode.elf"
        bin_file = tmpdir_path / "shellcode.bin"

        c_file.write_text(c_code)
        ld_file.write_text(linker_script)

        # Compile
        compile_cmd = [
            'arm-none-eabi-gcc',
            '-mcpu=cortex-m0plus',
            '-mthumb',
            '-O2',                    # Optimize for size/speed
            '-ffunction-sections',
            '-fdata-sections',
            '-nostdlib',              # No standard library
            '-nostartfiles',          # No startup code
            '-T', str(ld_file),       # Linker script
            str(c_file),
            '-o', str(elf_file)
        ]

        print("Compiling shellcode...")
        print(f"  Command: {' '.join(compile_cmd)}")

        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print("Compilation failed!")
            print("STDOUT:", result.stdout)
            print("STDERR:", result.stderr)
            raise RuntimeError(f"Compilation failed: {result.stderr}")

        print("  ✓ Compilation successful")

        # Extract binary
        objcopy_cmd = [
            'arm-none-eabi-objcopy',
            '-O', 'binary',
            str(elf_file),
            str(bin_file)
        ]

        print("Extracting binary...")
        result = subprocess.run(objcopy_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"objcopy failed: {result.stderr}")

        print("  ✓ Binary extracted")

        # Read binary
        shellcode = bin_file.read_bytes()
        print(f"  Shellcode size: {len(shellcode)} bytes")

        # Optionally save output files
        if output_path:
            output_path = Path(output_path)
            output_path.mkdir(parents=True, exist_ok=True)

            import shutil
            shutil.copy(elf_file, output_path / "shellcode.elf")
            shutil.copy(bin_file, output_path / "shellcode.bin")
            (output_path / "shellcode.c").write_text(c_code)
            (output_path / "shellcode.ld").write_text(linker_script)

            print(f"  Saved to: {output_path}")

        return shellcode


# =============================================================================
# Main Generator Function
# =============================================================================

def generate_gpio_shellcode(blink_pin: str, final_pin: str,
                           num_blinks: int = 3,
                           ram_address: int = 0x20000A00,
                           sysclk_hz: int = DEFAULT_SYSCLK_HZ,
                           save_output: Optional[str] = None) -> bytes:
    """
    Generate GPIO blink shellcode

    Args:
        blink_pin: GPIO pin to blink (e.g., "PA5", "PB1")
        final_pin: GPIO pin to set low after blinking (e.g., "PA0")
        num_blinks: Number of blink cycles (default: 3)
        ram_address: Target RAM address (default: 0x20000A00)
        sysclk_hz: System clock in Hz (default: 24MHz HSI)
        save_output: Optional directory to save build artifacts

    Returns:
        Shellcode as bytes

    Example:
        >>> shellcode = generate_gpio_shellcode("PA5", "PB1", num_blinks=5)
        >>> len(shellcode)
        256
    """

    print("\n" + "="*70)
    print("GPIO Blink Shellcode Generator")
    print("="*70)
    print()

    # Parse GPIO pins
    print(f"Blink pin: {blink_pin}")
    blink_port, blink_num = parse_gpio_pin(blink_pin)
    print(f"  → Port {blink_port}, Pin {blink_num}")
    print()

    print(f"Final pin: {final_pin}")
    final_port, final_num = parse_gpio_pin(final_pin)
    print(f"  → Port {final_port}, Pin {final_num}")
    print()

    print(f"Configuration:")
    print(f"  Blinks: {num_blinks}")
    print(f"  RAM address: 0x{ram_address:08X}")
    print(f"  System clock: {sysclk_hz/1e6:.1f} MHz")
    print()

    # Generate C code
    print("Generating C code...")
    c_code = generate_c_code(blink_port, blink_num,
                            final_port, final_num,
                            num_blinks, sysclk_hz)
    print("  ✓ C code generated")
    print()

    # Generate linker script
    print("Generating linker script...")
    linker_script = generate_linker_script(ram_address)
    print("  ✓ Linker script generated")
    print()

    # Compile
    shellcode = compile_shellcode(c_code, linker_script, save_output)

    print()
    print("="*70)
    print("✓ Shellcode Ready!")
    print("="*70)
    print()
    print(f"Size: {len(shellcode)} bytes")
    print(f"Target: 0x{ram_address:08X}")
    print()
    print("Behavior:")
    print(f"  1. Blink {blink_pin} {num_blinks} times (1Hz, 50% duty cycle)")
    print(f"  2. Set {final_pin} LOW")
    print(f"  3. Wait 10 seconds")
    print(f"  4. Reset MCU")
    print()

    return shellcode


# =============================================================================
# Injection Functions
# =============================================================================

def inject_and_run_blink(mcu, shellcode: bytes, ram_address: int = 0x20000A00):
    """
    Inject GPIO blink shellcode into RAM and execute

    Args:
        mcu: PY32F003 instance (from pyrsp_py32f003)
        shellcode: Binary shellcode
        ram_address: Target RAM address

    Example:
        >>> from pyrsp_py32f003 import PY32F003
        >>> shellcode = generate_gpio_shellcode("PA5", "PB1", 3)
        >>> with PY32F003() as mcu:
        ...     inject_and_run_blink(mcu, shellcode)
    """

    print("\n" + "="*70)
    print("Injecting and Running Shellcode")
    print("="*70)
    print()

    # Halt CPU
    print("[1] Halting CPU...")
    mcu.halt()
    print("    ✓ CPU halted")
    print()

    # Write shellcode
    print(f"[2] Writing {len(shellcode)} bytes to 0x{ram_address:08X}...")
    mcu.write(ram_address, shellcode)

    # Verify
    verify = mcu.read(ram_address, len(shellcode))
    if verify == shellcode:
        print(f"    ✓ Shellcode written and verified")
    else:
        print(f"    ✗ Verification failed!")
        raise RuntimeError("Shellcode write verification failed")
    print()

    # Set PC (must have bit 0 set for Thumb mode)
    print("[3] Setting PC to shellcode entry...")
    pc_thumb = ram_address | 0x01

    # Read all registers
    all_regs = mcu.rsp.fetch(b'g')
    reg_bytes = bytearray(bytes.fromhex(all_regs.decode('ascii')))

    # Modify PC (register 15, offset 60)
    pc_bytes = pc_thumb.to_bytes(4, byteorder='little')
    reg_bytes[60:64] = pc_bytes

    # Write back
    reg_hex = reg_bytes.hex().encode('ascii')
    mcu.rsp.fetchOK(b'G' + reg_hex)
    print(f"    ✓ PC set to 0x{pc_thumb:08X}")
    print()

    # Resume
    print("[4] Resuming CPU...")
    mcu.resume()
    print("    ✓ Shellcode is now executing!")
    print()

    print("="*70)
    print("✓ Execution Started")
    print("="*70)
    print()
    print("The MCU is now:")
    print("  • Running your custom GPIO shellcode")
    print("  • Blinking the specified pin")
    print("  • Will reset automatically when complete")
    print()
    print("You can halt and inspect state at any time.")
    print()


# =============================================================================
# Command Line Interface
# =============================================================================

def main():
    """Command line interface for shellcode generation"""
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate GPIO blink shellcode for PY32F030",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate code to blink PA5 3 times, then set PB1 low
  python gpio_blink_generator.py PA5 PB1 --blinks 3

  # Save build artifacts to directory
  python gpio_blink_generator.py PA5 PB1 --blinks 5 --save-output ./build

  # Specify custom RAM address
  python gpio_blink_generator.py PA5 PB1 --ram-addr 0x20001000
        """
    )

    parser.add_argument('blink_pin', help='GPIO pin to blink (e.g., PA5, PB12)')
    parser.add_argument('final_pin', help='GPIO pin to set low (e.g., PB1)')
    parser.add_argument('--blinks', type=int, default=3,
                       help='Number of blink cycles (default: 3)')
    parser.add_argument('--ram-addr', type=lambda x: int(x, 0), default=0x20000A00,
                       help='Target RAM address (default: 0x20000A00)')
    parser.add_argument('--sysclk-mhz', type=float, default=24.0,
                       help='System clock in MHz (default: 24.0)')
    parser.add_argument('--save-output', type=str,
                       help='Directory to save build artifacts')
    parser.add_argument('--output-hex', action='store_true',
                       help='Print shellcode as hex')

    args = parser.parse_args()

    sysclk_hz = int(args.sysclk_mhz * 1e6)

    # Generate
    shellcode = generate_gpio_shellcode(
        blink_pin=args.blink_pin,
        final_pin=args.final_pin,
        num_blinks=args.blinks,
        ram_address=args.ram_addr,
        sysclk_hz=sysclk_hz,
        save_output=args.save_output
    )

    # Print hex if requested
    if args.output_hex:
        print("Shellcode (hex):")
        print(shellcode.hex())
        print()


if __name__ == '__main__':
    main()
