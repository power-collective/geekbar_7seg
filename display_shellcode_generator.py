#!/usr/bin/env python3
"""
7-Segment Display Shellcode Generator

Generates ARM Thumb shellcode that directly controls GPIO pins to display
a 4-digit number on the 7-segment display. Does NOT rely on existing firmware.

Display Layout:
    [0] [1]    Position 0: Top-left
    [2] [3]    Position 1: Top-right
               Position 2: Bottom-left
               Position 3: Bottom-right

Pin Mapping:
    - PB0: Top-left cathode (active LOW)
    - PB1: Top-right cathode (active LOW)
    - PB2: Bottom-left cathode (active LOW)
    - PA8: Bottom-right cathode (active LOW)
    - PA11: Leading "1" anode (active HIGH)
    - Segments: PB8,7,6,5,3 (A,B,C,D,E) + PA15,12 (F,G)

Usage:
    from display_shellcode_generator import generate_display_shellcode, inject_and_run

    # Show "1234" (1 and 2 on top, 3 and 4 on bottom)
    shellcode = generate_display_shellcode(digits=[1, 2, 3, 4])

    # Show "88" on bottom row (blank top row)
    shellcode = generate_display_shellcode(digits=[0xFF, 0xFF, 8, 8])

    # Show leading "1" in top-left plus "23" on bottom
    shellcode = generate_display_shellcode(
        digits=[0xFF, 0xFF, 2, 3],
        show_leading_one=True,
        leading_one_position=0
    )

    # Inject and run
    inject_and_run(mcu, shellcode)
"""

import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Optional, Dict

# =============================================================================
# C Code Generator (GPIO-based)
# =============================================================================

def generate_c_code(digits: list = None,
                   show_leading_one: bool = False,
                   leading_one_position: int = 0,
                   duration_ms: int = 3000,
                   do_reset: bool = False) -> str:
    """
    Generate C code for GPIO-based display shellcode from template

    Args:
        digits: List of 4 digits [top-left, top-right, bottom-left, bottom-right]
                Each digit 0-9, or 0xFF for blank (default [0xFF, 0xFF, 0, 0])
        show_leading_one: Show the leading "1" digit (default False)
        leading_one_position: Position for leading "1" (0-3) (default 0)
        duration_ms: How long to display (default 3000ms = 3s)
        do_reset: Whether to reset MCU after displaying (default False)

    Returns:
        Generated C code as string
    """
    # Default: blank top row, "00" on bottom
    if digits is None:
        digits = [0xFF, 0xFF, 0, 0]

    # Validate inputs
    if len(digits) != 4:
        raise ValueError(f"digits must be list of 4 values, got {len(digits)}")

    for i, d in enumerate(digits):
        if d != 0xFF and not (0 <= d <= 9):
            raise ValueError(f"digit[{i}] must be 0-9 or 0xFF (blank), got {d}")

    if not 0 <= leading_one_position <= 3:
        raise ValueError(f"leading_one_position must be 0-3, got {leading_one_position}")

    if duration_ms < 0:
        raise ValueError(f"duration_ms must be >= 0, got {duration_ms}")

    # Load template
    template_path = Path(__file__).parent / "display_shellcode_gpio_template.c"

    with open(template_path, 'r') as f:
        template = f.read()

    # Replace placeholders
    replacements = {
        '{{DIGIT_0}}': str(digits[0]),
        '{{DIGIT_1}}': str(digits[1]),
        '{{DIGIT_2}}': str(digits[2]),
        '{{DIGIT_3}}': str(digits[3]),
        '{{SHOW_LEADING_ONE}}': '1' if show_leading_one else '0',
        '{{LEADING_ONE_POS}}': str(leading_one_position),
        '{{DURATION_MS}}': str(duration_ms),
        '{{DO_RESET}}': '1' if do_reset else '0',
    }

    code = template
    for placeholder, value in replacements.items():
        code = code.replace(placeholder, value)

    return code


def compile_shellcode(c_code: str, start_addr: int = 0x20000100, optimization: str = 'Os') -> bytes:
    """
    Compile C code to ARM Thumb shellcode

    Args:
        c_code: C source code
        start_addr: Target RAM address where shellcode will be loaded (default 0x20000100)
        optimization: GCC optimization level (Os, O2, O3)

    Returns:
        Compiled binary shellcode as bytes

    Raises:
        subprocess.CalledProcessError: If compilation fails
        FileNotFoundError: If arm-none-eabi-gcc not found
    """
    template_dir = Path(__file__).parent

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        # Write C code
        c_file = tmpdir / "shellcode.c"
        with open(c_file, 'w') as f:
            f.write(c_code)

        # Copy header
        header_src = template_dir / "display_shellcode_template.h"
        header_dst = tmpdir / "display_shellcode_template.h"
        with open(header_src, 'r') as f:
            header_content = f.read()
        with open(header_dst, 'w') as f:
            f.write(header_content)

        # Compile
        obj_file = tmpdir / "shellcode.o"
        elf_file = tmpdir / "shellcode.elf"
        bin_file = tmpdir / "shellcode.bin"

        # Compiler flags
        cflags = [
            'arm-none-eabi-gcc',
            '-march=armv6-m',
            '-mthumb',
            '-mcpu=cortex-m0plus',
            f'-{optimization}',
            '-Wall',
            '-ffunction-sections',
            '-fdata-sections',
            '-nostdlib',
            '-nostartfiles',
            '-c',
            str(c_file),
            '-o', str(obj_file)
        ]

        # Linker flags
        ldflags = [
            'arm-none-eabi-gcc',
            '-march=armv6-m',
            '-mthumb',
            '-mcpu=cortex-m0plus',
            '-nostdlib',
            '-nostartfiles',
            '-Wl,--gc-sections',
            '-Wl,--entry=shellcode_entry',
            f'-Wl,-Ttext={start_addr:#x}',  # Use actual injection address
            str(obj_file),
            '-o', str(elf_file)
        ]

        # objcopy flags
        objcopy_flags = [
            'arm-none-eabi-objcopy',
            '-O', 'binary',
            str(elf_file),
            str(bin_file)
        ]

        # Execute compilation
        try:
            subprocess.run(cflags, check=True, capture_output=True, text=True)
            subprocess.run(ldflags, check=True, capture_output=True, text=True)
            subprocess.run(objcopy_flags, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            print(f"Compilation failed!")
            print(f"Command: {' '.join(e.cmd)}")
            print(f"Return code: {e.returncode}")
            if e.stdout:
                print(f"Stdout:\n{e.stdout}")
            if e.stderr:
                print(f"Stderr:\n{e.stderr}")
            raise

        # Read binary
        with open(bin_file, 'rb') as f:
            shellcode = f.read()

        return shellcode


def generate_display_shellcode(digits: list = None,
                               show_leading_one: bool = False,
                               leading_one_position: int = 0,
                               duration_ms: int = 3000,
                               do_reset: bool = False,
                               target_addr: int = 0x20000100,
                               optimization: str = 'Os') -> bytes:
    """
    Generate GPIO-based display control shellcode

    Args:
        digits: List of 4 digits [top-left, top-right, bottom-left, bottom-right]
                Each digit 0-9, or 0xFF for blank (default [0xFF, 0xFF, 0, 0])
        show_leading_one: Show the leading "1" digit (default False)
        leading_one_position: Position for leading "1" (0-3) (default 0)
        duration_ms: How long to display (default 3000ms)
        do_reset: Reset MCU after displaying (default False)
        target_addr: RAM address where shellcode will be injected (default 0x20000100)
        optimization: GCC optimization level (default 'Os')

    Returns:
        Compiled shellcode as bytes

    Example:
        >>> # Show "1234" (all 4 digits)
        >>> shellcode = generate_display_shellcode(digits=[1, 2, 3, 4])
        >>> print(f"Generated {len(shellcode)} bytes")

        >>> # Show "88" on bottom row
        >>> shellcode = generate_display_shellcode(digits=[0xFF, 0xFF, 8, 8])

        >>> # Show leading "1" in top-left + "23" on bottom
        >>> shellcode = generate_display_shellcode(
        ...     digits=[0xFF, 0xFF, 2, 3],
        ...     show_leading_one=True,
        ...     leading_one_position=0,
        ...     duration_ms=5000
        ... )
    """
    c_code = generate_c_code(digits, show_leading_one, leading_one_position, duration_ms, do_reset)
    shellcode = compile_shellcode(c_code, start_addr=target_addr, optimization=optimization)
    return shellcode


# =============================================================================
# Injection and Execution
# =============================================================================

def inject_and_run(mcu, shellcode: bytes, ram_addr: int = 0x20000100) -> Dict:
    """
    Inject shellcode into RAM and execute it

    Args:
        mcu: PY32F003 instance (from pyrsp_py32f003)
        shellcode: Compiled shellcode bytes
        ram_addr: RAM address to inject at (default 0x20000100)

    Returns:
        Dict with execution results:
            'injected': bool
            'executed': bool
            'pc_before': int
            'pc_after': int

    Example:
        >>> from pyrsp_py32f003 import PY32F003
        >>> mcu = PY32F003()
        >>> shellcode = generate_display_shellcode(85, 12)
        >>> result = inject_and_run(mcu, shellcode)
        >>> print(f"Execution complete: {result}")
    """
    print(f"[*] Injecting {len(shellcode)} bytes to 0x{ram_addr:08x}")

    # Halt target
    mcu.halt()

    # Save original PC
    pc_before = mcu.read_pc()
    print(f"    Original PC: 0x{pc_before:08x}")

    # Write shellcode to RAM
    for i, byte in enumerate(shellcode):
        mcu.write_u8(ram_addr + i, byte)

    print(f"    ✓ Shellcode injected")

    # Set PC to shellcode entry point (Thumb mode, so +1)
    entry_point = ram_addr | 1
    mcu.write_register('pc', entry_point)
    print(f"    Set PC to 0x{entry_point:08x}")

    # Resume execution
    print(f"[*] Executing shellcode...")
    mcu.resume()

    # Small delay for execution
    import time
    time.sleep(0.1)

    # Read PC after execution
    mcu.halt()
    pc_after = mcu.read_pc()
    print(f"    PC after: 0x{pc_after:08x}")
    mcu.resume()

    return {
        'injected': True,
        'executed': True,
        'pc_before': pc_before,
        'pc_after': pc_after,
        'shellcode_size': len(shellcode),
        'entry_point': entry_point
    }


# =============================================================================
# Convenience Functions
# =============================================================================

def show_number(mcu, number: int, duration_ms: int = 3000, position: str = 'bottom', ram_addr: int = 0x20000100):
    """
    Convenience function to show a 2-digit number

    Args:
        mcu: PY32F003 instance
        number: Number to display (0-99)
        duration_ms: How long to show (default 3000ms)
        position: Where to show ('top' or 'bottom', default 'bottom')
        ram_addr: RAM address for injection (default 0x20000100)

    Example:
        >>> from pyrsp_py32f003 import PY32F003
        >>> mcu = PY32F003()
        >>> show_number(mcu, 42, duration_ms=5000, position='bottom')
    """
    tens = number // 10
    ones = number % 10

    if position == 'top':
        digits = [tens, ones, 0xFF, 0xFF]
    else:  # bottom
        digits = [0xFF, 0xFF, tens, ones]

    shellcode = generate_display_shellcode(digits, duration_ms=duration_ms, target_addr=ram_addr)
    inject_and_run(mcu, shellcode, ram_addr=ram_addr)
    print(f"\n✓ Displaying {number:02d} on {position} row for {duration_ms}ms")


def show_all_digits(mcu, digits: list, duration_ms: int = 3000,
                   show_leading_one: bool = False, leading_one_pos: int = 0, ram_addr: int = 0x20000100):
    """
    Convenience function to show all 4 digits

    Args:
        mcu: PY32F003 instance
        digits: List of 4 digits [top-left, top-right, bottom-left, bottom-right]
        duration_ms: How long to show (default 3000ms)
        show_leading_one: Show leading "1" (default False)
        leading_one_pos: Position for leading "1" (0-3)
        ram_addr: RAM address for injection (default 0x20000100)

    Example:
        >>> from pyrsp_py32f003 import PY32F003
        >>> mcu = PY32F003()
        >>> show_all_digits(mcu, [1, 2, 3, 4], duration_ms=5000)
        Shows: 12
               34
    """
    shellcode = generate_display_shellcode(
        digits=digits,
        show_leading_one=show_leading_one,
        leading_one_position=leading_one_pos,
        duration_ms=duration_ms,
        target_addr=ram_addr
    )
    inject_and_run(mcu, shellcode, ram_addr=ram_addr)
    digit_str = ''.join(str(d) if d != 0xFF else '_' for d in digits)
    print(f"\n✓ Displaying [{digit_str[0]}{digit_str[1]}] [{digit_str[2]}{digit_str[3]}] for {duration_ms}ms")


# =============================================================================
# Main (for testing)
# =============================================================================

if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage: python3 display_shellcode_generator.py <mode> [args...]")
        print()
        print("Modes:")
        print("  number <num> [position]   - Show 2-digit number (position: top/bottom)")
        print("  digits <d0> <d1> <d2> <d3> - Show all 4 digits (0-9 or 255 for blank)")
        print("  leading1 <pos> <d2> <d3>   - Show leading '1' + 2 bottom digits")
        print()
        print("Examples:")
        print("  python3 display_shellcode_generator.py number 42 bottom")
        print("  python3 display_shellcode_generator.py digits 1 2 3 4")
        print("  python3 display_shellcode_generator.py leading1 0 2 3")
        sys.exit(1)

    mode = sys.argv[1]

    if mode == 'number':
        if len(sys.argv) < 3:
            print("Error: 'number' mode requires <num> [position]")
            sys.exit(1)

        number = int(sys.argv[2])
        position = sys.argv[3] if len(sys.argv) > 3 else 'bottom'
        tens = number // 10
        ones = number % 10

        if position == 'top':
            digits = [tens, ones, 0xFF, 0xFF]
        else:
            digits = [0xFF, 0xFF, tens, ones]

        print(f"Generating shellcode to show {number:02d} on {position} row")
        shellcode = generate_display_shellcode(digits=digits, duration_ms=5000)

    elif mode == 'digits':
        if len(sys.argv) != 6:
            print("Error: 'digits' mode requires 4 digit values")
            sys.exit(1)

        digits = [int(sys.argv[i]) for i in range(2, 6)]
        print(f"Generating shellcode to show: [{digits[0]}{digits[1]}] [{digits[2]}{digits[3]}]")
        shellcode = generate_display_shellcode(digits=digits, duration_ms=5000)

    elif mode == 'leading1':
        if len(sys.argv) != 5:
            print("Error: 'leading1' mode requires <position> <digit2> <digit3>")
            sys.exit(1)

        leading_pos = int(sys.argv[2])
        d2 = int(sys.argv[3])
        d3 = int(sys.argv[4])
        digits = [0xFF, 0xFF, d2, d3]

        print(f"Generating shellcode to show leading '1' at position {leading_pos} + [{d2}{d3}]")
        shellcode = generate_display_shellcode(
            digits=digits,
            show_leading_one=True,
            leading_one_position=leading_pos,
            duration_ms=5000
        )

    else:
        print(f"Error: Unknown mode '{mode}'")
        sys.exit(1)

    print(f"✓ Generated {len(shellcode)} bytes of shellcode")

    # Save to file
    output_file = Path("display_shellcode.bin")
    with open(output_file, 'wb') as f:
        f.write(shellcode)
    print(f"✓ Saved to {output_file}")
