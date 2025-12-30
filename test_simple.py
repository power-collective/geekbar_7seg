#!/usr/bin/env python3
"""Simple single test - PA4 blink 3 times"""

from gpio_blink_generator import generate_gpio_shellcode, inject_and_run_blink
from pyrsp_py32f003 import PY32F003

# Generate shellcode to blink PA4 3 times, then set PA5 low
shellcode = generate_gpio_shellcode(
    blink_pin="PA4",
    final_pin="PA5",
    num_blinks=3,
    ram_address=0x20000A00,
    save_output="./build/test_simple"
)

print(f"\nGenerated {len(shellcode)} bytes")
print("\nConnecting to MCU...")

with PY32F003(verbose=True) as mcu:
    mcu.prepare_for_interactive_debug(reset_first=True)
    inject_and_run_blink(mcu, shellcode)
    mcu.resume()
    print("\n✓ Shellcode injected and running!")
    print("Watch PA4 blink 3 times, then PA5 should go low")
    print("MCU will reset after ~16 seconds")
