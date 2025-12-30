#!/usr/bin/env python3
"""Quick inject test - assumes MCU is already prepared"""

from gpio_blink_generator import inject_and_run_blink
from pyrsp_py32f003 import PY32F003

# Use pre-generated shellcode
with open('./build/manual_test/shellcode.bin', 'rb') as f:
    shellcode = f.read()

print(f"Loaded {len(shellcode)} bytes of shellcode")
print("Connecting and injecting...")

try:
    with PY32F003(verbose=True) as mcu:
        inject_and_run_blink(mcu, shellcode)
        print("\n✓ Done! Watch the LEDs")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
