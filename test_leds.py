#!/usr/bin/env python3
"""
Test script for GPIO shellcode generator
Tests PA4 and PA5 LEDs
"""

import sys
from pathlib import Path

# Import the generator and MCU interface
from gpio_blink_generator import generate_gpio_shellcode, inject_and_run_blink
from pyrsp_py32f003 import PY32F003

def test_pa4_blink():
    """Test PA4 LED blinking"""
    print("\n" + "="*70)
    print("TEST 1: Blink PA4 (3 times)")
    print("="*70)

    # Generate shellcode to blink PA4, then set PA5 low
    shellcode = generate_gpio_shellcode(
        blink_pin="PA4",
        final_pin="PA5",
        num_blinks=10,
        ram_address=0x20000A00,
        save_output="./build/test_pa4"
    )

    print(f"\nGenerated {len(shellcode)} bytes of shellcode")
    print("\nConnecting to MCU...")

    with PY32F003(verbose=True) as mcu:
        # Prepare device
        mcu.prepare_for_interactive_debug(reset_first=True)

        # Inject and run
        inject_and_run_blink(mcu, shellcode)

        print("\nShellcode is running!")
        print("Expected behavior:")
        print("  - PA4 should blink 3 times (1 Hz, 50% duty cycle)")
        print("  - PA5 should go LOW after PA4 finishes")
        print("  - MCU will reset after ~16 seconds")
        print("\nWatch your LEDs!")

def test_pa5_blink():
    """Test PA5 LED blinking"""
    print("\n" + "="*70)
    print("TEST 2: Blink PA5 (5 times)")
    print("="*70)

    # Generate shellcode to blink PA5, then set PA4 low
    shellcode = generate_gpio_shellcode(
        blink_pin="PA5",
        final_pin="PA4",
        num_blinks=10,
        ram_address=0x20000A00,
        save_output="./build/test_pa5"
    )

    print(f"\nGenerated {len(shellcode)} bytes of shellcode")
    print("\nConnecting to MCU...")

    with PY32F003(verbose=True) as mcu:
        # Prepare device
        mcu.prepare_for_interactive_debug(reset_first=True)

        # Inject and run
        inject_and_run_blink(mcu, shellcode)

        print("\nShellcode is running!")
        print("Expected behavior:")
        print("  - PA5 should blink 5 times (1 Hz, 50% duty cycle)")
        print("  - PA4 should go LOW after PA5 finishes")
        print("  - MCU will reset after ~20 seconds")
        print("\nWatch your LEDs!")

def test_both_sequential():
    """Test both LEDs in quick succession"""
    print("\n" + "="*70)
    print("TEST 3: Quick blink test - PA4 then PA5")
    print("="*70)

    # Quick 2 blinks on PA4
    shellcode = generate_gpio_shellcode(
        blink_pin="PA4",
        final_pin="PA5",
        num_blinks=2,
        ram_address=0x20000A00,
        save_output="./build/test_quick"
    )

    print(f"\nGenerated {len(shellcode)} bytes of shellcode")
    print("\nConnecting to MCU...")

    with PY32F003(verbose=True) as mcu:
        # Prepare device
        mcu.prepare_for_interactive_debug(reset_first=True)

        # Inject and run
        inject_and_run_blink(mcu, shellcode)

        print("\nShellcode is running!")
        print("Expected behavior:")
        print("  - PA4 should blink 2 times quickly")
        print("  - PA5 should go LOW")
        print("  - MCU will reset after ~14 seconds")

if __name__ == '__main__':
    import time

    try:
        print("\n" + "="*70)
        print("GPIO Shellcode Generator - LED Test Suite")
        print("="*70)
        print("\nHardware: PA4 and PA5 connected to LEDs")
        print("Debugger: Connected via GDB remote protocol")
        print()

        # Run test 1
        test_pa4_blink()

        print("\n\nWaiting for shellcode to complete (~16 seconds)...")
        time.sleep(17)

        print("\n\nReady for next test...")
        time.sleep(2)

        # Run test 2
        test_pa5_blink()

        print("\n\n" + "="*70)
        print("Tests Complete!")
        print("="*70)
        print("\nIf both LEDs blinked correctly, the GPIO shellcode")
        print("generator is working properly with the fixed addresses!")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nError during test: {e}")
        import traceback
        traceback.print_exc()
