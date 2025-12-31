#!/usr/bin/env python3
"""
GPIO Blink Demo

Demonstrates using the GPIO blink shellcode generator to:
1. Generate custom ARM code
2. Inject it into RAM
3. Execute it on the MCU
4. Automatically reset when done

This example blinks PA5 3 times, then sets PB1 low for 10 seconds,
then resets the MCU.
"""

import sys
from pathlib import Path

# Add utils to path
sys.path.insert(0, str(Path(__file__).parent.parent / "utils"))

from pyrsp_py32f003 import PY32F003
from gpio_blink_generator import generate_gpio_shellcode, inject_and_run_blink


def demo_basic_blink():
    """
    Basic demo: Blink PA5 3 times
    """
    print("\n" + "="*70)
    print("Demo: Basic GPIO Blink")
    print("="*70)
    print()

    # Generate shellcode
    shellcode = generate_gpio_shellcode(
        blink_pin="PA5",     # Blink this pin
        final_pin="PB1",     # Set this pin low at end
        num_blinks=3,        # Blink 3 times
        ram_address=0x20000A00,
        save_output="./build/basic_blink"  # Save build artifacts
    )

    # Connect and run
    print("Connecting to MCU...")
    with PY32F003(verbose=True) as mcu:
        # Prepare device
        #mcu.prepare_for_interactive_debug(reset_first=True)
        mcu.reset(halt_immediately=True)
        # Inject and run
        inject_and_run_blink(mcu, shellcode)

        # Wait for completion (3 blinks * 2 seconds + 10 second delay = ~16 seconds)
        print("Shellcode is running...")
        print("  - Watch PA5 blink 3 times (1 Hz)")
        print("  - PB1 will go low")
        print("  - MCU will reset after ~16 seconds")
        print()

        input("Press Enter to check status...")

        # Check if still running
        mcu.halt()
        all_regs = mcu.rsp.fetch(b'g')
        reg_bytes = bytes.fromhex(all_regs.decode('ascii'))
        pc = int.from_bytes(reg_bytes[60:64], byteorder='little')

        print(f"Current PC: 0x{pc:08X}")

        if (pc & ~0x03) >= 0x20000A00 and (pc & ~0x03) < 0x20000B00:
            print("✓ Shellcode is still running")
        else:
            print("✓ Shellcode completed (MCU reset to firmware)")

        mcu.resume()


def demo_fast_blink():
    """
    Demo: Fast blink with more cycles
    """
    print("\n" + "="*70)
    print("Demo: Fast Blink (10 times)")
    print("="*70)
    print()

    # Generate shellcode for 10 blinks
    shellcode = generate_gpio_shellcode(
        blink_pin="PA0",
        final_pin="PA1",
        num_blinks=10,
        save_output="./build/fast_blink"
    )

    with PY32F003(verbose=True) as mcu:
        mcu.prepare_for_interactive_debug(reset_first=True)
        inject_and_run_blink(mcu, shellcode)

        print("Watch PA0 blink 10 times!")
        input("Press Enter when done...")


def demo_multiple_pins():
    """
    Demo: Generate code for different pins (manual execution)
    """
    print("\n" + "="*70)
    print("Demo: Generate Code for Multiple Pins")
    print("="*70)
    print()

    configs = [
        ("PA5", "PB1", 3),
        ("PB0", "PA1", 5),
        ("PA7", "PB2", 2),
    ]

    for blink, final, count in configs:
        print(f"Generating: Blink {blink} {count}x, final {final}")

        shellcode = generate_gpio_shellcode(
            blink_pin=blink,
            final_pin=final,
            num_blinks=count,
            save_output=f"./build/blink_{blink.lower()}_{count}x"
        )

        print(f"  Generated {len(shellcode)} bytes")
        print()

    print("All shellcode generated! Check ./build/ directory")


def demo_custom_timing():
    """
    Demo: Use different system clock frequency
    """
    print("\n" + "="*70)
    print("Demo: Custom Timing (8 MHz clock)")
    print("="*70)
    print()

    # If running at 8 MHz instead of 24 MHz HSI
    shellcode = generate_gpio_shellcode(
        blink_pin="PA5",
        final_pin="PB1",
        num_blinks=3,
        sysclk_hz=8_000_000,  # 8 MHz
        save_output="./build/custom_timing"
    )

    print("Note: Delays will be calibrated for 8 MHz system clock")
    print("If running at different speed, blink timing will be off!")


def interactive_demo():
    """
    Interactive demo - ask user for parameters
    """
    print("\n" + "="*70)
    print("Interactive GPIO Blink Generator")
    print("="*70)
    print()

    print("Enter GPIO configuration:")
    print()

    blink_pin = input("  Blink pin (e.g., PA5): ").strip().upper()
    final_pin = input("  Final pin (e.g., PB1): ").strip().upper()

    try:
        num_blinks = int(input("  Number of blinks (e.g., 3): ").strip())
    except ValueError:
        num_blinks = 3
        print(f"  Using default: {num_blinks}")

    print()
    print(f"Generating shellcode:")
    print(f"  - Blink {blink_pin} {num_blinks} times")
    print(f"  - Set {final_pin} low for 10s")
    print(f"  - Reset MCU")
    print()

    try:
        shellcode = generate_gpio_shellcode(
            blink_pin=blink_pin,
            final_pin=final_pin,
            num_blinks=num_blinks,
            save_output="./build/interactive"
        )

        print()
        run = input("Inject and run on connected MCU? [y/N]: ").strip().lower()

        if run == 'y':
            with PY32F003(verbose=True) as mcu:
                mcu.prepare_for_interactive_debug(reset_first=True)
                inject_and_run_blink(mcu, shellcode)

                print()
                print("Shellcode is running! Watch your GPIO pins.")
                print()

    except Exception as e:
        print(f"Error: {e}")


def main():
    """Main demo menu"""
    print("\n" + "="*70)
    print("GPIO Blink Shellcode Generator - Demos")
    print("="*70)
    print()
    print("Choose a demo:")
    print()
    print("  1. Basic blink (PA5, 3 times)")
    print("  2. Fast blink (PA0, 10 times)")
    print("  3. Generate multiple configurations")
    print("  4. Custom timing (8 MHz clock)")
    print("  5. Interactive (custom pins)")
    print()

    choice = input("Enter choice [1-5]: ").strip()

    demos = {
        '1': demo_basic_blink,
        '2': demo_fast_blink,
        '3': demo_multiple_pins,
        '4': demo_custom_timing,
        '5': interactive_demo,
    }

    demo = demos.get(choice)
    if demo:
        demo()
    else:
        print("Invalid choice")


if __name__ == '__main__':
    main()
