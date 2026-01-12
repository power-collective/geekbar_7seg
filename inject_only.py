#!/usr/bin/env python3
"""Quick inject test - assumes MCU is already prepared"""

import argparse
from pathlib import Path
from gpio_blink_generator import inject_and_run_blink
from pyrsp_py32f003 import PY32F003

def main():
    parser = argparse.ArgumentParser(
        description="Inject and run pre-compiled shellcode on PY32F003",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default shellcode.bin in current directory
  python3 test_inject_only.py

  # Use specific .bin file
  python3 test_inject_only.py custom_shellcode.bin

  # Specify device and verbosity
  python3 test_inject_only.py --device 127.0.0.1:2000 -v
        """
    )

    parser.add_argument(
        'bin_file',
        nargs='?',
        default='./shellcode.bin',
        help='Path to .bin file (default: ./shellcode.bin)'
    )

    parser.add_argument(
        '-d', '--device',
        default='127.0.0.1:1337',
        help='GDB device (default: 127.0.0.1:1337)'
    )

    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        default=True,
        help='Enable verbose output (default: True)'
    )

    args = parser.parse_args()

    # Check if file exists
    bin_path = Path(args.bin_file)
    if not bin_path.exists():
        print(f"Error: File not found: {bin_path}")
        return 1

    # Load shellcode
    with open(bin_path, 'rb') as f:
        shellcode = f.read()

    print(f"Loaded {len(shellcode)} bytes from {bin_path}")
    print("Connecting and injecting...")

    try:
        with PY32F003(verbose=args.verbose, device=args.device) as mcu:
            inject_and_run_blink(mcu, shellcode)
            print("\n✓ Done! Watch the LEDs")
            return 0
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit(main())
