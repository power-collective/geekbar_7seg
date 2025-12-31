#!/usr/bin/env python3
"""
Trigger MCU reset via SWD - using PY32F003 class

This is the new version using pyrsp_py32f003.PY32F003 class.

Compare to utils/reset_mcu.py (old version):
- Old: 73 lines with boilerplate
- New: 18 lines (this file)

Reduction: 75% fewer lines!
"""

import time
from pyrsp_py32f003 import PY32F003


def main():
    with PY32F003(verbose = True) as mcu:
        # Reset and halt immediately
        mcu.reset(halt_immediately=True)

        # Wait briefly
        time.sleep(0.05)

        print("="*60)
        print("MCU reset and halted!")
        print("="*60)
        mcu.resume()


if __name__ == '__main__':
    main()
