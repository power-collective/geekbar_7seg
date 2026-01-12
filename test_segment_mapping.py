#!/usr/bin/env python3
"""
Test 7-segment display segment mapping by directly controlling GPIO pins.
This helps verify the actual hardware connections vs our code assumptions.
"""

from pyrsp_py32f003 import PY32F003
import time

# GPIO register offsets
GPIOA_BASE = 0x50000000
GPIOB_BASE = 0x50000400
GPIO_MODER = 0x00
GPIO_ODR = 0x14

# Pin definitions from our code
PIN_LED_A = (1 << 8)   # PB8
PIN_LED_B = (1 << 7)   # PB7
PIN_LED_C = (1 << 6)   # PB6
PIN_LED_D = (1 << 5)   # PB5
PIN_LED_E = (1 << 3)   # PB3
PIN_LED_F = (1 << 15)  # PA15
PIN_LED_G = (1 << 12)  # PA12

# Digit cathodes (active LOW)
PIN_DIGIT_TOP_LEFT = (1 << 0)   # PB0
PIN_DIGIT_TOP_RIGHT = (1 << 1)  # PB1
PIN_DIGIT_BOTTOM_LEFT = (1 << 2)  # PB2
PIN_DIGIT_BOTTOM_RIGHT = (1 << 8)  # PA8

# Leading "1"
PIN_LEADING_ONE = (1 << 11)  # PA11

def init_gpio(mcu):
    """Initialize GPIO pins as outputs"""
    print("Initializing GPIO pins...")

    # Read current MODER registers
    gpioa_moder = mcu.read_u32(GPIOA_BASE + GPIO_MODER)
    gpiob_moder = mcu.read_u32(GPIOB_BASE + GPIO_MODER)

    # Configure GPIOA pins (PA8, PA11, PA12, PA15) as output (mode 01)
    for pin in [8, 11, 12, 15]:
        gpioa_moder &= ~(3 << (pin * 2))  # Clear mode bits
        gpioa_moder |= (1 << (pin * 2))   # Set to output

    # Configure GPIOB pins (PB0-3, PB5-8) as output
    for pin in [0, 1, 2, 3, 5, 6, 7, 8]:
        gpiob_moder &= ~(3 << (pin * 2))
        gpiob_moder |= (1 << (pin * 2))

    mcu.write_u32(GPIOA_BASE + GPIO_MODER, gpioa_moder)
    mcu.write_u32(GPIOB_BASE + GPIO_MODER, gpiob_moder)

    # Initialize: all cathodes HIGH (disabled), all segments LOW
    mcu.write_u32(GPIOA_BASE + GPIO_ODR, PIN_DIGIT_BOTTOM_RIGHT)
    mcu.write_u32(GPIOB_BASE + GPIO_ODR, PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT)

    print("✓ GPIO initialized")

def display_segment_pattern(mcu, segment_mask, position='top-right', duration=5.0):
    """
    Display a specific segment pattern at a digit position

    Args:
        mcu: PY32F003 instance
        segment_mask: 7-bit mask where each bit controls a segment (bit 0=A, 1=B, etc.)
        position: 'top-left', 'top-right', 'bottom-left', 'bottom-right'
        duration: How long to display (seconds)
    """
    # Map segment bits to GPIO pins
    gpioa_val = PIN_DIGIT_BOTTOM_RIGHT  # Start with cathodes disabled
    gpiob_val = PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT

    # Set segment anodes based on pattern
    if segment_mask & (1 << 0):  # Bit 0 -> LED_A
        gpiob_val |= PIN_LED_A
    if segment_mask & (1 << 1):  # Bit 1 -> LED_B
        gpiob_val |= PIN_LED_B
    if segment_mask & (1 << 2):  # Bit 2 -> LED_C
        gpiob_val |= PIN_LED_C
    if segment_mask & (1 << 3):  # Bit 3 -> LED_D
        gpiob_val |= PIN_LED_D
    if segment_mask & (1 << 4):  # Bit 4 -> LED_E
        gpiob_val |= PIN_LED_E
    if segment_mask & (1 << 5):  # Bit 5 -> LED_F
        gpioa_val |= PIN_LED_F
    if segment_mask & (1 << 6):  # Bit 6 -> LED_G
        gpioa_val |= PIN_LED_G

    # Enable selected digit cathode (pull LOW)
    position_map = {
        'top-left': PIN_DIGIT_TOP_LEFT,
        'top-right': PIN_DIGIT_TOP_RIGHT,
        'bottom-left': PIN_DIGIT_BOTTOM_LEFT,
        'bottom-right': PIN_DIGIT_BOTTOM_RIGHT
    }

    cathode = position_map[position]
    if cathode & 0xFF00:  # GPIOA
        gpioa_val &= ~cathode
    else:  # GPIOB
        gpiob_val &= ~cathode

    # Write to GPIO
    mcu.write_u32(GPIOA_BASE + GPIO_ODR, gpioa_val)
    mcu.write_u32(GPIOB_BASE + GPIO_ODR, gpiob_val)

    time.sleep(duration)

    # Turn off
    mcu.write_u32(GPIOA_BASE + GPIO_ODR, PIN_DIGIT_BOTTOM_RIGHT)
    mcu.write_u32(GPIOB_BASE + GPIO_ODR, PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT)

def test_individual_segments(mcu):
    """Test each segment bit individually"""
    segment_names = ['A (top)', 'B (top-right)', 'C (bottom-right)',
                     'D (bottom)', 'E (bottom-left)', 'F (top-left)', 'G (middle)']

    print("\n" + "="*60)
    print("Testing individual segments")
    print("="*60)

    results = {}

    for bit in range(7):
        pattern = 1 << bit
        print(f"\nBit {bit} - Expected: {segment_names[bit]}")
        print(f"Pattern: 0b{pattern:07b}")

        display_segment_pattern(mcu, pattern, 'top-right', duration=0.5)
        time.sleep(0.3)

        actual = input(f"  Which segment lit up? (or 'none'): ").strip()
        results[bit] = actual

    print("\n" + "="*60)
    print("RESULTS:")
    print("="*60)
    for bit, actual in results.items():
        print(f"Bit {bit} ({segment_names[bit]}): {actual}")

    return results

def test_digit_patterns(mcu):
    """Test standard digit patterns"""
    digits = {
        '0': 0b00111111,
        '1': 0b00000110,
        '2': 0b01011011,
        '3': 0b01001111,
        '8': 0b01111111,
    }

    print("\n" + "="*60)
    print("Testing digit patterns")
    print("="*60)

    for digit, pattern in digits.items():
        print(f"\nDisplaying '{digit}' (pattern: 0b{pattern:07b})")
        display_segment_pattern(mcu, pattern, 'top-right', duration=2.0)

        looks_correct = input(f"  Does it look like '{digit}'? (y/n): ").strip().lower()
        if looks_correct != 'y':
            actual = input(f"  What does it look like? ").strip()
            print(f"  -> Expected '{digit}', got '{actual}'")

        time.sleep(0.5)

def main():
    print("7-Segment Display GPIO Test")
    print("="*60)

    mcu = PY32F003()
    print("✓ Connected to MCU")

    # Don't halt - we want firmware to keep running for stability
    mcu.prepare_for_interactive_debug()

    init_gpio(mcu)

    while True:
        print("\n" + "="*60)
        print("MENU:")
        print("  1. Test individual segments (discover mapping)")
        print("  2. Test digit patterns (0,1,2,3,8)")
        print("  3. Custom pattern (manual entry)")
        print("  4. All segments ON (test wiring)")
        print("  5. Exit")
        print("="*60)

        choice = input("Choose: ").strip()

        if choice == '1':
            test_individual_segments(mcu)
        elif choice == '2':
            test_digit_patterns(mcu)
        elif choice == '3':
            pattern_str = input("Enter 7-bit pattern (e.g., 0b1010101 or 85): ").strip()
            if pattern_str.startswith('0b'):
                pattern = int(pattern_str, 2)
            else:
                pattern = int(pattern_str)
            print(f"Displaying pattern: 0b{pattern:07b}")
            display_segment_pattern(mcu, pattern, 'top-right', duration=3.0)
        elif choice == '4':
            print("All segments ON (0b1111111)")
            display_segment_pattern(mcu, 0b1111111, 'top-right', duration=3.0)
        elif choice == '5':
            print("Exiting...")
            break
        else:
            print("Invalid choice")

if __name__ == '__main__':
    main()
