/**
 * GPIO-based 7-segment display shellcode template
 *
 * This template directly controls GPIO pins to display a 4-digit number.
 * Does NOT rely on existing firmware - pure GPIO manipulation.
 *
 * Template variables (replaced by generator):
 *   {{DIGIT_0}} - Top-left digit (0-9 or 0xFF for blank)
 *   {{DIGIT_1}} - Top-right digit (0-9 or 0xFF for blank)
 *   {{DIGIT_2}} - Bottom-left digit (0-9 or 0xFF for blank)
 *   {{DIGIT_3}} - Bottom-right digit (0-9 or 0xFF for blank)
 *   {{SHOW_LEADING_ONE}} - 1 to show leading "1", 0 to hide
 *   {{LEADING_ONE_POS}} - Position of leading "1" (0-3)
 *   {{DURATION_MS}} - How long to display (milliseconds)
 *   {{DO_RESET}} - 1 to reset after display, 0 to return
 */

#include <stdint.h>

// IWDG (Independent Watchdog) for kick_watchdog
#define IWDG_BASE           0x40003000
#define IWDG_KR             (IWDG_BASE + 0x00)
#define IWDG_KEY_RELOAD     0xAAAA

// GPIO Register addresses
#define GPIOA_BASE      0x50000000
#define GPIOB_BASE      0x50000400
#define GPIO_MODER      0x00
#define GPIO_OTYPER     0x04
#define GPIO_OSPEEDR    0x08
#define GPIO_PUPDR      0x0C
#define GPIO_ODR        0x14
#define GPIO_BSRR       0x18

// RCC for GPIO clocks
#define RCC_IOPENR      0x40021034

// AIRCR for reset
#define AIRCR           0xE000ED0C
#define AIRCR_VECTKEY   0x05FA0000
#define AIRCR_SYSRESETREQ 0x04

// Convenience macros
#define REG32(addr) (*(volatile uint32_t*)(addr))
#define GPIOA_MODER     REG32(GPIOA_BASE + GPIO_MODER)
#define GPIOA_OTYPER    REG32(GPIOA_BASE + GPIO_OTYPER)
#define GPIOA_OSPEEDR   REG32(GPIOA_BASE + GPIO_OSPEEDR)
#define GPIOA_PUPDR     REG32(GPIOA_BASE + GPIO_PUPDR)
#define GPIOA_ODR       REG32(GPIOA_BASE + GPIO_ODR)
#define GPIOA_BSRR      REG32(GPIOA_BASE + GPIO_BSRR)
#define GPIOA_BRR       REG32(GPIOA_BASE + 0x28)

#define GPIOB_MODER     REG32(GPIOB_BASE + GPIO_MODER)
#define GPIOB_OTYPER    REG32(GPIOB_BASE + GPIO_OTYPER)
#define GPIOB_OSPEEDR   REG32(GPIOB_BASE + GPIO_OSPEEDR)
#define GPIOB_PUPDR     REG32(GPIOB_BASE + GPIO_PUPDR)
#define GPIOB_ODR       REG32(GPIOB_BASE + GPIO_ODR)
#define GPIOB_BSRR      REG32(GPIOB_BASE + GPIO_BSRR)
#define GPIOB_BRR       REG32(GPIOB_BASE + 0x28)

// Pin definitions (from display_7seg.c)
#define PIN_LED_A       (1 << 8)   // PB8
#define PIN_LED_B       (1 << 7)   // PB7
#define PIN_LED_C       (1 << 6)   // PB6
#define PIN_LED_D       (1 << 5)   // PB5
#define PIN_LED_E       (1 << 3)   // PB3
#define PIN_LED_F       (1 << 15)  // PA15
#define PIN_LED_G       (1 << 12)  // PA12

// Digit cathode pins (active LOW for multiplexing)
// NOTE: PB0, PB1, PB2 are "High Sink" pins (80mA capability per PY32F030 datasheet)
// WARNING: PA8 is NOT a High Sink pin - may result in dimmer 4th digit
// For optimal brightness, all cathodes should use High Sink pins (PB0-2 only)
#define PIN_DIGIT_TOP_LEFT      (1 << 0)   // PB0 (High Sink)
#define PIN_DIGIT_TOP_RIGHT     (1 << 1)   // PB1 (High Sink)
#define PIN_DIGIT_BOTTOM_LEFT   (1 << 2)   // PB2 (High Sink)
#define PIN_DIGIT_BOTTOM_RIGHT  (1 << 8)   // PA8 (Standard I/O - may be dimmer!)

#define PIN_LEADING_ONE_ANODE   (1 << 11)  // PA11

// Masks for display-related pins (for read-modify-write operations)
#define GPIOA_DISPLAY_MASK (PIN_LED_F | PIN_LED_G | PIN_DIGIT_BOTTOM_RIGHT | PIN_LEADING_ONE_ANODE)
#define GPIOB_DISPLAY_MASK (PIN_LED_A | PIN_LED_B | PIN_LED_C | PIN_LED_D | PIN_LED_E | \
                            PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT)

// 7-segment encoding (0-9)
static const uint8_t DIGIT_PATTERNS[10] = {
    0b00111111,  // 0
    0b00000110,  // 1
    0b01011011,  // 2
    0b01001111,  // 3
    0b01100110,  // 4
    0b01101101,  // 5
    0b01111101,  // 6
    0b00000111,  // 7
    0b01111111,  // 8
    0b01101111,  // 9
};

// Simple delay
static void delay_cycles(uint32_t count) {
    volatile uint32_t i;
    for (i = 0; i < count; i++) {
        __asm__ volatile ("nop");
    }
}

// Initialize GPIO for display
static void gpio_init(void) {
    // Enable GPIOA and GPIOB clocks
    REG32(RCC_IOPENR) |= (1 << 0) | (1 << 1);  // IOPAEN | IOPBEN

    // Configure GPIOA pins (PA3,4,5 zone enables + PA8,11,12,15 display) as output
    uint32_t moder_a = GPIOA_MODER;
    moder_a &= ~((3 << (3*2)) | (3 << (4*2)) | (3 << (5*2)) | (3 << (8*2)) | (3 << (11*2)) | (3 << (12*2)) | (3 << (15*2)));
    moder_a |= (1 << (3*2)) | (1 << (4*2)) | (1 << (5*2)) | (1 << (8*2)) | (1 << (11*2)) | (1 << (12*2)) | (1 << (15*2));
    GPIOA_MODER = moder_a;

    // Configure GPIOB pins (PB0-3, PB5-8) as output
    uint32_t moder_b = GPIOB_MODER;
    moder_b &= ~((3 << (0*2)) | (3 << (1*2)) | (3 << (2*2)) | (3 << (3*2)) |
                 (3 << (5*2)) | (3 << (6*2)) | (3 << (7*2)) | (3 << (8*2)));
    moder_b |= (1 << (0*2)) | (1 << (1*2)) | (1 << (2*2)) | (1 << (3*2)) |
               (1 << (5*2)) | (1 << (6*2)) | (1 << (7*2)) | (1 << (8*2));
    GPIOB_MODER = moder_b;

    // Set high speed
    GPIOA_OSPEEDR |= (3 << (3*2)) | (3 << (4*2)) | (3 << (5*2)) | (3 << (8*2)) | (3 << (11*2)) | (3 << (12*2)) | (3 << (15*2));
    GPIOB_OSPEEDR |= (3 << (0*2)) | (3 << (1*2)) | (3 << (2*2)) | (3 << (3*2)) |
                     (3 << (5*2)) | (3 << (6*2)) | (3 << (7*2)) | (3 << (8*2));

    // Initialize: all cathodes HIGH (disabled), segments LOW
    // Use read-modify-write to preserve other pins
    uint32_t odr_a = GPIOA_ODR;
    odr_a &= ~GPIOA_DISPLAY_MASK;  // Clear display pins
    odr_a |= PIN_DIGIT_BOTTOM_RIGHT;  // PA8 HIGH (cathode disabled)
    GPIOA_ODR = odr_a;

    uint32_t odr_b = GPIOB_ODR;
    odr_b &= ~GPIOB_DISPLAY_MASK;  // Clear display pins
    odr_b |= (PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT);  // PB0,1,2 HIGH (cathodes disabled)
    GPIOB_ODR = odr_b;
}

// Convert 7-segment pattern to GPIO values
static void pattern_to_gpio(uint8_t pattern, uint8_t show_leading_one, uint32_t *gpioa_out, uint32_t *gpiob_out) {
    // Start with all digit cathodes HIGH (disabled) and zone enables HIGH
    uint32_t gpioa = PIN_DIGIT_BOTTOM_RIGHT;
    uint32_t gpiob = PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT;

    // Map segment bits to pins
    if (pattern & (1 << 0)) gpiob |= PIN_LED_A;
    if (pattern & (1 << 1)) gpiob |= PIN_LED_B;
    if (pattern & (1 << 2)) gpiob |= PIN_LED_C;
    if (pattern & (1 << 3)) gpiob |= PIN_LED_D;
    if (pattern & (1 << 4)) gpiob |= PIN_LED_E;
    if (pattern & (1 << 5)) gpioa |= PIN_LED_F;
    if (pattern & (1 << 6)) gpioa |= PIN_LED_G;

    // Add leading "1" as an 8th segment if requested
    // PA11 shares the same cathode as the digit it's displayed with
    if (show_leading_one) {
        gpioa |= PIN_LEADING_ONE_ANODE;
    }

    *gpioa_out = gpioa;
    *gpiob_out = gpiob;
}

// Display one digit at position, optionally with leading "1"
static void display_digit(uint8_t digit, uint8_t position, uint8_t show_leading_one) {
    uint32_t gpioa_val, gpiob_val;

    // STEP 1: Blank all cathodes first to prevent ghosting
    // Use read-modify-write to preserve non-display pins
    uint32_t odr_a = GPIOA_ODR;
    odr_a &= ~GPIOA_DISPLAY_MASK;
    odr_a |= PIN_DIGIT_BOTTOM_RIGHT;  // All cathodes HIGH (disabled)
    GPIOA_ODR = odr_a;

    uint32_t odr_b = GPIOB_ODR;
    odr_b &= ~GPIOB_DISPLAY_MASK;
    odr_b |= (PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT);
    GPIOB_ODR = odr_b;

    // STEP 2: Get pattern and convert to GPIO values
    uint8_t pattern = (digit <= 9) ? DIGIT_PATTERNS[digit] : 0x00;
    pattern_to_gpio(pattern, show_leading_one, &gpioa_val, &gpiob_val);

    // STEP 3: Enable selected digit cathode (pull LOW) in calculated value
    switch (position) {
        case 0: gpiob_val &= ~PIN_DIGIT_TOP_LEFT; break;
        case 1: gpiob_val &= ~PIN_DIGIT_TOP_RIGHT; break;
        case 2: gpiob_val &= ~PIN_DIGIT_BOTTOM_LEFT; break;
        case 3: gpioa_val &= ~PIN_DIGIT_BOTTOM_RIGHT; break;
    }

    // STEP 4: Write final values using read-modify-write to preserve non-display pins
    odr_a = GPIOA_ODR;
    odr_a &= ~GPIOA_DISPLAY_MASK;  // Clear display pins
    odr_a |= (gpioa_val & GPIOA_DISPLAY_MASK);  // Set new display values
    GPIOA_ODR = odr_a;

    odr_b = GPIOB_ODR;
    odr_b &= ~GPIOB_DISPLAY_MASK;  // Clear display pins
    odr_b |= (gpiob_val & GPIOB_DISPLAY_MASK);  // Set new display values
    GPIOB_ODR = odr_b;
}

// Display leading "1" at specified position
// Position 0-1 (top row): requires PB0 AND PB1 both LOW
// Position 2-3 (bottom row): requires PB2 AND PA8 both LOW
static void display_leading_one(uint8_t position) {
    // STEP 1: Blank all cathodes first using read-modify-write
    uint32_t odr_a = GPIOA_ODR;
    odr_a &= ~GPIOA_DISPLAY_MASK;
    odr_a |= PIN_DIGIT_BOTTOM_RIGHT;
    GPIOA_ODR = odr_a;

    uint32_t odr_b = GPIOB_ODR;
    odr_b &= ~GPIOB_DISPLAY_MASK;
    odr_b |= (PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT);
    GPIOB_ODR = odr_b;

    // STEP 2: Calculate final GPIO values
    // PA11 HIGH for leading "1" anode, all segments LOW
    uint32_t gpioa_val = PIN_DIGIT_BOTTOM_RIGHT | PIN_LEADING_ONE_ANODE;
    uint32_t gpiob_val = PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT;

    // STEP 3: Enable cathodes based on position (top or bottom row)
    if (position <= 1) {
        // Top row: enable both PB0 and PB1
        gpiob_val &= ~(PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT);
    } else {
        // Bottom row: enable both PB2 and PA8
        gpiob_val &= ~PIN_DIGIT_BOTTOM_LEFT;
        gpioa_val &= ~PIN_DIGIT_BOTTOM_RIGHT;
    }

    // STEP 4: Write final values using read-modify-write
    odr_a = GPIOA_ODR;
    odr_a &= ~GPIOA_DISPLAY_MASK;
    odr_a |= (gpioa_val & GPIOA_DISPLAY_MASK);
    GPIOA_ODR = odr_a;

    odr_b = GPIOB_ODR;
    odr_b &= ~GPIOB_DISPLAY_MASK;
    odr_b |= (gpiob_val & GPIOB_DISPLAY_MASK);
    GPIOB_ODR = odr_b;
}

// Entry point
void __attribute__((section(".text.entry"))) shellcode_entry(void) {
    // CRITICAL: Initialize stack pointer to safe location FIRST
    // PY32F030 has 4KB RAM at 0x20000000-0x20001000
    // Set SP to top of RAM (0x20001000) to avoid HardFault
    // Use only Thumb16 instructions compatible with ARMv6-M (Cortex-M0+)
    register uint32_t sp_val __asm__("r0");
    sp_val = 0x20000000;
    __asm__ volatile ("mov sp, %0" :: "r"(sp_val) : "memory");

    // Configuration from template
    const uint8_t digits[4] = {{{DIGIT_0}}, {{DIGIT_1}}, {{DIGIT_2}}, {{DIGIT_3}}};
    const uint8_t show_leading_one = {{SHOW_LEADING_ONE}};
    const uint8_t leading_one_pos = {{LEADING_ONE_POS}};
    const uint32_t duration_ms = {{DURATION_MS}};
    const uint8_t do_reset = {{DO_RESET}};

    // Initialize GPIO
    gpio_init();

    // Disable interrupts during display to prevent ISR jitter
    // Will re-enable before exiting
    __asm__ volatile ("cpsid i" ::: "memory");

    // Calculate cycles for multiplexing
    // Target: 2ms per digit for smooth 500Hz refresh (4 digits = 8ms total cycle = 125Hz)
    // At 8MHz HSI: 2ms = 16,000 cycles
    // Loop overhead ~4 cycles/iteration: 16,000 / 4 = 4,000 iterations
    const uint32_t digit_delay = 4000;

    // Count how many digits are actually displayed (not blank)
    uint8_t active_digit_count = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (digits[i] != 0xFF) active_digit_count++;
    }
    // Leading "1" is shown during digit cycles, not separately

    // Calculate actual cycle time based on digits that will be shown
    const uint32_t cycle_time_ms = active_digit_count * 2;  // 2ms per active digit
    // Multiplex display - avoid division by using multiplication in loop condition
    // Loop while: (cycle * cycle_time_ms) < duration_ms
    for (uint32_t cycle = 0; cycle_time_ms > 0 && (cycle * cycle_time_ms) < duration_ms; cycle++) {
        // Kick watchdog to prevent reset (if IWDG is enabled by firmware)
        REG32(IWDG_KR) = IWDG_KEY_RELOAD;

        // Show all 4 digit positions
        for (uint8_t pos = 0; pos < 4; pos++) {
            if (digits[pos] != 0xFF) {  // 0xFF = blank
                // Determine if this position should show leading "1"
                // Leading "1" spans a row: positions 0-1 (top) or 2-3 (bottom)
                uint8_t show_led1_here = show_leading_one &&
                                         ((pos <= 1 && leading_one_pos <= 1) ||
                                          (pos >= 2 && leading_one_pos >= 2));

                display_digit(digits[pos], pos, show_led1_here);
                delay_cycles(digit_delay);
            }
        }
    }

    // Turn off display: all cathodes HIGH (disabled), segments LOW
    // Use read-modify-write to preserve non-display pins
    uint32_t odr_a = GPIOA_ODR;
    odr_a &= ~GPIOA_DISPLAY_MASK;
    odr_a |= PIN_DIGIT_BOTTOM_RIGHT;  // PA8 HIGH (cathode disabled)
    GPIOA_ODR = odr_a;

    uint32_t odr_b = GPIOB_ODR;
    odr_b &= ~GPIOB_DISPLAY_MASK;
    odr_b |= (PIN_DIGIT_TOP_LEFT | PIN_DIGIT_TOP_RIGHT | PIN_DIGIT_BOTTOM_LEFT);  // PB0,1,2 HIGH (cathodes disabled)
    GPIOB_ODR = odr_b;

    // Re-enable interrupts before finishing
    __asm__ volatile ("cpsie i" ::: "memory");

    // Reset or return
    if (do_reset) {
        REG32(AIRCR) = AIRCR_VECTKEY | AIRCR_SYSRESETREQ;
        while(1);  // Wait for reset
    }

    // Infinite loop (shellcode never returns unless reset)
    // Interrupts are now enabled, so WFI can wake on SysTick or other IRQs
    while(1) {
        __asm__ volatile ("wfi");  // Wait for interrupt
    }
}
