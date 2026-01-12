/*
 * 7-Segment Display Shellcode Template - Header
 *
 * Defines for writing to BD0027 display buffers
 * Based on confirmed buffer analysis from monitor_display_buffers.py
 */

#ifndef DISPLAY_SHELLCODE_TEMPLATE_H
#define DISPLAY_SHELLCODE_TEMPLATE_H

#include <stdint.h>

// =============================================================================
// Display Buffer Addresses (CONFIRMED)
// =============================================================================

// Top display: Percentage 0-100
#define TOP_DISPLAY_ADDR        0x200000F8

// Bottom display: Raw digit values (0-9)
#define BOTTOM_RAW_ONES_ADDR    0x200001D8
#define BOTTOM_RAW_TENS_ADDR    0x200001D9

// Bottom display: 7-segment encoded
#define BOTTOM_7SEG_TENS_ADDR   0x200001E1
#define BOTTOM_7SEG_ONES_ADDR   0x200001E2

// =============================================================================
// 7-Segment Encoding (Common Anode, Active High)
// =============================================================================

// Segment bit positions: DP G F E D C B A
#define SEG_A   (1 << 0)
#define SEG_B   (1 << 1)
#define SEG_C   (1 << 2)
#define SEG_D   (1 << 3)
#define SEG_E   (1 << 4)
#define SEG_F   (1 << 5)
#define SEG_G   (1 << 6)
#define SEG_DP  (1 << 7)

// 7-segment digit patterns
static const uint8_t SEVEN_SEG_DIGITS[10] = {
    0x3F,  // 0: ABCDEF
    0x06,  // 1: BC
    0x5B,  // 2: ABDEG
    0x4F,  // 3: ABCDG
    0x66,  // 4: BCFG
    0x6D,  // 5: ACDFG
    0x7D,  // 6: ACDEFG
    0x07,  // 7: ABC
    0x7F,  // 8: ABCDEFG
    0x6F   // 9: ABCDFG
};

// =============================================================================
// System Control (for reset)
// =============================================================================

#define SCB_AIRCR           0xE000ED0C
#define AIRCR_VECTKEY       0x05FA0000
#define AIRCR_SYSRESETREQ   0x00000004

// =============================================================================
// Timing (8 MHz HSI - Default PY32F030 Reset Clock)
// =============================================================================
// IMPORTANT: PY32F030 defaults to 8 MHz HSI on reset, NOT 24 MHz
// If firmware has configured a different clock, these delays will be incorrect

#define SYSCLK_HZ           8000000
#define ITERATIONS_PER_MS   (SYSCLK_HZ / 1000 / 4)
#define SYSTICK_FREQ        1000  // SysTick frequency in Hz (assuming 1ms tick)

// =============================================================================
// SysTick Timer (for time measurement)
// =============================================================================

#define SYSTICK_BASE        0xE000E010
#define SYSTICK_CSR         (SYSTICK_BASE + 0x00)
#define SYSTICK_RVR         (SYSTICK_BASE + 0x04)
#define SYSTICK_CVR         (SYSTICK_BASE + 0x08)

// =============================================================================
// Independent Watchdog (IWDG)
// =============================================================================

#define IWDG_BASE           0x40003000
#define IWDG_KR             (IWDG_BASE + 0x00)
#define IWDG_KEY_RELOAD     0xAAAA

// =============================================================================
// Helper Macros
// =============================================================================

#define REG8(addr)   (*(volatile uint8_t*)(addr))
#define REG32(addr)  (*(volatile uint32_t*)(addr))

// =============================================================================
// Inline Functions
// =============================================================================

static inline void delay_ms(uint32_t ms) __attribute__((always_inline));
static inline void delay_ms(uint32_t ms) {
    volatile uint32_t count = ms * ITERATIONS_PER_MS;
    while (count--) {
        __asm__ volatile ("nop");
    }
}

static inline void write_top_display(uint8_t percentage) __attribute__((always_inline));
static inline void write_top_display(uint8_t percentage) {
    if (percentage > 100) percentage = 100;
    REG8(TOP_DISPLAY_ADDR) = percentage;
}

static inline void write_bottom_display(uint8_t value) __attribute__((always_inline));
static inline void write_bottom_display(uint8_t value) {
    if (value > 99) value = 99;

    uint8_t tens = (value / 10) % 10;
    uint8_t ones = value % 10;

    // Write raw digits
    REG8(BOTTOM_RAW_ONES_ADDR) = ones;
    REG8(BOTTOM_RAW_TENS_ADDR) = tens;

    // Write 7-segment encoded
    REG8(BOTTOM_7SEG_TENS_ADDR) = SEVEN_SEG_DIGITS[tens];
    REG8(BOTTOM_7SEG_ONES_ADDR) = SEVEN_SEG_DIGITS[ones];
}

static inline void system_reset(void) __attribute__((always_inline, noreturn));
static inline void system_reset(void) {
    REG32(SCB_AIRCR) = AIRCR_VECTKEY | AIRCR_SYSRESETREQ;
    while (1) __asm__ volatile ("nop");
}

static inline uint32_t get_systick(void) __attribute__((always_inline));
static inline uint32_t get_systick(void) {
    // Read current SysTick counter value
    // Note: SysTick counts DOWN, so we may need to invert or track rollovers
    // For simplicity, assume firmware has configured SysTick at 1ms
    return REG32(SYSTICK_CVR);
}

static inline void kick_watchdog(void) __attribute__((always_inline));
static inline void kick_watchdog(void) {
    // Reload IWDG counter to prevent reset
    REG32(IWDG_KR) = IWDG_KEY_RELOAD;
}

static inline void delay_us(uint32_t us) __attribute__((always_inline));
static inline void delay_us(uint32_t us) {
    volatile uint32_t count = (us * SYSCLK_HZ) / 1000000 / 4;
    while (count--) {
        __asm__ volatile ("nop");
    }
}

#endif // DISPLAY_SHELLCODE_TEMPLATE_H
