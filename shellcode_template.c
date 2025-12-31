/*
 * GPIO Blink Shellcode Template - FULL VERSION (NOT WORKING)
 *
 * NOTE: This version has issues with clock initialization causing infinite loops.
 * USE shellcode_minimal.c INSTEAD - it is confirmed working on hardware.
 *
 * This file is kept for reference and future development.
 */

#include "shellcode_template.h"

// =============================================================================
// Configuration (will be replaced by generator)
// =============================================================================

// GPIO Configuration
#define BLINK_GPIO_BASE     {{BLINK_GPIO_BASE}}
#define FINAL_GPIO_BASE     {{FINAL_GPIO_BASE}}
#define BLINK_PIN           {{BLINK_PIN}}
#define FINAL_PIN           {{FINAL_PIN}}
#define BLINK_CLOCK_BIT     {{BLINK_CLOCK_BIT}}
#define FINAL_CLOCK_BIT     {{FINAL_CLOCK_BIT}}

// Behavior
#define NUM_BLINKS          {{NUM_BLINKS}}

// Timing
#define SYSCLK_HZ           {{SYSCLK_HZ}}
#define ITERATIONS_PER_MS   (SYSCLK_HZ / 1000 / 4)

// =============================================================================
// System Initialization
// =============================================================================

static inline void system_init(void) __attribute__((always_inline));
static inline void system_init(void) {
    // Enable HSI (24 MHz internal oscillator)
    REG32(RCC_CR) |= RCC_CR_HSION;

    // Wait for HSI ready
    while (!(REG32(RCC_CR) & RCC_CR_HSIRDY));

    // Set Flash latency to 0 wait states (safe for 24 MHz)
    REG32(FLASH_ACR) = FLASH_ACR_LATENCY_0;
}

// =============================================================================
// Watchdog Functions
// =============================================================================

static inline void iwdg_init_max_timeout(void) __attribute__((always_inline));
static inline void iwdg_init_max_timeout(void) {
    // Enable write access to IWDG registers
    REG32(IWDG_KR) = IWDG_KEY_WRITE;

    // Wait for write access
    while (REG32(IWDG_SR) & 0x01);  // Wait for PVU bit clear

    // Set maximum prescaler (divide by 256)
    REG32(IWDG_PR) = IWDG_PRESCALER_256;

    // Wait for prescaler update
    while (REG32(IWDG_SR) & 0x01);

    // Set maximum reload value (0xFFF = 4095)
    REG32(IWDG_RLR) = 0xFFF;

    // Wait for reload update
    while (REG32(IWDG_SR) & 0x02);  // Wait for RVU bit clear

    // Reload counter
    REG32(IWDG_KR) = IWDG_KEY_RELOAD;

    // Enable watchdog if not already enabled
    REG32(IWDG_KR) = IWDG_KEY_ENABLE;
}

static inline void iwdg_kick(void) __attribute__((always_inline));
static inline void iwdg_kick(void) {
    REG32(IWDG_KR) = IWDG_KEY_RELOAD;
}

// =============================================================================
// Delay Function
// =============================================================================

static inline void delay_ms(uint32_t ms) __attribute__((always_inline));
static inline void delay_ms(uint32_t ms) {
    volatile uint32_t count = ms * ITERATIONS_PER_MS;
    while (count--) {
        __asm__ volatile ("nop");

        // Kick watchdog every ~1000 iterations (~1ms at 24MHz)
        if ((count & 0x3FF) == 0) {
            iwdg_kick();
        }
    }
}

// =============================================================================
// Main Shellcode Entry Point
// =============================================================================

__attribute__((naked, noreturn, section(".text.entry")))
void shellcode_entry(void) {

    // Skip system init - assume firmware already configured clocks
    // system_init();


    // Small delay for clock to stabilize
    delay_ms(1);

    // Configure blink pin as output (MODER = 01 for output)
    volatile uint32_t *blink_moder = (volatile uint32_t*)(BLINK_GPIO_BASE + GPIO_MODER_OFFSET);
    uint32_t moder_val = *blink_moder;
    moder_val &= ~(3U << (BLINK_PIN * 2));  // Clear mode bits
    moder_val |= (1U << (BLINK_PIN * 2));   // Set as output
    *blink_moder = moder_val;

    // Configure final pin as output
    volatile uint32_t *final_moder = (volatile uint32_t*)(FINAL_GPIO_BASE + GPIO_MODER_OFFSET);
    moder_val = *final_moder;
    moder_val &= ~(3U << (FINAL_PIN * 2));  // Clear mode bits
    moder_val |= (1U << (FINAL_PIN * 2));   // Set as output
    *final_moder = moder_val;

    // BSRR register for atomic bit set/reset
    volatile uint32_t *blink_bsrr = (volatile uint32_t*)(BLINK_GPIO_BASE + GPIO_BSRR_OFFSET);
    volatile uint32_t *final_bsrr = (volatile uint32_t*)(FINAL_GPIO_BASE + GPIO_BSRR_OFFSET);

    // Blink loop
    for (int i = 0; i < NUM_BLINKS; i++) {
        // Set pin high (BS - bit set)
        *blink_bsrr = (1U << BLINK_PIN);
        delay_ms(500);  // 0.5 second on

        // Set pin low (BR - bit reset, upper 16 bits)
        *blink_bsrr = (1U << (BLINK_PIN + 16));
        delay_ms(500);  // 0.5 second off

        // Kick watchdog between blinks
        iwdg_kick();
    }

    // Set final pin high
    *final_bsrr = (1U << FINAL_PIN);

    // Wait 10 seconds (with watchdog kicking)
    delay_ms(10000);

    // Software reset via AIRCR
    REG32(SCB_AIRCR) = AIRCR_VECTKEY | AIRCR_SYSRESETREQ;

    // Should never reach here, but infinite loop just in case
    while (1) {
        __asm__ volatile ("nop");
    }
}
