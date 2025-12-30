/*
 * PY32F030 Hardware Definitions for GPIO Shellcode
 * Auto-generated header file
 */

#ifndef SHELLCODE_TEMPLATE_H
#define SHELLCODE_TEMPLATE_H

#include <stdint.h>

// =============================================================================
// Hardware Register Addresses
// =============================================================================

// RCC (Reset and Clock Control)
#define RCC_BASE        0x40021000U
#define RCC_CR          (RCC_BASE + 0x00)  // Clock control register
#define RCC_ICSCR       (RCC_BASE + 0x04)  // Internal clock sources calibration
#define RCC_CFGR        (RCC_BASE + 0x08)  // Clock configuration register
#define RCC_IOPENR      (RCC_BASE + 0x34)  // GPIO clock enable

// RCC_CR bits
#define RCC_CR_HSION    (1 << 8)   // HSI clock enable
#define RCC_CR_HSIRDY   (1 << 10)  // HSI clock ready flag

// FLASH
#define FLASH_BASE      0x40022000U
#define FLASH_ACR       (FLASH_BASE + 0x00)  // Access control register

// FLASH_ACR bits (wait states)
#define FLASH_ACR_LATENCY_0  0x00  // 0 wait states (up to 24 MHz)
#define FLASH_ACR_LATENCY_1  0x01  // 1 wait state (24-48 MHz)

// GPIO Bases (IOPORT region at 0x50000000)
#define GPIOA_BASE      0x50000000U
#define GPIOB_BASE      0x50000400U
#define GPIOC_BASE      0x50000800U
#define GPIOD_BASE      0x50000C00U
#define GPIOF_BASE      0x50001400U

// GPIO Register Offsets
#define GPIO_MODER_OFFSET   0x00
#define GPIO_ODR_OFFSET     0x14
#define GPIO_BSRR_OFFSET    0x18

// System Control Block
#define SCB_AIRCR       0xE000ED0CU

// AIRCR Reset Values
#define AIRCR_VECTKEY       0x05FA0000U
#define AIRCR_SYSRESETREQ   0x00000004U

// Independent Watchdog (IWDG)
#define IWDG_BASE       0x40003000U
#define IWDG_KR         (IWDG_BASE + 0x00)  // Key register
#define IWDG_PR         (IWDG_BASE + 0x04)  // Prescaler register
#define IWDG_RLR        (IWDG_BASE + 0x08)  // Reload register
#define IWDG_SR         (IWDG_BASE + 0x0C)  // Status register

// IWDG Key Values
#define IWDG_KEY_RELOAD     0xAAAAU  // Reload counter (kick watchdog)
#define IWDG_KEY_ENABLE     0xCCCCU  // Enable watchdog
#define IWDG_KEY_WRITE      0x5555U  // Enable PR/RLR write access

// IWDG Prescaler Values
#define IWDG_PRESCALER_256  0x06U    // Divide by 256 (max)

// SRAM
#define SRAM_BASE       0x20000000U
#define SRAM_SIZE       0x00002000U  // 8KB
#define SRAM_TOP        (SRAM_BASE + SRAM_SIZE)

// =============================================================================
// Register Access Macro
// =============================================================================

#define REG32(addr) (*(volatile uint32_t*)(addr))

#endif /* SHELLCODE_TEMPLATE_H */
