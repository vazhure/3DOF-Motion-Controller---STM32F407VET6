// FastGPIO.h - Fast GPIO macros for STM32F407 (Roger Clark core)
// Direct register access for maximum speed in ISR
//
// Version: 1.0
// Author: v.azhure@gmail.com
//
// Usage:
//   PIN_SET(GPIOE_REGS, 8);   // Set PE8 high
//   PIN_CLR(GPIOE_REGS, 8);   // Set PE8 low
//   PIN_TOGGLE(GPIOE_REGS, 8); // Toggle PE8
//
// Speed comparison:
//   digitalWrite():    ~300-600 ns
//   Fast GPIO macros:  ~12-30 ns (10-20x faster!)
// ============================================================================

#pragma once

#include <Arduino.h>

// ============================================================================
// STM32F4 GPIO Register Structure
// ============================================================================
typedef struct {
  volatile uint32_t MODER;    // 0x00 - GPIO port mode register
  volatile uint32_t OTYPER;   // 0x04 - GPIO port output type register
  volatile uint32_t OSPEEDR;  // 0x08 - GPIO port output speed register
  volatile uint32_t PUPDR;    // 0x0C - GPIO port pull-up/pull-down register
  volatile uint32_t IDR;      // 0x10 - GPIO port input data register
  volatile uint32_t ODR;      // 0x14 - GPIO port output data register
  volatile uint32_t BSRR;     // 0x18 - GPIO port bit set/reset register
  volatile uint32_t LCKR;     // 0x1C - GPIO port configuration lock register
  volatile uint32_t AFRL;     // 0x20 - GPIO alternate function low register
  volatile uint32_t AFRH;     // 0x24 - GPIO alternate function high register
} GPIO_Regs;

// ============================================================================
// GPIO Base Addresses (STM32F4)
// ============================================================================
#define GPIOA_BASE_ADDR 0x40020000
#define GPIOB_BASE_ADDR 0x40020400
#define GPIOC_BASE_ADDR 0x40020800
#define GPIOD_BASE_ADDR 0x40020C00
#define GPIOE_BASE_ADDR 0x40021000
#define GPIOF_BASE_ADDR 0x40021400
#define GPIOG_BASE_ADDR 0x40021800
#define GPIOH_BASE_ADDR 0x40021C00

// ============================================================================
// GPIO Register Pointers
// ============================================================================
#define GPIOA_REGS ((GPIO_Regs*)GPIOA_BASE_ADDR)
#define GPIOB_REGS ((GPIO_Regs*)GPIOB_BASE_ADDR)
#define GPIOC_REGS ((GPIO_Regs*)GPIOC_BASE_ADDR)
#define GPIOD_REGS ((GPIO_Regs*)GPIOD_BASE_ADDR)
#define GPIOE_REGS ((GPIO_Regs*)GPIOE_BASE_ADDR)
#define GPIOF_REGS ((GPIO_Regs*)GPIOF_BASE_ADDR)
#define GPIOG_REGS ((GPIO_Regs*)GPIOG_BASE_ADDR)
#define GPIOH_REGS ((GPIO_Regs*)GPIOH_BASE_ADDR)

// ============================================================================
// FAST GPIO MACROS
// ============================================================================
// BSRR register: Bits 0-15 = Set, Bits 16-31 = Reset
// Writing to BSRR is atomic - no read-modify-write needed

#define PIN_SET(regs, pin) ((regs)->BSRR = (1 << (pin)))
#define PIN_CLR(regs, pin) ((regs)->BSRR = (1 << ((pin) + 16)))
#define PIN_TOGGLE(regs, pin) ((regs)->ODR ^= (1 << (pin)))
#define PIN_READ(regs, pin) (((regs)->IDR >> (pin)) & 1)

// ============================================================================
// PRE-COMMON GPIO PORTS FOR LED AND STATUS PINS
// Pre-defined for faster access without getGpioPort() calls
// ============================================================================
// LED_PIN = PA6, SERIAL_LED_PIN = PA7, ALARM_PIN = PA0
#define LED_PORT GPIOA_REGS
#define LED_PIN_BIT 6
#define SERIAL_LED_PORT GPIOA_REGS
#define SERIAL_LED_BIT 7
#define ALARM_PORT GPIOA_REGS
#define ALARM_PIN_BIT 0

// ============================================================================
// HELPER FUNCTIONS - Convert Arduino pin to GPIO port and bit
// ============================================================================
// Roger Clark core: Pin encoding is (port * 16 + pin)
// PA0 = 0*16+0 = 0, PB0 = 1*16+0 = 16, PE8 = 4*16+8 = 72

inline GPIO_Regs* getGpioPort(uint32_t arduinoPin) {
  uint8_t portNum = (arduinoPin >> 4) & 0x0F;
  switch (portNum) {
    case 0: return GPIOA_REGS;
    case 1: return GPIOB_REGS;
    case 2: return GPIOC_REGS;
    case 3: return GPIOD_REGS;
    case 4: return GPIOE_REGS;
    case 5: return GPIOF_REGS;
    case 6: return GPIOG_REGS;
    case 7: return GPIOH_REGS;
    default: return GPIOA_REGS;  // Fallback
  }
}

inline uint8_t getGpioPinBit(uint32_t arduinoPin) {
  return arduinoPin & 0x0F;
}

// ============================================================================
// INLINE FAST WRITE FUNCTIONS (alternative to macros)
// ============================================================================
inline void fastDigitalWrite(GPIO_Regs* port, uint8_t pin, uint8_t value) {
  if (value) {
    port->BSRR = (1 << pin);  // Set bit
  } else {
    port->BSRR = (1 << (pin + 16));  // Reset bit
  }
}

inline uint8_t fastDigitalRead(GPIO_Regs* port, uint8_t pin) {
  return (port->IDR >> pin) & 1;
}