/*
 * stm32f070xB.h
 *
 *  Created on: Feb 16, 2021
 *      Author: Cosan
 */

#ifndef DRIVERS_INC_STM32F070XB_H_
#define DRIVERS_INC_STM32F070XB_H_

#include <stdint.h>
#define __IO volatile

/*
 * base adresses of Flash and SRAM memories
 */

#define FLASH_BASE									0x08000000UL
#define SRAM_BASE									0x20000000UL
#define ROM											0x1FFFC800UL
#define SRAM										SRAM_BASE


/*
 * AHBx and APB Bus Peripheral base adresses
 */
#define PERIPH_BASE									(0x40000000UL)
#define APBPERIPH_BASE								(PERIPH_BASE)
#define AHB1PERIPH_BASE								(PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE								(PERIPH_BASE + 0x08000000UL)

/*
 * Base adresses of peripherals which are hanging on AHB2 bus
 */

#define GPIOA_BASE									(AHB2PERIPH_BASE)
#define GPIOB_BASE									(AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE									(AHB2PERIPH_BASE + 0x00000800UL)
#define GPIOD_BASE									(AHB2PERIPH_BASE + 0x00000C00UL)
#define GPIOF_BASE									(AHB2PERIPH_BASE + 0x00001400UL)

/*!< AHB peripherals */
#define DMA1_BASE             (AHBPERIPH_BASE + 0x00000000UL)
#define DMA1_Channel1_BASE    (DMA1_BASE + 0x00000008UL)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x0000001CUL)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x00000030UL)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x00000044UL)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x00000058UL)

#define RCC_BASE              (AHB1PERIPH_BASE + 0x00001000UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x00002000UL) /*!< FLASH registers base address */
#define OB_BASE               0x1FFFF800UL       /*!< FLASH Option Bytes base address */
#define FLASHSIZE_BASE        0x1FFFF7CCUL       /*!< FLASH Size register base address */
#define UID_BASE              0x1FFFF7ACUL       /*!< Unique device ID register base address */
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000UL)

/*
 * Base adresses of peripherals which are hanging on APB bus
 */

#define TIM3_BASE             (APBPERIPH_BASE + 0x00000400UL)
#define TIM6_BASE             (APBPERIPH_BASE + 0x00001000UL)
#define TIM7_BASE             (APBPERIPH_BASE + 0x00001400UL)
#define TIM14_BASE            (APBPERIPH_BASE + 0x00002000UL)
#define RTC_BASE              (APBPERIPH_BASE + 0x00002800UL)
#define WWDG_BASE             (APBPERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE             (APBPERIPH_BASE + 0x00003000UL)
#define SPI2_BASE             (APBPERIPH_BASE + 0x00003800UL)
#define USART2_BASE           (APBPERIPH_BASE + 0x00004400UL)
#define USART3_BASE           (APBPERIPH_BASE + 0x00004800UL)
#define USART4_BASE           (APBPERIPH_BASE + 0x00004C00UL)
#define I2C1_BASE             (APBPERIPH_BASE + 0x00005400UL)
#define I2C2_BASE             (APBPERIPH_BASE + 0x00005800UL)
#define USB_BASE              (APBPERIPH_BASE + 0x00005C00UL) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR           (APBPERIPH_BASE + 0x00006000UL) /*!< USB_IP Packet Memory Area base address */
#define PWR_BASE              (APBPERIPH_BASE + 0x00007000UL)
#define SYSCFG_BASE           (APBPERIPH_BASE + 0x00010000UL)
#define EXTI_BASE             (APBPERIPH_BASE + 0x00010400UL)
#define ADC1_BASE             (APBPERIPH_BASE + 0x00012400UL)
#define ADC_BASE              (APBPERIPH_BASE + 0x00012708UL)
#define TIM1_BASE             (APBPERIPH_BASE + 0x00012C00UL)
#define SPI1_BASE             (APBPERIPH_BASE + 0x00013000UL)
#define USART1_BASE           (APBPERIPH_BASE + 0x00013800UL)
#define TIM15_BASE            (APBPERIPH_BASE + 0x00014000UL)
#define TIM16_BASE            (APBPERIPH_BASE + 0x00014400UL)
#define TIM17_BASE            (APBPERIPH_BASE + 0x00014800UL)
#define DBGMCU_BASE           (APBPERIPH_BASE + 0x00015800UL)

/*
 * Structuring Peripheral registers
 */

typedef struct
{
  __IO uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
  __IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function low register,  Address offset: 0x20-0x24 */
  __IO uint32_t BRR;          /*!< GPIO bit reset register,                     Address offset: 0x28      */
} GPIO_RegDef_t;



typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                   Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __IO uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  __IO uint32_t BDCR;       /*!< RCC Backup domain control register,                          Address offset: 0x20 */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  __IO uint32_t CFGR2;      /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
  __IO uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  __IO uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */
} RCC_RegDef_t;

/*
 * Peripherals definition macros
 */

#define GPIOA 						((GPIO_RegDef_t*) GPIOA_BASE)
#define GPIOB 						((GPIO_RegDef_t*) GPIOB_BASE)
#define GPIOC 						((GPIO_RegDef_t*) GPIOC_BASE)
#define GPIOD 						((GPIO_RegDef_t*) GPIOD_BASE)
#define GPIOF						((GPIO_RegDef_t*) GPIOF_BASE)
#define RCC							((RCC_RegDef_t*) RCC_BASE)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()		(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()		(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()		(RCC->AHBENR |= (1 << 20))
#define GPIOF_PCLK_EN()		(RCC->AHBENR |= (1 << 22))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()	(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()	(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 0))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()		(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()		(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()		(RCC->AHBENR &= ~(1 << 20))
#define GPIOF_PCLK_DI()		(RCC->AHBENR &= ~(1 << 22))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0))


/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()			do{(RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22));}while(0)

/*
 * Some generic macros
 */

#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET   		ENABLE
#define GPIO_PIN_RESET  	DISABLE


#include "stm32f070xB_gpio_driver.h"

#endif /* DRIVERS_INC_STM32F070XB_H_ */
