/*
 * stm32f767xx.h
 *
 *  Created on: Mar 14, 2022
 *      Author: Truong
 */

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#include<stddef.h>
#include <stdint.h>
#define _vo volatile
#define __weak	__attribute__((weak))	

/***********************************************************START : processor specific detail *********************************************************************/
/**
 * 	 Địa chỉ các thanh ghi (enable )thể hiện thứ tự các ngắt ( ko phải mức ưu tiên)
 * ARM cortex	Mx processor NVIC ISERx register Addresses
 *  */
#define NVIC_ISER0 ((_vo uint32_t *)0xE000E100)
#define NVIC_ISER1 ((_vo uint32_t *)0xE000E104)
#define NVIC_ISER2 ((_vo uint32_t *)0xE000E108)
#define NVIC_ISER3 ((_vo uint32_t *)0xE000E10C)
#define NVIC_ISER4 ((_vo uint32_t *)0xE000E110)
#define NVIC_ISER5 ((_vo uint32_t *)0xE000E114)
#define NVIC_ISER6 ((_vo uint32_t *)0xE000E118)
#define NVIC_ISER7 ((_vo uint32_t *)0xE000E11C)

/**
 * Địa chỉ các thanh ghi (disnable )thể hiện thứ tự các ngắt ( ko phải mức ưu tiên)
 * ARM cortex	Mx processor NVIC ICERx register Addresses
 *  */
#define NVIC_ICER0 ((_vo uint32_t *)0XE000E180)
#define NVIC_ICER1 ((_vo uint32_t *)0XE000E184)
#define NVIC_ICER2 ((_vo uint32_t *)0XE000E188)
#define NVIC_ICER3 ((_vo uint32_t *)0XE000E18C)
#define NVIC_ICER4 ((_vo uint32_t *)0XE000E190)
#define NVIC_ICER5 ((_vo uint32_t *)0XE000E194)
#define NVIC_ICER6 ((_vo uint32_t *)0XE000E198)
#define NVIC_ICER7 ((_vo uint32_t *)0XE000E19C)

/**
 * Địa chỉ các thanh ghi cấu hình mức ưu tiên ngắt
 * ARM cortex	Mx processor Priority register Addresses caculattion
 *  */
#define NVIC_PR_BASE_ADDR ((_vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4 // Chỉ có 4 bit cao của mỗi section trong thanh ghi mức ưu tiên là được sử dụng

/* Base address of FLASH and SRAM memories */

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20020000U
#define SRAM2_BASEADDR 0x2007C000U
/*
 * #define ROM_BASEADDR
 * #define OTP_BASEADDR
 * #define SRAM
 * */

/*
 * AHBx and ABPx Bus Peripheral base addresses
 */
#define PERIPH_BASE 0x40000000U
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U
#define AHB3PERIPH_BASE 0xA0000000U

/*
 * Dia chi co ban cua cac ngoai vi treo tren  bus AHB1
 */

#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR (AHB1PERIPH_BASE + 0x2800)
#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x3800)

/*
 * Dia chi co ban cua cac ngoai vi treo tren  bus APB1
 */

#define SPI2_BASEADDR (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASE + 0x5C00)
#define I2C4_BASEADDR (APB1PERIPH_BASE + 0x6000)

/*
 * Dia chi co ban cua cac ngoai vi treo tren  bus APB2
 */
#define USART1_BASEADDR (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0x1400)

#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR (APB2PERIPH_BASE + 0x3400)

#define EXTI_BASEADDR (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0x3800)

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	_vo uint32_t MODER;	  // GPIO port mode register
	_vo uint32_t OTYPER;  // GPIO port output type register
	_vo uint32_t OSPEEDR; // GPIO port output speed register
	_vo uint32_t PUPDR;	  // GPIO port pull-up/pull-down register
	_vo uint32_t IDR;	  // GPIO port input data register
	_vo uint32_t ODR;	  // GPIO port output data register
	_vo uint32_t BSRR;	  // GPIO port bit set/reset register (
	_vo uint32_t LCKR;	  // GPIO port configuration lock register
	_vo uint32_t AFR[2];  // GPIO alternate function low/high
} GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct
{
	_vo uint32_t CR;		 // 00
	_vo uint32_t PLLCFGR;	 // 04
	_vo uint32_t CFGR;		 // 08
	_vo uint32_t CIR;		 // 0c
	_vo uint32_t AHB1RSTR;	 // 10
	_vo uint32_t AHB2RSTR;	 // 14
	_vo uint32_t AHB3RSTR;	 // 18
	uint32_t Reserved7;		 // 1c
	_vo uint32_t APB1RSTR;	 // 20
	_vo uint32_t APB2RSTR;	 // 24
	uint32_t Reserved0[2];	 // 28- 2c
	_vo uint32_t AHB1ENR;	 // 30
	_vo uint32_t AHB2ENR;	 // 34
	_vo uint32_t AHB3ENR;	 // 38
	uint32_t Reserved2;		 // 3c
	_vo uint32_t APB1ENR;	 // 40
	_vo uint32_t APB2ENR;	 // 44
	uint32_t Reserved3[2];	 // 48 -4c
	_vo uint32_t AHB1LPENR;	 // 50
	_vo uint32_t AHB2LPENR;	 // 54
	_vo uint32_t AHB3LPENR;	 // 58
	uint32_t Reserved4;		 // 5c
	_vo uint32_t APB1LPENR;	 // 60
	_vo uint32_t APB2LPENR;	 // 64
	uint32_t Reserved5[2];	 // 68-6c
	_vo uint32_t BDCR;		 // 70
	_vo uint32_t CSR;		 // 74
	uint32_t Reserved6[2];	 // 78-7c
	_vo uint32_t SSCGR;		 // 80
	_vo uint32_t PLLI2SCFGR; // 84
	_vo uint32_t PLLSAICFGR; // 88
	_vo uint32_t DCKCFGR1;	 // 8c
	_vo uint32_t DCKCFGR2;	 // 90

} RCC_Regdef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	_vo uint32_t IMR;	// Interrupt mask register
	_vo uint32_t EMR;	// Event mask register
	_vo uint32_t RTSR;	// Rising trigger selection register
	_vo uint32_t FTSR;	// Falling trigger selection register
	_vo uint32_t SWIER; // Software interrupt event register
	_vo uint32_t PR;	// Pending register

} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct
{
	_vo uint32_t MEMRMP;	// SYSCFG memory remap register
	_vo uint32_t PMC;		// SYSCFG peripheral mode configuration register
	_vo uint32_t EXTICR[4]; // SYSCFG external interrupt configuration register 1-4
	uint32_t RESVERVED;
	_vo uint32_t CBR;	// Class B register
	_vo uint32_t CMPCR; // Compensation cell control register

} SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */

typedef struct
{
	_vo uint32_t CR1;	  // SPI control register 1
	_vo uint32_t CR2;	  // SPI control register 2
	_vo uint32_t SR;	  // SPI status register
	_vo uint32_t DR;	  // SPI data register
	_vo uint32_t CRCPR;	  // SPI CRC polynomial registe
	_vo uint32_t RXCRCR;  //	SPI Rx CRC register
	_vo uint32_t TXCRCR;  //	SPI Tx CRC register
	_vo uint32_t I2SCFGR; // SPIx configuration registe
	_vo uint32_t I2SPR;	  // SPIx prescaler register

} SPI_RegDef_t;

/*
 * Peripheral  definition  (xxxRegdef_t)
 */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define RCC ((RCC_Regdef_t *)RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)

/*
 * Clock enable Macros for GPIOx Peripharal
 */

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1U << 0U)) //#define GPIOA_PERI_CLOCK_ENABLE
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1U << 1U))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN() (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN() (RCC->AHB1ENR |= (1 << 10))

/*
 * Clock enable Macros for I2Cx Peripheral
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))

/*
 * Clock enable Macros for SPIx Peripheral
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

/*
 * Clock enable Macros for SYSCFG Peripheral
 */

#define SYSCFG_PLCK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx Peripheral
 */

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 10))

/*
 * Clock Disable Macros for SPIx Peripheral
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/*
 *  Macros TO RESET GPIOx Peripheral
 */

#define GPIOA_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 0));  \
		(RCC->AHB1RSTR &= ~(1 << 0)); \
	} while (0) // reset xong phải tắt chế độ reset đi
#define GPIOB_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 1));  \
		(RCC->AHB1RSTR &= ~(1 << 1)); \
	} while (0)
#define GPIOC_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 2));  \
		(RCC->AHB1RSTR &= ~(1 << 2)); \
	} while (0)
#define GPIOD_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 3));  \
		(RCC->AHB1RSTR &= ~(1 << 3)); \
	} while (0)
#define GPIOE_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 4));  \
		(RCC->AHB1RSTR &= ~(1 << 4)); \
	} while (0)
#define GPIOF_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 5));  \
		(RCC->AHB1RSTR &= ~(1 << 5)); \
	} while (0)
#define GPIOG_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 6));  \
		(RCC->AHB1RSTR &= ~(1 << 6)); \
	} while (0)
#define GPIOH_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 7));  \
		(RCC->AHB1RSTR &= ~(1 << 7)); \
	} while (0)
#define GPIOI_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 8));  \
		(RCC->AHB1RSTR &= ~(1 << 8)); \
	} while (0)
#define GPIOJ_REG_RESET()             \
	do                                \
	{                                 \
		(RCC->AHB1RSTR |= (1 << 9));  \
		(RCC->AHB1RSTR &= ~(1 << 9)); \
	} while (0)
#define GPIOK_REG_RESET()              \
	do                                 \
	{                                  \
		(RCC->AHB1RSTR |= (1 << 10));  \
		(RCC->AHB1RSTR &= ~(1 << 10)); \
	} while (0)

/*
 *  Return port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : (x == GPIOB) ? 1  \
												 : (x == GPIOC)	  ? 2  \
												 : (x == GPIOD)	  ? 3  \
												 : (x == GPIOE)	  ? 4  \
												 : (x == GPIOF)	  ? 5  \
												 : (x == GPIOG)	  ? 6  \
												 : (x == GPIOH)	  ? 7  \
												 : (x == GPIOI)	  ? 8  \
												 : (x == GPIOJ)	  ? 9  \
												 : (x == GPIOK)	  ? 10 \
																  : 0)

/**
 * IRQ Number  ( VỊ TRÍ CHỨ KO PHẢI MỨC ƯU TIÊN )
 *
 */

#define IRQ_NO_PVD 1
#define IRQ_NO_TAMP_STAMP 2
#define IRQ_NO_RTC_WKUP 3
#define IRQ_NO_FLASH 4
#define IRQ_NO_RCC 5
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_DMA1_Stream0 11
#define IRQ_NO_DMA1_Stream1 12
#define IRQ_NO_DMA1_Stream2 13
#define IRQ_NO_DMA1_Stream3 14
#define IRQ_NO_DMA1_Stream4 15
#define IRQ_NO_DMA1_Stream5 16
#define IRQ_NO_DMA1_Stream6 17
#define IRQ_NO_ADC 18
#define IRQ_NO_CAN1_TX 19
#define IRQ_NO_CAN1_RX0 20
#define IRQ_NO_CAN1_RX1 21
#define IRQ_NO_CAN1_SCE 22
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_TIM1_BRK_TIM9 24
#define IRQ_NO_TIM1_UP_TIM10 25
#define IRQ_NO_TIM1_TRG_COM_TIM11 26
#define IRQ_NO_TIM1_CC 27
#define IRQ_NO_TIM2 28
#define IRQ_NO_TIM3 29
#define IRQ_NO_TIM4 30
#define IRQ_NO_I2C1_EV 31
#define IRQ_NO_I2C1_ER 32
#define IRQ_NO_I2C2_EV 33
#define IRQ_NO_I2C2_ER 34
#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_USART1 37
#define IRQ_NO_USART2 38
#define IRQ_NO_USART3 39
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_RTC_Alarm 41
#define IRQ_NO_OTG_FS_WKUP 42
#define IRQ_NO_TIM8_BRK_TIM12 43
#define IRQ_NO_TIM8_UP_TIM13 44
#define IRQ_NO_TIM8_TRG_COM_TIM14 45
#define IRQ_NO_TIM8_CC 46
#define IRQ_NO_DMA1_Stream7 47
#define IRQ_NO_FMC 48
#define IRQ_NO_SDMMC1 49
#define IRQ_NO_TIM5 50
#define IRQ_NO_SPI3 51
#define IRQ_NO_UART4 52
#define IRQ_NO_UART5 53
#define IRQ_NO_TIM6_DAC 54
#define IRQ_NO_TIM7 55
#define IRQ_NO_DMA2_Stream0 56
#define IRQ_NO_DMA2_Stream1 57
#define IRQ_NO_DMA2_Stream2 58
#define IRQ_NO_DMA2_Stream3 59
#define IRQ_NO_DMA2_Stream4 60
#define IRQ_NO_ETH 61
#define IRQ_NO_ETH_WKUP 62
#define IRQ_NO_CAN2_TX 63
#define IRQ_NO_CAN2_RX0 64
#define IRQ_NO_CAN2_RX1 65
#define IRQ_NO_CAN2_SCE 66
#define IRQ_NO_OTG_FS 67
#define IRQ_NO_DMA2_Stream5 68
#define IRQ_NO_DMA2_Stream6 69
#define IRQ_NO_DMA2_Stream7 70
#define IRQ_NO_USART6 71
#define IRQ_NO_I2C3_EV 72
#define IRQ_NO_I2C3_ER 73
#define IRQ_NO_OTG_HS_EP1_OUT 74
#define IRQ_NO_OTG_HS_EP1_IN 75
#define IRQ_NO_OTG_HS_WKUP 76
#define IRQ_NO_OTG_HS 77
#define IRQ_NO_DCMI 78
#define IRQ_NO_CRYP 79
#define IRQ_NO_HASH_RNG 80
#define IRQ_NO_FPU 81
#define IRQ_NO_UART7 82
#define IRQ_NO_UART8 83
#define IRQ_NO_SPI4 84
#define IRQ_NO_SPI5 85
#define IRQ_NO_SPI6 86
#define IRQ_NO_SAI1 87
#define IRQ_NO_LCD_TFT 88
#define IRQ_NO_LCD_TFTE 89
#define IRQ_NO_DMA2D 90
#define IRQ_NO_SAI2 91
#define IRQ_NO_QuadSPI 92
#define IRQ_NO_LP Timer1 93
#define IRQ_NO_HDMI_CEC 94
#define IRQ_NO_I2C4_EV 95
#define IRQ_NO_I2C4_ER 96
#define IRQ_NO_SPDIFRX 97
#define IRQ_NO_DSIHOST 98
#define IRQ_NO_DFSDM1_FLT0 99
#define IRQ_NO_DFSDM1_FLT1 100
#define IRQ_NO_DFSDM1_FLT2 101
#define IRQ_NO_DFSDM1_FLT3 102
#define IRQ_NO_SDMMC2 103
#define IRQ_NO_CAN3_TX 104
#define IRQ_NO_CAN3_RX0 105
#define IRQ_NO_CAN3_RX1 106
#define IRQ_NO_CAN3_SCE 107
#define IRQ_NO_JPEG 108
#define IRQ_NO_MDIOS 109

/*
 *   Macro CÁC MỨC ƯU TIÊN NGẮT
 */

#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI1 1
#define NVIC_IRQ_PRI2 2
#define NVIC_IRQ_PRI3 3
#define NVIC_IRQ_PRI4 4
#define NVIC_IRQ_PRI5 5
#define NVIC_IRQ_PRI6 6
#define NVIC_IRQ_PRI7 7
#define NVIC_IRQ_PRI8 8
#define NVIC_IRQ_PRI9 9
#define NVIC_IRQ_PRI10 10
#define NVIC_IRQ_PRI11 11
#define NVIC_IRQ_PRI12 12
#define NVIC_IRQ_PRI13 13
#define NVIC_IRQ_PRI14 14
#define NVIC_IRQ_PRI15 15

/*
 *  Một số Macro khác
 */

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET ENABLE
#define GPIO_PIN_RESET DISABLE
#define FLAG_RESET 0
#define FLAG_SET 1

/**********************************************************************************************************************
 *                      Các macros cho các vị trí bit của SPI
 *********************************************************************************************************************/

// Các bit trong  Thanh ghi  SPI_CR1

#define SPI_CR1_CPHA 0
#define SPI_CR1_CPOL 1
#define SPI_CR1_MSTR 2
#define SPI_CR1_BR 3
#define SPI_CR1_SPE 6
#define SPI_CR1_LSBFIRST 7
#define SPI_CR1_SSI 8
#define SPI_CR1_SSM 9
#define SPI_CR1_RXONLY 10
#define SPI_CR1_CRCL 11
#define SPI_CR1_CRCNEXT 12
#define SPI_CR1_CRCEN 13
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMODE 15

// Các bit trong  Thanh ghi  SPI_CR2
#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAE 1
#define SPI_CR2_SSOE 2
#define SPI_CR2_NSSP 3
#define SPI_CR2_FRF 4
#define SPI_CR2_ERRIE 5
#define SPI_CR2_RXNEIE 6
#define SPI_CR2_TXEIE 7
#define SPI_CR2_DS 8
#define SPI_CR2_FRXTH 12
#define SPI_CR2_LDMA_RX 13
#define SPI_CR2_LDMA_TX 14

// Các bit trong  Thanh ghi  SPI_SR   ( status)

#define SPI_SR_RXNE 0
#define SPI_SR_TXE 1
#define SPI_SR_CHSIDE 2
#define SPI_SR_UDR 3
#define SPI_SR_CRCERR 4
#define SPI_SR_MODF 5
#define SPI_SR_OVR 6
#define SPI_SR_BSY 7
#define SPI_SR_FRE 8
#define SPI_SR_FRLVL 9
#define SPI_SR_FTLVL 11


// Các cờ thông báo trong SPI
#define SPI_TXE_FLAG  (1<<SPI_SR_TXE)
#define SPI_RXE_FLAG  (1<<SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG  (1<<SPI_SR_CHSIDE)
#define SPI_UDR_FLAG  (1<<SPI_SR_UDR)
#define SPI_CRCERR_FLAG  (1<<SPI_SR_CRCERR)
#define SPI_SR_MODF_FLAG  (1<<SPI_SR_MODF)
#define SPI_OVR_FLAG  (1<<SPI_SR_OVR)
#define SPI_BSY_FLAG  (1<<SPI_SR_BSY)
#define SPI_FRE_FLAG  (1<<SPI_SR_FRE)
#define SPI_FRLVL_FLAG  (1<<SPI_SR_FRLVL)
#define SPI_FTLVL_FLAG  (1<<SPI_SR_FTLVL)





#include "stm32f767xx_gpio_driver.h"
// #include "stm32f767xx_spi_driver.h"
//#include "scheduler_driver.h"

#endif /* INC_STM32F767XX_H_ */
