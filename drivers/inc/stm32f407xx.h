/*
 * stm32f407xx.h
 *
 *  Created on: Aug 6, 2019
 *      Author: Tips
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

/******************************** Processor specific details**********************/

/*
 * ARM Cortex Mx processor NVIC ISERx register addresses
 */

#define NVIC_ISER0						((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1						((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2						((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3						((volatile uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx processor NVIC ICERx register addresses
 */

#define NVIC_ICER0						((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1						((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2						((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3						((volatile uint32_t*)0xE000E18C)

/*
 * ARM Cortex Mx processor priority register addresses
 */

#define NVIC_PR_BASE_ADDR				((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			4

/*base address of flash and SRAM memories */

#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM 							SRAM1_BASEADDR

/* AHBx and APBx bus peripheral base addresses */

#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U

/* Base adresses of peripherals hanging on AHB1 bus */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)

/* Base adresses of peripherals hanging on APB1 bus */

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)

/* Base adresses of peripherals hanging on APB2 bus */

#define EXTI_BASEADDR					(AHB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR					(AHB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR					(AHB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR					(AHB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(AHB2PERIPH_BASEADDR + 0x1400)


/************************ Peripheral register definition structures**************************/

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_Regdef_t;


/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;
}SYSCFG_RegDef_t;

/********Peripheral definitions (peripheral base adresses typcasted to xxx_RegDef_t)***********/

#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI 						((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)

/*************************Clock enable macros for GPIOx peripherals*****************************/

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))

/*************************Clock enable macros for I2Cx peripherals*****************************/

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))

/*************************Clock enable macros for SPIx peripherals*****************************/

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))

/*************************Clock enable macros for USARTx peripherals***************************/

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))

/*************************Clock enable macros for SYSCFG peripherals***************************/

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))

/*************************Clock disable macros for GPIOx peripherals***************************/

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 8))

/*************************Clock disable macros for I2Cx peripherals*****************************/

#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))

/*************************Clock disable macros for I2Cx peripherals*****************************/

#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))

/*************************Clock disable macros for USARTx peripherals***************************/

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))

/*************************Clock disable macros for SYSCFG peripherals***************************/

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/* macros to reset GPIOx peripherals */

#define	GPIOA_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define	GPIOB_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define	GPIOC_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define	GPIOD_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define	GPIOE_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define	GPIOF_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define	GPIOG_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define	GPIOH_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define	GPIOI_REG_RESET()		do { (RCC->AHB1RSTR |= (1 << 8));  (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/****returns port code for given GPIOx base address *********/

#define GPIO_BASEADDR_TO_CODE(x) 	( (x == GPIOA)? 0: \
								  	  (x == GPIOB)? 1: \
								  	  (x == GPIOC)? 2: \
								  	  (x == GPIOD)? 3: \
								  	  (x == GPIOE)? 4: \
								  	  (x == GPIOF)? 5: \
									  (x == GPIOG)? 6: \
									  (x == GPIOH)? 7: 8 )

/*
 * IRQ (interrupt request) numbers of stm32f407x MCU
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI15			15

/****Generic Macros *********/

#define ENABLE  				1
#define DISABLE 				0
#define SET						ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET

#endif /* INC_STM32F407XX_H_ */
