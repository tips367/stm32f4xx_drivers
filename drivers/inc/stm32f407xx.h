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

/* base addresses of flash and SRAM memories */

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

/* Base addresses of peripherals hanging on AHB1 bus */

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

/* Base addresses of peripherals hanging on APB1 bus */

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)

/* Base addresses of peripherals hanging on APB2 bus */

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)


/************************ Peripheral register definition structures**************************/

/**
  * @brief General Purpose I/O
  */

typedef struct
{
	volatile uint32_t MODER;		/*	GPIO port mode register 				*/
	volatile uint32_t OTYPER;		/*	GPIO port output type register 			*/
	volatile uint32_t OSPEEDR;		/*	GPIO port output speed register			*/
	volatile uint32_t PUPDR;		/*	GPIO port pull-up/pull-down register	*/
	volatile uint32_t IDR;			/*	GPIO port input data register 			*/
	volatile uint32_t ODR;			/*	GPIO port output data register 			*/
	volatile uint32_t BSRR;			/*	GPIO port bit set/reset register 		*/
	volatile uint32_t LCKR;			/*	GPIO port configuration lock register 	*/
	volatile uint32_t AFR[2];		/*	GPIO alternate function registers 		*/
}GPIO_RegDef_t;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
	volatile uint32_t CR;			/*  RCC clock control register        	 							*/
	volatile uint32_t PLLCFGR;		/*  RCC PLL configuration register        	 						*/
	volatile uint32_t CFGR;			/*  RCC clock configuration register        	 					*/
	volatile uint32_t CIR;			/*  RCC clock interrupt register        	 						*/
	volatile uint32_t AHB1RSTR;		/*  RCC AHB1 peripheral reset register       	 					*/
	volatile uint32_t AHB2RSTR;		/*  RCC AHB2 peripheral reset register        	 					*/
	volatile uint32_t AHB3RSTR;		/*  RCC AHB3 peripheral reset register        	 					*/
	uint32_t RESERVED0;				/*  Reserved        	 											*/
	volatile uint32_t APB1RSTR;		/*  RCC APB1 peripheral reset register        	 					*/
	volatile uint32_t APB2RSTR;		/*  RCC APB2 peripheral reset register        	 					*/
	uint32_t RESERVED1[2];			/*  Reserved        	 											*/
	volatile uint32_t AHB1ENR;		/*  RCC AHB1 peripheral clock enable register        	 			*/
	volatile uint32_t AHB2ENR;		/*  RCC AHB2 peripheral clock enable register        	 			*/
	volatile uint32_t AHB3ENR;		/*  RCC AHB3 peripheral clock enable register        	 			*/
	uint32_t RESERVED2;				/*  Reserved       	 												*/
	volatile uint32_t APB1ENR;		/*  RCC APB1 peripheral clock enable register        	 			*/
	volatile uint32_t APB2ENR;		/*  RCC APB2 peripheral clock enable register       	 			*/
	uint32_t RESERVED3[2];			/*  Reserved        	 											*/
	volatile uint32_t AHB1LPENR;	/*  RCC AHB1 peripheral clock enable in low power mode register 	*/
	volatile uint32_t AHB2LPENR;	/*  RCC AHB2 peripheral clock enable in low power mode register 	*/
	volatile uint32_t AHB3LPENR;	/*  RCC AHB3 peripheral clock enable in low power mode register 	*/
	uint32_t RESERVED4;				/*  Reserved        	 											*/
	volatile uint32_t APB1LPENR;	/*  RCC APB1 peripheral clock enable in low power mode register 	*/
	volatile uint32_t APB2LPENR;	/*  RCC APB2 peripheral clock enable in low power mode register     */
	uint32_t RESERVED5[2];			/*  Reserved        	 											*/
	volatile uint32_t BDCR;			/*  RCC Backup domain control register        	 					*/
	volatile uint32_t CSR;			/*  RCC clock control & status register        	 					*/
	uint32_t RESERVED6[2];			/*  Reserved        	 											*/
	volatile uint32_t SSCGR;		/*  RCC spread spectrum clock generation register        	 		*/
	volatile uint32_t PLLI2SCFGR;	/*  RCC PLLI2S configuration register        	 					*/
}RCC_RegDef_t;

/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
	volatile uint32_t IMR;			/* EXTI Interrupt mask register 			*/
	volatile uint32_t EMR;			/* EXTI Event mask register 				*/
	volatile uint32_t RTSR;			/* EXTI Rising trigger selection register 	*/
	volatile uint32_t FTSR;			/* EXTI Falling trigger selection register 	*/
	volatile uint32_t SWIER;		/* EXTI Software interrupt event register 	*/
	volatile uint32_t PR;			/* EXTI Pending register 					*/
}EXTI_RegDef_t;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
	volatile uint32_t CR1;			/* SPI control register 1 (not used in I2S mode) 		*/
	volatile uint32_t CR2;			/* SPI control register 2  								*/
	volatile uint32_t SR;			/* SPI status register 									*/
	volatile uint32_t DR;			/* SPI data register 									*/
	volatile uint32_t CRCPR;		/* SPI CRC polynomial register (not used in I2S mode) 	*/
	volatile uint32_t RXCRCR;		/* SPI RX CRC register (not used in I2S mode) 			*/
	volatile uint32_t TXCRCR;		/* SPI TX CRC register (not used in I2S mode) 			*/
	volatile uint32_t I2SCFGR;		/* SPI_I2S configuration register 						*/
	volatile uint32_t I2SPR;		/* SPI_I2S prescaler register 							*/
}SPI_RegDef_t;


/**
  * @brief System configuration controller
  */

typedef struct
{
	volatile uint32_t MEMRMP;		/* SYSCFG memory remap register 						*/
	volatile uint32_t PMC;			/* SYSCFG peripheral mode configuration register 		*/
	volatile uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration registers 	*/
	uint32_t RESERVED1[2];			/* Reserved												*/
	volatile uint32_t CMPCR;		/* SYSCFG Compensation cell control register 			*/
}SYSCFG_RegDef_t;

/********Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)***********/

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
#define SPI4						((SPI_RegDef_t*)SPI4_BASEADDR)

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

/*************************Clock disable macros for SPIx peripherals*****************************/

#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))

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

/************************************************
 * Bit position definitions of SPI peripheral
 ************************************************/

/*
 * Bit position definitions of SPI_CR1 register
 */

#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 * Bit position definitions of SPI_CR2 register
 */

#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bit position definitions of SPI_SR register
 */

#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


#endif /* INC_STM32F407XX_H_ */
