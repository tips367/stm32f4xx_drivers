/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Aug 29, 2019
 *      Author: Tips
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/** Configuration structure for SPIx peripheral **/

typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/** Handle structure for SPIx peripheral **/

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;


/************************APIs supported by this driver*********************************/

/*
 *  Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi );


/*
 *  Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 *  Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);

/*
 *  IRQ configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 *  Other peripheral control APIs
 */


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
