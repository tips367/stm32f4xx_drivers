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
	SPI_Regdef_t *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
