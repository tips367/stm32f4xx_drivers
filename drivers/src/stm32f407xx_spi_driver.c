/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 29, 2019
 *      Author: Tips
 */

#include "stm32f407xx_spi_driver.h"

/********************************************************************
 * 	@function			-	SPI_PeriClockControl
 *
 * 	@brief				-	This function enables or disables peripheral clock for given SPI.
 *
 * 	@param[in]			-	base address of SPI peripheral
 * 	@param[in]			-	Enable or disable macros
 * 	@param[in]
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi )
{
	if (EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/********************************************************************
 * 	@function			-	SPI_Init
 *
 * 	@brief				-	This function initializes the SPIx peripheral.
 *
 * 	@param[in]			-	Handle for SPIx peripheral
 * 	@param[in]			-
 * 	@param[in]
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}
