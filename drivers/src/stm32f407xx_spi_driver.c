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
	// configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. configure device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure the bus config
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Bi directional mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Bi directional mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// Bi directional mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// Rx only bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	// 5. configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/********************************************************************
 * 	@function			-	SPI_DeInit
 *
 * 	@brief				-	This function deinitializes the SPIx peripheral.
 *
 * 	@param[in]			-	Base address for SPIx peripheral
 * 	@param[in]			-
 * 	@param[in]
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

/********************************************************************
 * 	@function			-	SPI_SendData
 *
 * 	@brief				-	This function is for sending data to slave.
 *
 * 	@param[in]			-	Base address for SPIx peripheral
 * 	@param[in]			-   pointer to Data buffer
 * 	@param[in]			-	length of data
 *
 * 	@return				-	none
 *
 * 	@note				-	This is blocking call (Polling based)
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len)
{
	while (len > 0)
	{
		//1. wait until TXE is set
		while (!(pSPIx->SR & (1<<SPI_SR_TXE)));

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			// 16 bit DFF
			// load the data into DR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			len = len-2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			// load the data into DR
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}
