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
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
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
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

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
 * 	@brief				-	This function de-initializes the SPIx peripheral.
 *
 * 	@param[in]			-	Base address for SPIx peripheral
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

/********************************************************************
 * 	@function			-	SPI_GetFlagStatus
 *
 * 	@brief				-	This function returns the flag status
 *
 * 	@param[in]			-	Base address for SPIx peripheral
 * 	@param[in]			-	name of SPI related status flag
 *
 * 	@return				-	SET/RESET
 *
 * 	@note				-	none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
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
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		//1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

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

/*********************************************************************
 * @function      	  - SPI_PeripheralControl
 *
 * @brief             - enable or disable SPI peripheral
 *
 * @param[in]         - Base address for SPIx peripheral
 * @param[in]         - Enable or disable macros
 *
 * @return            - none
 *
 * @note              - none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @function      	  - SPI_SSIConfig
 *
 * @brief             - configures the SSI bit of CR1
 *
 * @param[in]         - Base address for SPIx peripheral
 * @param[in]         - Enable or disable macros
 *
 * @return            - none
 *
 * @note              - none
 */
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @function      	  - SPI_SSOEConfig
 *
 * @brief             - configures the SSOE bit of CR2
 *
 * @param[in]         - Base address for SPIx peripheral
 * @param[in]         - Enable or disable macros
 *
 * @return            - none
 *
 * @note              - none
 */
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}
