/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 29, 2019
 *      Author: Tips
 */

#include "stm32f407xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

/********************************************************************
 * 	@function			-	SPI_ReceiveData
 *
 * 	@brief				-	This function is for receiving data to master.
 *
 * 	@param[in]			-	Base address for SPIx peripheral
 * 	@param[in]			-   pointer to Data buffer
 * 	@param[in]			-	length of data
 *
 * 	@return				-	none
 *
 * 	@note				-	This is blocking call (Polling based)
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		//1. wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			// 16 bit DFF
			// load the data from DR to Rx buffer
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			len = len-2;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			// 8 bit DFF
			// load the data into DR
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @function      	  - SPI_IRQInterruptConfig
 *
 * @brief             - This function configures the IRQ interrupt
 *
 * @param[in]         - IRQ number
 * @param[in]         - Enable or disable
 *
 * @return            - none
 *
 * @note              - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/*********************************************************************
 * @function      	  - 	SPI_IRQPriorityConfig
 *
 * @brief             -		This function configures the priority of IRQ
 *
 * @param[in]         -		IRQ number
 * @param[in]         -		IRQ priority
 *
 * @return            -		none
 *
 * @note              -		none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection  = IRQNumber % 4 ;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED) ;
	*( NVIC_PR_BASE_ADDR + iprx ) |=  (IRQPriority << shiftAmount);
}

/********************************************************************
 * 	@function			-	SPI_SendDataIT
 *
 * 	@brief				-	This function is for sending data to slave using interrupt
 *
 * 	@param[in]			-	Handle for SPIx peripheral
 * 	@param[in]			-   pointer to Data buffer
 * 	@param[in]			-	length of data
 *
 * 	@return				-	Tx state
 *
 * 	@note				-	This is non blocking call (Interrupt based)
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t currentState = pSPIHandle->TxState;

	if(currentState != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and length information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return currentState;
}

/********************************************************************
 * 	@function			-	SPI_ReceiveDataIT
 *
 * 	@brief				-	This function is for receiving data to master using interrupt
 *
 * 	@param[in]			-	Handle for SPIx peripheral
 * 	@param[in]			-   pointer to Data buffer
 * 	@param[in]			-	length of data
 *
 * 	@return				-	Rx state
 *
 * 	@note				-	This is non blocking call (Interrupt based)
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t currentState = pSPIHandle->RxState;

	if(currentState != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and length information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		//2. Mark the SPI state as busy in reception so that
		// no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}
	return currentState;
}

/********************************************************************
 * 	@function			-	SPI_IRQHandling
 *
 * 	@brief				-
 *
 * 	@param[in]			-	Handle for SPIx peripheral
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;

	// check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		// handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		// handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
	{
		// 16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		// TxLen is zero, so close the SPI transmission and inform the application that transmission is over.
		// this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen)
	{
		// reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// 1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/********************************************************************
 * 	@function			-	SPI_CloseTransmisson
 *
 * 	@brief				-	Close the SPI transmission
 *
 * 	@param[in]			-	Handle for SPIx peripheral
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 */
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/********************************************************************
 * 	@function			-	SPI_CloseReception
 *
 * 	@brief				-	Close the SPI reception
 *
 * 	@param[in]			-	Handle for SPIx peripheral
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/********************************************************************
 * 	@function			-	SPI_ClearOVRFlag
 *
 * 	@brief				-	Clears SPI overrun flag
 *
 * 	@param[in]			-	Base address for SPIx peripheral
 *
 * 	@return				-	none
 *
 * 	@note				-	none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// This is a weak implementation, the user application may override this function.
}
