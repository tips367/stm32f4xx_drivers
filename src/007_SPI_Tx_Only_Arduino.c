/*
 * 007_SPI_Tx_Only_Arduino.c
 *
 *  Created on: June 22, 2021
 *      Author: Tips
 */

/*
 * USING GPIO PINS IN ALT FUNC MODE
 * PB14 ---> SPI2 MISO
 * PB15 ---> SPI2 MOSI
 * PB13 ---> SPI2 SCLK
 * PB12 ---> SPI2 NSS
 * ALT Functionality mode : 5
 */

#include<string.h>
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

void delay(void);

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO pin config (not required for this application)
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	// GPIO_Init(&SPIPins);

	// NSS pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;	// generates sclk of 2 MHz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;	// hardware slave management

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioBtn;

	// This is button GPIO configuration
	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioBtn);
}

int main(void)
{
	char data[] = "Hey there! Sending data...";

	GPIO_ButtonInit();

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Init();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		// wait till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// Avoid button de-bouncing related issues
		delay();

		// Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// First send length information
		uint8_t dataLen = strlen(data);
		SPI_SendData(SPI2, &dataLen, 1);

		// send data
		SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

		// confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}
