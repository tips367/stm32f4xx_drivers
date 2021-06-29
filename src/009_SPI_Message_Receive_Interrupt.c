/*
 * 009_SPI_Message_Receive_Interrupt.c
 *
 *  Created on: Jun 29, 2021
 *      Author: Tips
 */

/*
 * This application receives and prints the user message received from Arduino in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */

/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

SPI_Handle_t SPI2Handle;

#define MAX_LEN 1000

char receiveBuffer[MAX_LEN];

volatile char readByte;
volatile uint8_t receiveStop = 0;

/* This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void);

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

// This function is used to initialize the GPIO pins to behave as SPI2 pins
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

	// MISO pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

//This function is used to initialize the SPI2 peripheral parameters
void SPI2_Init(void)
{
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}

/* This function configures the gpio pin over which SPI peripheral issues
 * data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiInterruptPin;
	memset(&spiInterruptPin, 0, sizeof(spiInterruptPin));

	spiInterruptPin.pGPIOx = GPIOD;
	spiInterruptPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	spiInterruptPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiInterruptPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiInterruptPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&spiInterruptPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
}

int main(void)
{
	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

	SPI2_GPIOInit();

	SPI2_Init();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while(1)
	{
		receiveStop = 0;

		// wait till data available interrupt from transmitter device(slave)
		while(!dataAvailable);

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

		// Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!receiveStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while (SPI_SendDataIT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while (SPI_ReceiveDataIT(&SPI2Handle, &readByte, 1) == SPI_BUSY_IN_RX);
		}

		// confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Received data = %s\n", receiveBuffer);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	}

	return 0;
}

/* Runs when a data byte is received from the device over SPI */
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	static uint32_t i = 0;

	/* In the RX complete event , copy data in to receive buffer .
	 * '\0' indicates end of message (receiveStop = 1) */
	if(AppEv == SPI_EVENT_RX_COMPLETE)
	{
		receiveBuffer[i++] = readByte;
		if(readByte == '\0' || ( i == MAX_LEN))
		{
			receiveStop = 1;
			receiveBuffer[i-1] = '\0';
			i = 0;
		}
	}
}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}
