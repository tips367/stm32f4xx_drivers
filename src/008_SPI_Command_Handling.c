/*
 * 008_SPI_Command_Handling.c
 *
 *  Created on: Jun 23, 2021
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
#include<stdio.h>
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

extern void initialise_monitor_handles();

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

// arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

// arduino led

#define LED_PIN  9

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

	// MISO pin config
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

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

uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == (uint8_t)0xF5)
	{
		// ack
		return 1;
	}
	return 0;
}

int main(void)
{
	initialise_monitor_handles();

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

		//1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandCode, 1);

		uint8_t dummy_read;
		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		uint8_t dummy_write = 0xff;
		//Send some dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		//end of COMMAND_LED_CTRL

		//2. CMD_SENOSR_READ   <analog pin number(1) >

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues
		delay();

		commandCode = COMMAND_SENSOR_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = ANALOG_PIN0;

			// send arguments
			SPI_SendData(SPI2, args, 1); // sending one byte

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay so that slave can be ready with the data
			delay();

			// Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d\n", analog_read);
		}
		//end of CMD_SENOSR_READ

		//3. CMD_LED_READ 	 <pin no(1) >

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues
		delay();

		commandCode = COMMAND_LED_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can be ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("COMMAND_READ_LED %d\n",led_status);
		}
		//end of CMD_LED_READ

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues
		delay();

		commandCode = COMMAND_PRINT;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		uint8_t message[] = "Hello Arduino ! STM32 this side.";

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = strlen((char*)message);

			// send arguments
			SPI_SendData(SPI2, args, 1); //sending message length

			// send message
			SPI_SendData(SPI2, message, args[0]);

			printf("COMMAND_PRINT Executed \n");
		}
		// end of CMD_PRINT

		//5. CMD_ID_READ

		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues
		delay();

		commandCode = COMMAND_ID_READ;

		// send command
		SPI_SendData(SPI2, &commandCode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		uint8_t id[11];

		if(SPI_VerifyResponse(ackByte))
		{
			// read 10 bytes id from the slave
			for(uint32_t i = 0 ; i < 10 ; i++)
			{
				// send dummy byte to fetch data from slave
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}
			id[11] = '\0';
			printf("COMMAND_ID : %s \n",id);
		}
		// end of CMD_ID_READ

		// confirm SPI is not busy before disabling SPI peripheral
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

