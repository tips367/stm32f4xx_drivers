/*
 * 004_LED_Button_ext.c
 *
 *  Created on: Aug 18, 2019
 *      Author: Tips
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define HIGH 			1
#define LOW				0
#define BUTTON_PRESSED	LOW

void delay(void)
{
	for(uint32_t i=0;i<500000;i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed, gpioBtn;

	/* LED GPIO configuration */
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);

	GPIO_PeriClockControl(GPIOD, ENABLE);
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&gpioLed);

	/* Button GPIO configuration */
	gpioBtn.pGPIOx = GPIOB;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&gpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_11) == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		}
	}
	return 0;
}
