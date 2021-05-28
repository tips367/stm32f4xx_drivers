/*
 * 003_LED_Button.c
 *
 *  Created on: Aug 15, 2019
 *      Author: Tips
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define HIGH 			1
#define BUTTON_PRESSED	HIGH

void delay(void)
{
	for(uint32_t i=0;i<500000;i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed, gpioBtn;

	/* LED GPIO configuration */
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&gpioLed);

	/* Button GPIO configuration */
	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		}
	}
	return 0;
}
