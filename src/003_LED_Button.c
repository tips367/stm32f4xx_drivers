/*
 * 002_LED_Button.c
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
	GPIO_Handle_t gpioled, gpiobtn;

	/* LED GPIO configuration */
	gpioled.pGPIOx = GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&gpioled);
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&gpioled);

	/* Button GPIO configuration */

	gpiobtn.pGPIOx = GPIOA;
	gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&gpiobtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)==BUTTON_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_13);
			delay();
		}
	}
	return 0;
}
