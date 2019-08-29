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
	GPIO_Handle_t gpioled, gpiobtn;

	/* LED GPIO configuration */
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&gpioled);

	GPIO_PeriClockControl(GPIOD,ENABLE);
	gpioled.pGPIOx = GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&gpioled);

	/* Button GPIO configuration */

	gpiobtn.pGPIOx = GPIOB;
	gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Init(&gpiobtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_11)==BUTTON_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_13);
			delay();
		}
	}
	return 0;
}