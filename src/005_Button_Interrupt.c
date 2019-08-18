/*
 * 005_GPIO_Button_interrupt.c
 *
 *  Created on: Aug 18, 2019
 *      Author: Tips
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0;i<500000;i++);
}

int main(void)
{
	GPIO_Handle_t gpioled, gpiobtn;
	memset(&gpioled,0,sizeof(gpioled));
	memset(&gpiobtn,0,sizeof(gpiobtn));

	/* LED GPIO configuration */
	gpioled.pGPIOx = GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&gpioled);
	//gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	//GPIO_Init(&gpioled);

	/* Button GPIO configuration */

	gpiobtn.pGPIOx = GPIOA;
	gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&gpiobtn);

	GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_RESET);
	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);
	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_13);
}
