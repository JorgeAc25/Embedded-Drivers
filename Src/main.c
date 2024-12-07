/*
 * main.c
 *
 *  Created on: Jul 12, 2024
 *      Author: Jorge Acosta
 */

#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx.h"
#include <string.h>

void Clock_Init(void);
void GPIOx_Init(void);
void delay();

uint8_t interrupt_flag = 0;

int main() {
	Clock_Init();
	GPIOx_Init();
	GPIO_IRQInterruptConfig(EXTI1_IRQn, ENABLE);
	GPIO_IRQPriorityConfig(EXTI1_IRQn, NVIC_IRQ_PRI14);

	while (1) {
		if (interrupt_flag == 1) {
			delay();
			GPIO_ToggleOutputPIN(GPIOA, GPIO_PIN_5);
			interrupt_flag--;
		}
	}

}

void Clock_Init() {
	HSI_ON();
	SELECT_HSI_4MHZ();

}
void GPIOx_Init(void) {

	//LED
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Handle_t GPIO_LED_CONFIG;

	memset(&GPIO_LED_CONFIG, 0, sizeof(GPIO_LED_CONFIG));
	GPIO_LED_CONFIG.pGPIOx = GPIOA;
	GPIO_LED_CONFIG.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED_CONFIG.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_LED_CONFIG.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_LED_CONFIG.GPIO_pinConfig.GPIO_PinOPType = GPIO_PUSH_PULL;
	GPIO_LED_CONFIG.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_LED_CONFIG.GPIO_pinConfig.GPIO_PinAltFunMode = DISABLE;

	GPIO_Init(&GPIO_LED_CONFIG);

	//Button
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Handle_t GPIO_BUTTON_CONFIG;

	memset(&GPIO_BUTTON_CONFIG, 0, sizeof(GPIO_BUTTON_CONFIG));
	GPIO_BUTTON_CONFIG.pGPIOx = GPIOC;
	GPIO_BUTTON_CONFIG.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_IT_FTRT;
	GPIO_BUTTON_CONFIG.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_BUTTON_CONFIG.GPIO_pinConfig.GPIO_PinOPType = GPIO_PUSH_PULL;
	GPIO_BUTTON_CONFIG.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_BUTTON_CONFIG.GPIO_pinConfig.GPIO_PinAltFunMode = DISABLE;

	GPIO_Init(&GPIO_BUTTON_CONFIG);

}
/*
void EXTI0_IRQHandler() {
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	interrupt_flag++;
}
*/
void EXTI1_IRQHandler(){
	delay();
	GPIO_IRQHandling(GPIO_PIN_1);
	interrupt_flag++;
}

void delay() {
	for (uint32_t i = 0; i < 50000; i++)
		;
}
