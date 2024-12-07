/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Jun 14, 2024
 *      Author: Jorge Acosta
 */

#ifndef STM32F411XX_GPIO_DRIVER_H_
#define STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef struct {

	uint32_t GPIO_PinNumber; 		/* Establish the GPIO pin number. #ref GPIO_PIN_NUMBERS*/
	uint32_t GPIO_PinMode; 			/* Input / Output mode. */
	uint32_t GPIO_PinSpeed;			// Pin Speed.
	uint32_t GPIO_PinPuPdControl;	// Pull-Up or Pull-Down pin.
	uint32_t GPIO_PinOPType;		// Output Type: Pull Pull / Open Drain
	uint32_t GPIO_PinAltFunMode;	// Alternate Function.

} GPIO_PinConfig_t;

typedef struct {

	GPIO_Reg_Def_t *pGPIOx;
	GPIO_PinConfig_t GPIO_pinConfig;

} GPIO_Handle_t;

/**************************************************************
 ****************APIs supported for this driver****************
 **************************************************************/
/* Peripheral Clock Setup*/
void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t En_Dis);

/* Init and De-Init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx);

/* Data read and write*/
uint8_t GPIO_ReadFromInputPIN(GPIO_Reg_Def_t *pGPIOx, uint8_t pinNumer);
uint16_t GPIO_ReadFromInputPORT(GPIO_Reg_Def_t *pGPIOx);
void GPIO_WriteToOutputPIN(GPIO_Reg_Def_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPORT(GPIO_Reg_Def_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPIN(GPIO_Reg_Def_t *pGPIOx, uint8_t pinNumber);

/* IRQ Configuration */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnaDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/*
 * @ GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_0 	0
#define GPIO_PIN_1 	1
#define GPIO_PIN_2 	2
#define GPIO_PIN_3 	3
#define GPIO_PIN_4 	4
#define GPIO_PIN_5 	5
#define GPIO_PIN_6 	6
#define GPIO_PIN_7 	7
#define GPIO_PIN_8 	8
#define GPIO_PIN_9 	9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

/*
 * @ GPIO_PIN_MODE
 */
#define	GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4 /* Falling Trigger */
#define GPIO_MODE_IT_RT		5 /* Raising Trigger */
#define GPIO_MODE_IT_FTRT	6 /* Falling and Raising Trigger*/

/*
 * @ GPIO_PIN_SPEED
 */
#define GPIO_SPEED_LOW	0
#define GPIO_SPEED_MED	1
#define GPIO_SPEED_FAST	2
#define GPIO_SPEED_HIGH	3

/*
 * @ GPIO_PIN_PUPD
 */
#define GPIO_NO_PUPD	0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2

/*
 * GPIO_PIN_OPTYPE
 */
#define GPIO_PUSH_PULL	0
#define GPIO_OPEN_DRAIN	1

/*
 * Other Macros
 */

#define ENABLE 		1
#define DISABLE 	0
#define GPIO_SET 	1
#define GPIO_RESET	0

#endif /* STM32F411XX_GPIO_DRIVER_H_ */
