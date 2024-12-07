/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Jun 14, 2024
 *      Author: Jorge Acosta
 */

#include "stm32f411xx_gpio_driver.h"
/* GPIO APP Implementation*/

void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t En_Dis) {

	if (En_Dis == ENABLE) {
		if (pGPIOx == GPIOA) {

			GPIOA_PERI_CLOCK_ENABLE();
		}

		else if (pGPIOx == GPIOB) {

			GPIOB_PERI_CLOCK_ENABLE();
		}

		else if (pGPIOx == GPIOC) {
			GPIOC_PERI_CLOCK_ENABLE();
		}

		else if (pGPIOx == GPIOD) {

			GPIOD_PERI_CLOCK_ENABLE();
		}

		else if (pGPIOx == GPIOE) {

			GPIOE_PERI_CLOCK_ENABLE();

		}
		else if (pGPIOx == GPIOH) {

			GPIOH_PERI_CLOCK_ENABLE();

		}

	} else if (En_Dis == DISABLE) {
		if (pGPIOx == GPIOA) {

			GPIOA_PERI_CLOCK_DISABLE();
		}

		else if (pGPIOx == GPIOB) {

			GPIOB_PERI_CLOCK_DISABLE();
		}

		else if (pGPIOx == GPIOC) {
			GPIOC_PERI_CLOCK_DISABLE();
		}

		else if (pGPIOx == GPIOD) {

			GPIOD_PERI_CLOCK_DISABLE();
		}

		else if (pGPIOx == GPIOE) {

			GPIOE_PERI_CLOCK_DISABLE();

		}
		else if (pGPIOx == GPIOH) {

			GPIOH_PERI_CLOCK_DISABLE();

		}

	}

}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0; 	// Temp register value

	/**********************************************************************************************************
	 * Pin Mode
	 */
	if (pGPIOHandle->GPIO_pinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//Apuntador modo del pin "0 input"	<<	2 * numero de pin (moder tiene campps de 2 bits)
		temp = (pGPIOHandle->GPIO_pinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));

		//Limpieza de los campos de registro
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

		// == Modo del pin en su respectivo campo
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{
		if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// Apuntador con el modo del pin << numero de pin
			temp = (pGPIOHandle->GPIO_pinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));

			// Limpieza del campo del registro
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

			// Asignación del valor al pin
			pGPIOHandle->pGPIOx->MODER |= temp;

			//CONFIGURAR EL REGISTRO FTSR y limpiar el registro RSTR
			EXTI->FTSR |= (ENABLE << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(ENABLE << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// Apuntador con el modo del pin << numero de pin
			temp = pGPIOHandle->GPIO_pinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

			//Limpieza de los campos del registro
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

			//Asignación del valor al campo del registro
			pGPIOHandle->pGPIOx->MODER |= temp;
			//CONFIGURAR EL REGISTRO RSTR y limpiar el registro FSTR
			EXTI->RTSR |= (ENABLE<<pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(ENABLE<<pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FTRT){
			// Apuntador con el modo del pin << numero de pin
			temp = (pGPIOHandle->GPIO_pinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));
			// Limpieza del campo del registro
			pGPIOHandle->pGPIOx->MODER &= ~(1<<pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			// Asignación del valor al pin
			pGPIOHandle->pGPIOx->MODER |= temp;
			//CONFIGURAR EL REGISTRO RSTR y FSTR
			EXTI->FTSR |= (ENABLE << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (ENABLE << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
		}
		uint32_t reg,pin;
		reg = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber / 4;
		pin = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber & 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PERI_CLOCK_ENABLE();
		SYSCFG->EXTICR[reg] |= portcode << (pin*4);

		// Habilitar el interruptor EXTI usando IMR
		EXTI->IMR |= (ENABLE <<pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
	}

	/**********************************************************************************************************
	 * Velocidad del pin
	 */
	temp = 0;
	// Apuntador de velocidad direccionado a  numero de pin
	temp = (pGPIOHandle->GPIO_pinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));

	// Limpieza de los campos de registro
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2* pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

	// Velocidad asignada con pin asignado (temp) hacia el registro (OSPEEDR)
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	/**********************************************************************************************************
	 *
	 * Pull up y Pull down
	 */
	temp = 0;

	// Apuntador de pullup y pulldown direccionado a numero de pin
	temp = (pGPIOHandle->GPIO_pinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));

	// Limpieza de los campos del registro
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2* pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));

	// Estado de PuPd asignado al registro
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	/**********************************************************************************************************
	 *
	 * Pull Pull / Open Drain
	 */
	temp = 0;
	// Apuntador del output type direccionado al numero de pin
	temp = (pGPIOHandle->GPIO_pinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));

	// Limpieza de campos del registro
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

	// Estado del output type asignado al registro
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	/**********************************************************************************************************
	 *
	 * Funcion alternativa
	 */
	if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_ALT){
		uint8_t high_low,pin;
		high_low = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber / 8; // Contiene si es alternate HIGH = 1 LOW = 0
		pin = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber & 8; // Contiene el campo de bit de inicio en el registro para el pin
		pGPIOHandle->pGPIOx->AFR[high_low] &= ~(0xF<< (4* pin));
		pGPIOHandle->pGPIOx->AFR[high_low] |= (pGPIOHandle->GPIO_pinConfig.GPIO_PinAltFunMode << (4 * pin));
	}




}

void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_RESET();
	}
}

uint8_t GPIO_ReadFromInputPIN(GPIO_Reg_Def_t *pGPIOx, uint8_t pinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR>>pinNumber) & 0x1);
	return value;

}

uint16_t GPIO_ReadFromInputPORT(GPIO_Reg_Def_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPIN(GPIO_Reg_Def_t *pGPIOx, uint8_t pinNumber, uint8_t value){
	if(value == GPIO_SET){
		pGPIOx->ODR |= (1<<pinNumber);
	}
	else if(value == GPIO_RESET){
		pGPIOx->ODR &= ~(1<<pinNumber);
	}

}

void GPIO_WriteToOutputPORT(GPIO_Reg_Def_t *pGPIOx, uint8_t value){
	if(value == GPIO_SET){
		pGPIOx->ODR |= 1;
	}
	else if(value == GPIO_RESET){
		pGPIOx->ODR &= ~1;
	}
}

void GPIO_ToggleOutputPIN(GPIO_Reg_Def_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1<<pinNumber);
}

/* IRQ APP Implementation */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_Dis){
	if (En_Dis == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}
	else{
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// Encontrar el registro IPR
	uint8_t iprx = IRQNumber / 4; //32 bits en cada ipr, 8 bits en cada pri, también se toma como offset

	uint8_t iprx_section = IRQNumber % 4; //Encontrar en que PRI corresponde

	uint8_t shift_value = (8 * iprx_section) + 4; //Posicionamiento dentro del pri xxxx-0000 (ceros no implementados, x es posicionamiento)

	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_value); //dirección + offset |= nivel de prioridad en la posicion del PRI correspondiente
}

void GPIO_IRQHandling(uint8_t PinNumber){
	// Validar si la bandera de ese pin está en 1
	if(EXTI->PR & (1<<PinNumber)){
		//Limpiar
		EXTI->PR = (1<<PinNumber); //This bit is cleared by programming it to ‘1’.
	}
}
