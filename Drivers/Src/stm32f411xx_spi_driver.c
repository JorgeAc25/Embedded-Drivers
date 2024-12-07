/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Jorge Acosta
 */

#include "stm32f411xx_spi_driver.h"

void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx, uint8_t En_Dis) {
	if (En_Dis == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PERI_CLOCK_ENABLE();
		} else if (pSPIx == SPI2) {
			SPI2_PERI_CLOCK_ENABLE();

		} else if (pSPIx == SPI3) {
			SPI3_PERI_CLOCK_ENABLE();

		} else if (pSPIx == SPI4) {
			SPI4_PERI_CLOCK_ENABLE();

		} else if (pSPIx == SPI5) {
			SPI5_PERI_CLOCK_ENABLE();
		}
	}
	else{
		if (pSPIx == SPI1) {
			SPI1_PERI_CLOCK_DISABLE();
		} else if (pSPIx == SPI2) {
			SPI2_PERI_CLOCK_DISABLE();

		} else if (pSPIx == SPI3) {
			SPI3_PERI_CLOCK_DISABLE();

		} else if (pSPIx == SPI4) {
			SPI4_PERI_CLOCK_DISABLE();

		} else if (pSPIx == SPI5) {
			SPI5_PERI_CLOCK_DISABLE();
		}
	}
}
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint32_t tempreg = 0;
	// Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Master/Slave Mode
	tempreg = (pSPIHandle->SPI_Config.SPI_DeviceMode << MSTR_BIT);

	// Bus Configuration
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		tempreg &= ~(1<<BIDIMODE_BIT);
	}

	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		tempreg |= (1<<BIDIMODE_BIT);
	}

	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY){
		if(pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE){
			tempreg &= ~(1<<BIDIMODE_BIT);
			tempreg |= (RXONLY_BIT);

		}
		else{
			tempreg &= ~(1<<BIDIMODE_BIT);
		}
	}


	// SPI Clock Speed
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << BR_BIT);

	// Set the DFF bit to define 8- or 16-bit data frame format
	tempreg |= (pSPIHandle->SPI_Config.SPI_DataFrameFormat << DFF_BIT);

	// CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA<<CPHA_BIT);

	// CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL<<CPOL_BIT);

	// Write in SPI_CR1
	pSPIHandle->pSPIx->SPI_CR1 = tempreg;

	// SPI Enable
	pSPIHandle->pSPIx->SPI_CR1 |= (ENABLE<<SPE_BIT); //Careful with this one
}

void SPI_DeInit(SPI_Reg_Def_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_RESET();
	}
	else if (pSPIx == SPI2){
		SPI2_RESET();
	}
	else if (pSPIx == SPI3){
		SPI3_RESET();
	}
	else if (pSPIx == SPI4){
		SPI4_RESET();
	}
	else if (pSPIx == SPI5){
		SPI5_RESET();
	}
	SPI_PeriClockControl(pSPIx, DISABLE);
}
