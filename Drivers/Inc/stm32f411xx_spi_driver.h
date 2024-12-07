/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Aug 6, 2024
 *      Author: Jorge Acosta
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

typedef struct {
    uint8_t SPI_DeviceMode;     	// Defines whether the SPI device operates in master (1) or slave (0) mode
    uint8_t SPI_BusConfig;      	// SPI bus configuration: full-duplex (0), half-duplex (1), simplex transmit only (2), or simplex receive only (3)
    uint8_t SPI_SclkSpeed;          // SPI speed: defines the communication speed, based on clock dividers
    uint8_t SPI_DataFrameFormat;	// Data frame size: 8 bits (0) or 16 bits (1)
    uint8_t SPI_CPHA;           	// Clock phase: data capture on the first clock edge (0) or the second clock edge (1)
    uint8_t SPI_CPOL;           	// Clock polarity: clock idle low (0) or clock idle high (1)
    uint8_t SPI_SSM;            	// Slave select management: hardware (0) or software (1)

} SPI_Config_t;

typedef struct{
	SPI_Reg_Def_t *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;

/*SPI Device Mode*/
#define SPI_DEVICE_MODE_MASTER 	1
#define SPI_DEVICE_MODE_SLAVE 	0

/*SPI Bus Config*/
#define SPI_BUS_CONFIG_FD		1	//Full duplex
#define SPI_BUS_CONFIG_HD 		2	//Half duplex
#define SPI_BUS_CONFIG_S_RXONLY	3	//Simplex rx

/*SPI SclkSpeed*/
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4 	1
#define SPI_SCLK_SPEED_DIV8 	2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32 	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/*SPI_DataFrameFormat*/
#define SPI_DATAFRAME_8BIT	0
#define SPI_DATAFRAME_16BIT	1

/*SPI CPHA*/
#define SPI_CPHA_FIRST 	0
#define SPI_CPHA_SECOND	1

/*SPI_CPOL*/
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

/*General Macros*/
#define SPI_SLAVE	0
#define SPI_MASTER	1

/*Bit Positions*/
#define CPHA_BIT		0
#define CPOL_BIT		1
#define MSTR_BIT		2
#define BR_BIT			3
#define SPE_BIT			6
#define RXONLY_BIT		10
#define DFF_BIT			11
#define BIDIMODE_BIT	15


/*Pheripheral clock setup*/
void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx, uint8_t En_Dis);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Reg_Def_t *pSPIx);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
