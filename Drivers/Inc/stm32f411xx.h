/*
 * stm32f411xx.h
 *
 *  Created on: Jun 12, 2024
 *      Author: Jorge Acosta
 */
#include<stdint.h>
#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#define	__vo volatile
/*
 * CORTEX M4 Processor specific
 * *Interrupt set-enable registers*
 */
#define NVIC_ISER0	((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1 	((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2 	((__vo uint32_t*)0xE000E108U)

/*
 * *Interrupt clear-enable registers*
 */
#define NVIC_ICER0 ((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1 ((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2 ((__vo uint32_t*)0XE000E188U)

/*
 *  *Interrupt Priority Registers
 */
#define NVIC_IPR_BASE_ADDR ((__vo uint32_t*)0xE000E400U)

/*----------------------BASE ADDRESSES BEGIN----------------------*
 *
 * *Memories*
 * All this definitions can be found in the page 42 of the reference manual
 */
#define EZ_FLASH_BASEADDR 			0x08000000U
#define EZ_ROM_BASEADDR 			0x1FFF0000U
#define EZ_SRAM1_BASEADDR			0x20000000U
#define EZ_FLASHALIASED_BASEADDR	0x00000000U

/*
 *
 * *Peripheral Buses*
 * All this definitions can be found in the page 38 of the reference manual
 */
#define EZ_AHB2_BASEADDR 	0x50000000U
#define EZ_AHB1_BASEADDR	0x40020000U
#define EZ_APB2_BASEADDR	0x40011000U
#define EZ_APB1_BASEADDR	0x40000000U

/*
 * AHB2 BUS
 */
#define EZ_USBOTGFS_BASEADDR	0x50000000U

/*
 * AHB1 BUS
 */
#define EZ_RCC_BASEADDR		0x40023800U
#define EZ_GPIOH_BASEADDR	0x40021C00U
#define EZ_GPIOE_BASEADDR	0x40021000U
#define EZ_GPIOD_BASEADDR	0x40020C00U
#define EZ_GPIOC_BASEADDR	0x40020800U
#define EZ_GPIOB_BASEADDR	0x40020400U
#define EZ_GPIOA_BASEADDR	0x40020000U

/*
 * APB2 BUS
 */
#define EZ_SPI5_BASEADDR	0x40015000U
#define EZ_EXTI_BASEADDR	0x40013C00U
#define EZ_SYSCFG_BASEADDR	0x40013800U
#define EZ_SPI4_BASEADDR	0x40013400U
#define EZ_SPI1_BASEADDR	0x40013000U

/*
 * APB1 BUS
 */
#define EZ_I2C3_BASEADDR	0x40005C00U
#define EZ_I2C2_BASEADDR	0x40005800U
#define EZ_I2C1_BASEADDR	0x40005400U
#define EZ_SPI3_BASEADDR	0x40003C00U
#define EZ_SPI2_BASEADDR	0x40003800U

/*
 * ----------------------BASE ADDRESSES END----------------------*
 */

/*
 * Peripheral Registers Definitions Structures for GPIO
 */
typedef struct {			// All these GPIO Registers can be found in page 157 of the reference manual.
	__vo uint32_t MODER; 	// Mode Register. These bits are written by software to configure I/O direction mode.												Address offset: 0x00
	__vo uint32_t OTYPER; 	// Output type register. These bits are written by software to configure the output type of the I/O port.							Address offset: 0x04
	__vo uint32_t OSPEEDR;	// Output speed register. These bits are written by software to configure the I/O output speed.										Address offset: 0x08
	__vo uint32_t PUPDR;	// Pull-up/pull-down register. These bits are written by software to configure the I/O pull-up or pull-down.						Address offset: 0x0C
	__vo uint32_t IDR; 		/* Input data register. These bits are read-only and can be accessed in word mode only. They contain the input						Address offset: 0x10
	 	 	 	 	 	 	 	value of the corresponding I/O port.*/
	__vo uint32_t ODR;		// Output data register. These bits can be read and written by software.															Address offset: 0x14
	__vo uint32_t BSRR;		// Bit set/reset register. These bits are write-only and can be accessed in word, half-word or byte mode							Address offset: 0x18
	__vo uint32_t LCKR;		// Configuration lock register. This bit can be read any time. It can only be modified using the lock key write sequence.			Address offset: 0x1C
	/*
	 * LOCK key write sequence:
	 WR LCKR[16] = ‘1’ + LCKR[15:0]
	 WR LCKR[16] = ‘0’ + LCKR[15:0]
	 WR LCKR[16] = ‘1’ + LCKR[15:0]
	 RD LCKR
	 RD LCKR[16] = ‘1’ (this read operation is optional but it confirms that the lock is active)
	 */
	__vo uint32_t AFR[2];	// Function low register. AFRL = [0], AFRH = [1],These bits are written by software to configure alternate function I/Os.			Address offset: 0x20

} GPIO_Reg_Def_t;

/*
 * Peripheral Registers Definitions Structures for SPI
 */
typedef struct{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2CSCFGR;
	__vo uint32_t SPI_I2SPR;
} SPI_Reg_Def_t;


/*
 * Peripheral Registers Definitions Structures for RCC
 */
typedef struct{
	__vo uint32_t IMR;	//
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;


}EXTI_Reg_Def_t;

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_Reg_Def_t;

typedef struct {
	__vo uint32_t RCC_CR;			// Clock control register. For this register, refer to the reference manual.																		Address offset: 0x00
	__vo uint32_t RCC_PLLCFGR;		// PLL configuration register. This register is used to configure the PLL clock outputs according to the formulas in the reference manual.			Address offset: 0x04
	__vo uint32_t RCC_CFGR;			// Clock configuration register. Access: 0 ≤ wait state ≤ 2, word, half-word and byte access 														Address offset: 0x08
									// 1 or 2 wait states inserted only if the access occurs during a clock source switch.
	__vo uint32_t RCC_CIR;			// Clock interrupt register. For this register, refer to the reference manual.																		Address offset: 0x0C
	__vo uint32_t RCC_AHB1RSTR;		// Peripheral reset register. For this register, refer to the reference manual.																		Address offset: 0x10
	__vo uint32_t RCC_AHB2RSTR;		// Peripheral reset register. For this register, refer to the reference manual.																		Address offset: 0x14
	__vo uint32_t RESERVED1;
	__vo uint32_t RESERVED2;
	__vo uint32_t RCC_APB1RSTR;		// Peripheral reset register. For this register, refer to the reference manual.																														Address offset: 0x20
	__vo uint32_t RCC_APB2RSTR;		// Peripheral reset register. For this register, refer to the reference manual.																														Address offset: 0x24
	__vo uint32_t RESERVED3;
	__vo uint32_t RESERVED4;
	__vo uint32_t RCC_AHB1ENR;		// Peripheral clock enable register.
	__vo uint32_t RCC_AHB2ENR;		// Peripheral clock enable register.
	__vo uint32_t RESERVED5;
	__vo uint32_t RESERVED6;
	__vo uint32_t RCC_APB1ENR;		// Peripheral clock enable register.
	__vo uint32_t RCC_APB2ENR;		// Peripheral clock enable register.
	__vo uint32_t RESERVED7;
	__vo uint32_t RESERVED8;
	__vo uint32_t RCC_AHB1LPENR;	// Peripheral clock enable in low power mode register.
	__vo uint32_t RCC_AHB2LPENR;	// Peripheral clock enable in low power mode register.
	__vo uint32_t RESERVED9;
	__vo uint32_t RESERVED10;
	__vo uint32_t RCC_APB1LPENR;	// Peripheral clock enable in low power mode register.
	__vo uint32_t RCC_APB2LPENR;	// Peripheral clock enable in low power mode register.
	__vo uint32_t RESERVED11;
	__vo uint32_t RESERVED12;
	__vo uint32_t RCC_BDCR;			// Backup domain control register. These bits are only reset after a Backup domain Reset
	__vo uint32_t RCC_CSR;			// Clock control & status register.
	__vo uint32_t RESERVED13;
	__vo uint32_t RESERVED14;
	__vo uint32_t RCC_SSCGR;		// Spread spectrum clock generation register.
	__vo uint32_t RCC_PLLI2SCFGR;	// Configuration register.
	__vo uint32_t RCC_DCKCFGR;		// Dedicated Clocks Configuration Register.

} RCC_Reg_Def_t;

/*
 * Peripheral Registers Definition Structures for EXTI
 */


//GPIO's Definition
#define GPIOA ((GPIO_Reg_Def_t*)EZ_GPIOA_BASEADDR)
#define GPIOB ((GPIO_Reg_Def_t*)EZ_GPIOB_BASEADDR)
#define GPIOC ((GPIO_Reg_Def_t*)EZ_GPIOC_BASEADDR)
#define GPIOD ((GPIO_Reg_Def_t*)EZ_GPIOD_BASEADDR)
#define GPIOE ((GPIO_Reg_Def_t*)EZ_GPIOE_BASEADDR)
#define GPIOH ((GPIO_Reg_Def_t*)EZ_GPIOH_BASEADDR)

// EXTI Definition
#define EXTI ((EXTI_Reg_Def_t *)EZ_EXTI_BASEADDR)

// RCC Definition
#define RCC ((RCC_Reg_Def_t *)EZ_RCC_BASEADDR)

// SYSCFG Definition
#define SYSCFG ((SYSCFG_Reg_Def_t *)EZ_SYSCFG_BASEADDR)

// SPI Definition
#define SPI1 ((SPI_Reg_Def_t *)EZ_SPI1_BASEADDR)
#define SPI2 ((SPI_Reg_Def_t *)EZ_SPI2_BASEADDR)
#define SPI3 ((SPI_Reg_Def_t *)EZ_SPI3_BASEADDR)
#define SPI4 ((SPI_Reg_Def_t *)EZ_SPI4_BASEADDR)
#define SPI5 ((SPI_Reg_Def_t *)EZ_SPI5_BASEADDR)

/***********************************************************************/
/* Source Clock Macro */
#define HSI_ON() (RCC->RCC_CR |= (1<<0))
//#define SELECT_HSI_4MHZ()	do{(RCC->RCC_CFGR &= ~(31 << 27));	/*Pre*/(RCC->RCC_CFGR |= (6 << 27)); /*MCO*/(RCC->RCC_CFGR &= ~(3 << 30));}while(0)
#define SELECT_HSI_4MHZ() do{(RCC->RCC_CFGR &= ~(63 << 21));	(RCC->RCC_CFGR |= (48 << 21));}while(0)

/* Clock Enable Macros for SYSCFG peripheral, APB2 Bus.*/
#define SYSCFG_PERI_CLOCK_ENABLE()	(RCC->RCC_APB2ENR |= (1<<0))


/*Clock Enable macros for GPIOx peripherals, AHB1 bus.*/
#define GPIOA_PERI_CLOCK_ENABLE()	(RCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PERI_CLOCK_ENABLE()	(RCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PERI_CLOCK_ENABLE()	(RCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PERI_CLOCK_ENABLE()	(RCC->RCC_AHB1ENR |= (1<<3))
#define GPIOE_PERI_CLOCK_ENABLE()	(RCC->RCC_AHB1ENR |= (1<<4))
#define GPIOH_PERI_CLOCK_ENABLE()	(RCC->RCC_AHB1ENR |= (1<<7))

/*Clock Disable  macros for GPIOx peripherals*/
#define GPIOA_PERI_CLOCK_DISABLE()	(RCC->RCC_AHB1ENR |= ~(1<<0))
#define GPIOB_PERI_CLOCK_DISABLE()	(RCC->RCC_AHB1ENR |= ~(1<<1))
#define GPIOC_PERI_CLOCK_DISABLE()	(RCC->RCC_AHB1ENR |= ~(1<<2))
#define GPIOD_PERI_CLOCK_DISABLE()	(RCC->RCC_AHB1ENR |= ~(1<<3))
#define GPIOE_PERI_CLOCK_DISABLE()	(RCC->RCC_AHB1ENR |= ~(1<<4))
#define GPIOH_PERI_CLOCK_DISABLE()	(RCC->RCC_AHB1ENR |= ~(1<<7))

/*Peripheral Clock Reset macros*/
#define GPIOA_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<3)); (RCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<4)); (RCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_RESET()	do{(RCC->RCC_AHB1RSTR |= (1<<7)); (RCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)? 0 :\
									 (x == GPIOB)? 1 :\
									 (x == GPIOC)? 2 :\
									 (x == GPIOD)? 3 :\
									 (x == GPIOE)? 4 :\
									 (x == GPIOH)? 7 :0)

#define SPI_BASEADDR_TO_CODE(x)		((x == SPI1)? 0 :\
									 (x == SPI2)? 1 :\
									 (x == SPI3)? 2 :\
									 (x == SPI4)? 3 :\
									 (x == SPI5)? 4 :0)


/*Clock Enable macros for SPIx peripherals, APB1, APB2 bus.*/
#define SPI1_PERI_CLOCK_ENABLE()	(RCC->RCC_APB2ENR |= (1<<12));
#define SPI2_PERI_CLOCK_ENABLE()	(RCC->RCC_APB1ENR |= (1<<14));
#define SPI3_PERI_CLOCK_ENABLE()	(RCC->RCC_APB1ENR |= (1<<15));
#define SPI4_PERI_CLOCK_ENABLE()	(RCC->RCC_APB2ENR |= (1<<13));
#define SPI5_PERI_CLOCK_ENABLE()	(RCC->RCC_APB2ENR |= (1<<20));

/*Clock Disable macros for SPIx peripherals, APB1, APB2 bus.*/
#define SPI1_PERI_CLOCK_DISABLE()	(RCC->RCC_APB2ENR |= ~(1<<12));
#define SPI2_PERI_CLOCK_DISABLE()	(RCC->RCC_APB1ENR |= ~(1<<14));
#define SPI3_PERI_CLOCK_DISABLE()	(RCC->RCC_APB1ENR |= ~(1<<15));
#define SPI4_PERI_CLOCK_DISABLE()	(RCC->RCC_APB2ENR |= ~(1<<13));
#define SPI5_PERI_CLOCK_DISABLE()	(RCC->RCC_APB2ENR |= ~(1<<20));

/*Peripheral Clock Reset Macros*/
#define SPI1_RESET()	do{(RCC->RCC_APB2RSTR |= (1<<12)); (RCC->RCC_APB2RSTR |= ~(1<<12));}while(0)
#define SPI2_RESET()	do{(RCC->RCC_APB1RSTR |= (1<<14)); (RCC->RCC_APB1RSTR |= ~(1<<14));}while(0)
#define SPI3_RESET()	do{(RCC->RCC_APB1RSTR |= (1<<15)); (RCC->RCC_APB1RSTR |= ~(1<<15));}while(0)
#define SPI4_RESET()	do{(RCC->RCC_APB2RSTR |= (1<<13)); (RCC->RCC_APB1RSTR |= ~(1<<13));}while(0)
#define SPI5_RESET()	do{(RCC->RCC_APB2RSTR |= (1<<20)); (RCC->RCC_APB1RSTR |= ~(1<<20));}while(0)

/*
 * Other Macros
 */

#define ENABLE 		1
#define DISABLE 	0
#define SET 		1
#define RESET		0

/************ STM32 SPECIFIC INTERRUPT NUMBERS ************/
#define EXTI0_IRQn	6
#define EXTI1_IRQn 	7


/* PRIORITY NUMBERS FOR INTERRUPTIONS*/
#define NVIC_IRQ_PRI0 	0
#define NVIC_IRQ_PRI1	1
#define NVIC_IRQ_PRI2	2
#define NVIC_IRQ_PRI3	3
#define NVIC_IRQ_PRI4	4
#define NVIC_IRQ_PRI5	5
#define NVIC_IRQ_PRI6	6
#define NVIC_IRQ_PRI7	7
#define NVIC_IRQ_PRI8	8
#define NVIC_IRQ_PRI9	9
#define NVIC_IRQ_PRI10 	10
#define NVIC_IRQ_PRI11 	11
#define NVIC_IRQ_PRI12 	12
#define NVIC_IRQ_PRI13 	13
#define NVIC_IRQ_PRI14 	14
#define NVIC_IRQ_PRI15 	15





#endif /* INC_STM32F411XX_H_ */
