/*
 * stm32f4xx_spi.h
 *
 *  Created on: May 9, 2024
 *      Author: mbeuler
 */

#ifndef STM32F4XX_INCLUDE_STM32F4XX_SPI_H_
#define STM32F4XX_INCLUDE_STM32F4XX_SPI_H_

#include "stm32f4xx.h"


/* SPI register offsets */
#define SPI_O_CR1     0x00000000  // Control register 1
#define SPI_O_CR2     0x00000004  // Control register 2
#define SPI_O_SR      0x00000008  // Status register
#define SPI_O_DR      0x0000000C  // Data register

/* GPIO register offsets */
#define GPIO_O_BSRR   0x00000018  // GPIO port bit set/reset register

/* Macro to access hardware register */
#ifndef HWREG
#define HWREG(x)  (*((volatile uint32_t *)(x)))
#endif

#define SPI_MAX_INSTANCES   2

typedef enum
{
	SPI_CLK_2   = 0x00 << SPI_CR1_BR_Pos,
	SPI_CLK_4   = 0x01 << SPI_CR1_BR_Pos,
	SPI_CLK_8   = 0x02 << SPI_CR1_BR_Pos,
	SPI_CLK_16  = 0x03 << SPI_CR1_BR_Pos,
	SPI_CLK_32  = 0x04 << SPI_CR1_BR_Pos,
	SPI_CLK_64  = 0x05 << SPI_CR1_BR_Pos,
	SPI_CLK_128 = 0x06 << SPI_CR1_BR_Pos,
	SPI_CLK_256 = 0x07 << SPI_CR1_BR_Pos
}SPIBaudRatePrescaler_t;

typedef struct
{
	uint32_t ui32Base;
	uint32_t ui32Pin;
}NSS_t;

typedef enum
{
	SPI_STATE_RESET  = 0x00,
	SPI_STATE_READY  = 0x01,
	SPI_STATE_BUSY   = 0x02,
	SPI_STATE_ERROR  = 0x03
}SPIState_t;

typedef struct
{
	SPIState_t SPIState;
	uint32_t ui32Base;
	uint32_t ui32ClockSpeed; // Clock frequency of SPI_CLK pin
	bool     boCPOL;         // Clock polarity
	bool     boCPHA;         // Clock phase
	NSS_t    NSS;
}SPIConfig_t;


class SPI
{
public:
	/* Constructor, Destructor */
	SPI(uint32_t ui32Base, SPIBaudRatePrescaler_t SPIBaudRatePrescaler, bool boCPOL, bool boCPHA);
	~SPI(void);

	/* Methods */
	void enableNSS(void);
	void disableNSS(void);

	int16_t transmitSPI(uint8_t *pData, uint8_t ui8Size);
	int16_t receiveSPI(uint8_t *pData, uint8_t ui8Size);


private:
	/* Variables */
	SPIConfig_t SPIConfig;

	/* Methods */
	inline bool isValidBase();
};

#endif /* STM32F4XX_INCLUDE_STM32F4XX_SPI_H_ */
