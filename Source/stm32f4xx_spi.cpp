/*
 * stm32f4xx_spi.cpp
 *
 *  Created on: May 9, 2024
 *      Author: mbeuler
 */

/*****************************************
 * Revision of used documents
 * - DS10314 : Rev  8
 * - RM0383  : Rev  4
 *****************************************/

#include "stm32f4xx_spi.h"


/* SPI class */
SPI::SPI(uint32_t ui32Base, SPIBaudRatePrescaler_t SPIBaudRatePrescaler, bool boCPOL, bool boCPHA)
{
	/* General SPI settings (fixed within SPI class instantiation)
	 * Mode         : Full Duplex Master (BIDIMODE = 0, RXONLY = 0, MSTR = 1)
	 * CRC calc     : Disabled (CRCEN = 0)
	 * Data frame   : 8-bit (DFF = 0)
	 * First bit    : MSB first (LSBFIRST = 0)
	 * NSS          : Software mode (SSM = 1, SSI = 1, RM0383 p. 564)
	 * Frame format : SPI Motorola mode (FRF = 0) */

	uint32_t ui32APB1Clock, ui32APB2Clock, ui32ClockSpeed;
	uint8_t  ui8PPRE1, ui8PPRE2;

	SPIConfig.SPIState = SPI_STATE_RESET;
	SPIConfig.ui32Base   = ui32Base;
	SPIConfig.boCPOL     = boCPOL;  // Clock polarity
	SPIConfig.boCPHA     = boCPHA;  // Clock phase

	/* Check base address */
	if (!isValidBase()) {return;};

	/* Disable SPI peripheral (default) */
	HWREG(ui32Base + SPI_O_CR1) = 0x0000;  // Reset value
	HWREG(ui32Base + SPI_O_CR2) = 0x0000;  // Reset value

	/* Enable clock and alternate function, set all pins to "High speed"
	 * This depends on the selected SPI, therefore HWREG macro cannot be used */
	switch(ui32Base)
	{
	case SPI1_BASE:                                         // AF = 5, NSS = PA4 (SW controlled), SCK = PA5, MISO = PB4, MOSI = PB5
		                                                    // (DS10314 p. 48 f.)
		RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOAEN;              // Enable GPIOA clock
		RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOBEN;              // Enable GPIOB clock
		RCC->APB2ENR   |= RCC_APB2ENR_SPI1EN;               // Enable SPI1 clock (APB2, RM0383 p. 39)
		GPIOA->MODER   &= ~GPIO_MODER_MODER4_Msk;           // Clear pin mode for PA4
		GPIOA->MODER   |= (1 << GPIO_MODER_MODER4_Pos);     // Set output mode for PA4 (NSS is controlled by software)
		GPIOA->BSRR     = GPIO_BSRR_BS4;                    // Set ODR4 bit in GPIOA_ODR register (NSS disabled)
		GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
		GPIOA->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED4_Pos);  // PA4 is set to "Fast speed"
		//
		GPIOA->AFR[0]  &= ~GPIO_AFRL_AFSEL5_Msk;            //
		GPIOA->AFR[0]  |= (5 << GPIO_AFRL_AFSEL5_Pos);      // Alternate function 'SPI1_SCK' for PA5
		GPIOA->MODER   &= ~GPIO_MODER_MODER5_Msk;           //
		GPIOA->MODER   |= (2 << GPIO_MODER_MODER5_Pos);     // Enable alternate function for PA5
		GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_Msk;
		GPIOA->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED5_Pos);  // PA5 is set to "Fast speed"
		//
		GPIOB->AFR[0]  &= ~GPIO_AFRL_AFSEL4_Msk;            //
		GPIOB->AFR[0]  |= (5 << GPIO_AFRL_AFSEL4_Pos);      // Alternate function 'SPI1_MISO' for PB4
		GPIOB->MODER   &= ~GPIO_MODER_MODER4_Msk;           //
		GPIOB->MODER   |= (2 << GPIO_MODER_MODER4_Pos);     // Enable alternate function for PB4
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
		GPIOB->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED4_Pos);  // PB4 is set to "Fast speed"
		GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPD4_Msk;
		GPIOB->PUPDR   |= (1 << GPIO_PUPDR_PUPD4_Pos);      // PB4 is set to "Pull-up"  --> Check if this is necessary!
		//
		GPIOB->AFR[0]  &= ~GPIO_AFRL_AFSEL5_Msk;            //
		GPIOB->AFR[0]  |= (5 << GPIO_AFRL_AFSEL5_Pos);      // Alternate function 'SPI1_MOSI' for PB5
		GPIOB->MODER   &= ~GPIO_MODER_MODER5_Msk;           //
		GPIOB->MODER   |= (2 << GPIO_MODER_MODER5_Pos);     // Enable alternate function for PB5
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_Msk;
		GPIOB->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED5_Pos);  // PB5 is set to "Fast speed"
		//
		break;

	case SPI2_BASE:                                         // AF = 5, NSS = PB12 (SW controlled), SCK = PB13, MISO = PB14, MOSI = PB15
		                                                    // (DS10314 p. 49)
		RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOBEN;              // Enable GPIOB clock
		RCC->APB1ENR   |= RCC_APB1ENR_SPI2EN;               // Enable SPI2 clock (APB1, RM0383 p. 39)
		GPIOB->MODER   &= ~GPIO_MODER_MODER12_Msk;          // Clear pin mode for PB12
		GPIOB->MODER   |= (1 << GPIO_MODER_MODER12_Pos);    // Set output mode for PB12 (NSS is controlled by software)
		GPIOB->BSRR     = GPIO_BSRR_BS12;                   // Set ODR12 bit in GPIOB_ODR register (NSS disabled)
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12_Msk;
		GPIOB->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED12_Pos); // PB12 is set to "Fast speed"
		//
		GPIOB->AFR[1]  &= ~GPIO_AFRH_AFSEL13_Msk;           //
		GPIOB->AFR[1]  |= (5 << GPIO_AFRH_AFSEL13_Pos);     // Alternate function 'SPI2_SCK' for PB13
		GPIOB->MODER   &= ~GPIO_MODER_MODER13_Msk;          //
		GPIOB->MODER   |= (2 << GPIO_MODER_MODER13_Pos);    // Enable alternate function for PB13
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED13_Msk;
		GPIOB->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED13_Pos); // PB13 is set to "Fast speed"
		//
		GPIOB->AFR[1]  &= ~GPIO_AFRH_AFSEL14_Msk;           //
		GPIOB->AFR[1]  |= (5 << GPIO_AFRH_AFSEL14_Pos);     // Alternate function 'SPI2_MISO' for PB14
		GPIOB->MODER   &= ~GPIO_MODER_MODER14_Msk;          //
		GPIOB->MODER   |= (2 << GPIO_MODER_MODER14_Pos);    // Enable alternate function for PB14
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED14_Msk;
		GPIOB->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED14_Pos); // PB14 is set to "Fast speed"
		GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPD14_Msk;
		GPIOB->PUPDR   |= (1 << GPIO_PUPDR_PUPD14_Pos);     // PB14 is set to "Pull-up"  --> Check if this is necessary!
		//
		GPIOB->AFR[1]  &= ~GPIO_AFRH_AFSEL15_Msk;           //
		GPIOB->AFR[1]  |= (5 << GPIO_AFRH_AFSEL15_Pos);     // Alternate function 'SPI2_MOSI' for PB15
		GPIOB->MODER   &= ~GPIO_MODER_MODER15_Msk;          //
		GPIOB->MODER   |= (2 << GPIO_MODER_MODER15_Pos);    // Enable alternate function for PB15
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED15_Msk;
		GPIOB->OSPEEDR |= (2 << GPIO_OSPEEDR_OSPEED15_Pos); // PB15 is set to "Fast speed"
		//
		break;

	default:
		/* We will not get here */
		SPIConfig.SPIState = SPI_STATE_ERROR;
		return;
	}

	switch(ui32Base)
	{
	case SPI1_BASE: SPIConfig.NSS.ui32Base = GPIOA_BASE; // GPIOA
	                SPIConfig.NSS.ui32Pin  = 0x00000010; // Pin 4
	                break;

	case SPI2_BASE: SPIConfig.NSS.ui32Base = GPIOB_BASE; // GPIOB
	                SPIConfig.NSS.ui32Pin  = 0x00001000; // Pin 12
	                break;

	default       : /* We will not get here */
		            SPIConfig.SPIState = SPI_STATE_ERROR;
		            return;
	}

	/* SPI initialization
	 * Get APB1 and APB2 prescaler */
	ui8PPRE1 = (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
	ui8PPRE2 = (RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;

	/* Convert PPRE1 and PPRE2 for clock calculation (RM0383 p. 107 f.) */
	switch(ui8PPRE1)
	{
	case 0x04: ui8PPRE1 = 1; // AHB clock (core clock) divided by 2  (right shift by 1 bit)
	           break;
	case 0x05: ui8PPRE1 = 2; // AHB clock (core clock) divided by 4  (right shift by 2 bits)
	           break;
	case 0x06: ui8PPRE1 = 3; // AHB clock (core clock) divided by 8  (right shift by 3 bits)
			   break;
	case 0x07: ui8PPRE1 = 4; // AHB clock (core clock) divided by 16 (right shift by 4 bits)
			   break;
	default  : ui8PPRE1 = 0; // AHB clock (core clock) not divided   (no shift operation)
			   break;
	}

	switch(ui8PPRE2)
	{
	case 0x04: ui8PPRE2 = 1; // AHB clock (core clock) divided by 2  (right shift by 1 bit)
	           break;
	case 0x05: ui8PPRE2 = 2; // AHB clock (core clock) divided by 4  (right shift by 2 bits)
	           break;
	case 0x06: ui8PPRE2 = 3; // AHB clock (core clock) divided by 8  (right shift by 3 bits)
			   break;
	case 0x07: ui8PPRE2 = 4; // AHB clock (core clock) divided by 16 (right shift by 4 bits)
			   break;
	default  : ui8PPRE2 = 0; // AHB clock (core clock) not divided   (no shift operation)
			   break;
	}

	/* Update SystemCoreClock variable with the current core clock and calculate
	 * APB1 as well as APB2 clock frequency
	 * (SPI1 is clocked by APB2, SPI2 is clocked by APB1 */
	SystemCoreClockUpdate();
	ui32APB1Clock = SystemCoreClock >> ui8PPRE1;
	ui32APB2Clock = SystemCoreClock >> ui8PPRE2;

	switch(ui32Base)
	{
	case SPI1_BASE: ui32ClockSpeed = ui32APB2Clock;  // SPI1: APB2 (RM0383 p. 39)
	                break;
	case SPI2_BASE: ui32ClockSpeed = ui32APB1Clock;  // SPI2: APB1 (RM0383 p. 39)
	                break;
	default       : /* We will not get here */
		            SPIConfig.SPIState = SPI_STATE_ERROR;
		            return;
	}

	switch(SPIBaudRatePrescaler)
	{
	case SPI_CLK_2   : ui32ClockSpeed = ui32ClockSpeed / 2;
	                   break;
	case SPI_CLK_4   : ui32ClockSpeed = ui32ClockSpeed / 4;
	                   break;
	case SPI_CLK_8   : ui32ClockSpeed = ui32ClockSpeed / 8;
	                   break;
	case SPI_CLK_16  : ui32ClockSpeed = ui32ClockSpeed / 16;
	                   break;
	case SPI_CLK_32  : ui32ClockSpeed = ui32ClockSpeed / 32;
	                   break;
	case SPI_CLK_64  : ui32ClockSpeed = ui32ClockSpeed / 64;
	                   break;
	case SPI_CLK_128 : ui32ClockSpeed = ui32ClockSpeed / 128;
	                   break;
	case SPI_CLK_256 : ui32ClockSpeed = ui32ClockSpeed / 256;
	                   break;
	default          : /* We will not get here */
		               SPIConfig.SPIState = SPI_STATE_ERROR;
		               return;
	}

	HWREG(ui32Base + SPI_O_CR1) |= SPIBaudRatePrescaler;
	SPIConfig.ui32ClockSpeed = ui32ClockSpeed;

	if (boCPOL) {HWREG(ui32Base + SPI_O_CR1) |= SPI_CR1_CPOL;}  // Clock polarity
	if (boCPHA) {HWREG(ui32Base + SPI_O_CR1) |= SPI_CR1_CPHA;}  // Clock phase

	/* General SPI settings in SPI_CR1 (fixed within SPI class instantiation)
	 * Mode       : Full Duplex Master (BIDIMODE = 0, RXONLY = 0, MSTR = 1)
	 * CRC calc   : Disabled (CRCEN = 0)
	 * Data frame : 8-bit (DFF = 0)
	 * First bit  : MSB first (LSBFIRST = 0)
	 * NSS        : Software mode (SSM = 1, SSI = 1, RM0383 p. 564) */
	HWREG(ui32Base + SPI_O_CR1) |= (SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR);

	/* General SPI settings in SPI_CR2 (fixed within SPI class instantiation)
	 * Frame format : SPI Motorola mode (FRF = 0) */
	// Already done (Reset value)

	/* Enable SPI peripheral */
	HWREG(ui32Base + SPI_O_CR1) |= SPI_CR1_SPE;

	SPIConfig.SPIState = SPI_STATE_READY;
}


/* Public methods */
void SPI::enableNSS(void)
{
	switch(SPIConfig.ui32Base)
	{
	case SPI1_BASE: GPIOA->BSRR = GPIO_BSRR_BR4;  // Reset ODR4 bit in GPIOA_ODR register (NSS enabled)
	                break;
	case SPI2_BASE: GPIOB->BSRR = GPIO_BSRR_BR12; // Reset ODR12 bit in GPIOB_ODR register (NSS enabled)
			        break;
	default       : /* We will not get here */
			        break;
	}
}


void SPI::disableNSS(void)
{
	switch(SPIConfig.ui32Base)
	{
	case SPI1_BASE: GPIOA->BSRR = GPIO_BSRR_BS4;  // Set ODR4 bit in GPIOA_ODR register (NSS disabled)
			        break;
	case SPI2_BASE: GPIOB->BSRR = GPIO_BSRR_BS12; // Set ODR12 bit in GPIOB_ODR register (NSS disabled)
			        break;
	default       : /* We will not get here */
			        break;
	}
}


int16_t SPI::transmitSPI(uint8_t *pData, uint8_t ui8Size)
{
	uint8_t ui8Aux;

	if (SPIConfig.SPIState != SPI_STATE_READY) {return -1;}

	SPIConfig.SPIState = SPI_STATE_BUSY;

	/* 2-line unidirectional data mode already selected (BIDIMODE = 0, RM0383 p. 598) */
	/* Check if SPI is already enabled */
	if ((HWREG(SPIConfig.ui32Base + SPI_O_CR1) & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		HWREG(SPIConfig.ui32Base + SPI_O_CR1) |= SPI_CR1_SPE;
	}

	/* Transmit data in 8 bit mode */
	while (ui8Size > 0)
	{
		while (!(HWREG(SPIConfig.ui32Base + SPI_O_SR) & SPI_SR_TXE));  // Wait until TXE flag is set to send data
		*((__IO uint8_t *) & HWREG(SPIConfig.ui32Base + SPI_O_DR)) = *pData++;
		ui8Size--;
	}

	/* Wait until TXE flag is set to send data */
	while (!(HWREG(SPIConfig.ui32Base + SPI_O_SR) & SPI_SR_TXE));

	/* Wait until SPI is not busy */
	while (HWREG(SPIConfig.ui32Base + SPI_O_SR) & SPI_SR_BSY);

	/* Clear overrun flag in 2-line communication mode by reading DR and SR because received data
	 * is not used */
	ui8Aux = HWREG(SPIConfig.ui32Base + SPI_O_DR);
	ui8Aux = HWREG(SPIConfig.ui32Base + SPI_O_SR);

	(void) ui8Aux; // Work around compiler warning

	SPIConfig.SPIState = SPI_STATE_READY;

	return 0;
}


int16_t SPI::receiveSPI(uint8_t *pData, uint8_t ui8Size)
{
	if (SPIConfig.SPIState != SPI_STATE_READY) {return -1;}

	SPIConfig.SPIState = SPI_STATE_BUSY;

	/* 2-line unidirectional data mode already selected (BIDIMODE = 0, RM0383 p. 598) */
	/* Check if SPI is already enabled */
	if ((HWREG(SPIConfig.ui32Base + SPI_O_CR1) & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		HWREG(SPIConfig.ui32Base + SPI_O_CR1) |= SPI_CR1_SPE;
	}

	/* Transmit and Receive data in 8 Bit mode */
	while (ui8Size > 0)
	{
		*((__IO uint8_t *) & HWREG(SPIConfig.ui32Base + SPI_O_DR)) = 0x00; // Transmit dummy byte

		/* Wait until TXE flag is set */
		while (!(HWREG(SPIConfig.ui32Base + SPI_O_SR) & SPI_SR_TXE));

		/* Wait until RXNE flag is set */
		while (!(HWREG(SPIConfig.ui32Base + SPI_O_SR) & SPI_SR_RXNE));

		*pData++ = HWREG(SPIConfig.ui32Base + SPI_O_DR);
		ui8Size--;
	}

	SPIConfig.SPIState = SPI_STATE_READY;

	return 0;
}


/************************************************************************************************************************/
/* Private methods */
inline bool SPI::isValidBase()
{
	/* Only SPI1 and SPI2 are supported by the driver! */
	return ((SPIConfig.ui32Base == SPI1_BASE) || (SPIConfig.ui32Base == SPI2_BASE));
}

