/*
 * stm32f4xx_usart.cpp
 *
 *  Created on: Aug 2, 2022
 *      Author: marcel.beuler
 */

#include "stm32f4xx_usart.h"


USART* USART::myInstance[USART_MAX_INSTANCES]; // Callback instance handle for instanceISR()

// ISR for each instance
void USART::globalISR1a() {USART::myInstance[0]->instanceISRa();}
void USART::globalISR1b() {USART::myInstance[0]->instanceISRb();}
void USART::globalISR1c() {USART::myInstance[0]->instanceISRc();}
#if defined (STM32F411xE)
void USART::globalISR6a() {USART::myInstance[1]->instanceISRa();}
void USART::globalISR6b() {USART::myInstance[1]->instanceISRb();}
void USART::globalISR6c() {USART::myInstance[1]->instanceISRc();}
#elif defined (STM32F446xx)
void USART::globalISR3a() {USART::myInstance[1]->instanceISRa();}
void USART::globalISR3b() {USART::myInstance[1]->instanceISRb();}
void USART::globalISR3c() {USART::myInstance[1]->instanceISRc();}
void USART::globalISR6a() {USART::myInstance[2]->instanceISRa();}
void USART::globalISR6b() {USART::myInstance[2]->instanceISRb();}
void USART::globalISR6c() {USART::myInstance[2]->instanceISRc();}
#else
    #error "Please select first the target device used in your application (via preprocessor)"
#endif


/* USART class */
/**
  @brief  Constructor initializes USART
  @param  ui32Base (STM32F411xE: USART1_BASE, USART6_BASE
                    STM32F446xx: USART1_BASE, USART3_BASE, USART6_BASE)
  @param  ui32BaudRate (1200, 2400, 4800, 9600, 19200, 38400, 57600,
                        115200, 230400, 460800, 921600)
  @param  boDMA (false: DMA is not used)
  @note   Private variable USARTConfig:
          - USARTConfig.StatusOK is set to true if the initialization was successful
          - USARTConfig.ui32Base is set to parameter ui32Base
          - USARTConfig.ui32BaudRate is set to parameter ui32BaudRate
          - USARTConfig.bDMA is set to parameter bDMA
**/
USART::USART(uint32_t ui32Base, uint32_t ui32BaudRate, bool boDMA=false)
{
	uint32_t ui32APBClock;
	uint8_t  ui8PPRE;

	uint32_t ui32Numerator;
	uint32_t ui32Denominator;
	uint32_t ui32Mantissa;
	uint32_t ui32Fraction;

	uint8_t ui8Index;

	USARTConfig.boStatusOK   = false;
	USARTConfig.ui32Base     = ui32Base;
	USARTConfig.ui32BaudRate = ui32BaudRate;
	USARTConfig.boDMA        = boDMA;


	/* Check base address */
	if (!isValidBase()) {return;}

	/* Check if UE bit in register CR1 is cleared (USART disabled; RM0383 p. 550 f.; RM0390 p. 839) */
	if (HWREG(ui32Base + USART_O_CR1) & USART_CR1_UE) {return;}

	/* Check baud rate */
	if (!isValidBaudRate()) {return;}

	/* Enable clock, alternate function and USART IRQ in NVIC
	 * This depends on the selected USART, therefore HWREG macro cannot be used */
	switch(ui32Base)
	{
	case USART1_BASE:                                   // AF = 7, TX = PA9, RX = PA10 (DS10314 p. 47; DS10693 p. 57)
		RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;           // Enable GPIOA clock
		RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;          // Enable USART1 clock (APB2, RM0383 p. 39; RM0390 p. 58)
		GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;         //
		GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos);   // Alternate function for USART1_TX
		GPIOA->MODER  &= ~GPIO_MODER_MODER9_Msk;        //
		GPIOA->MODER  |= (2 << GPIO_MODER_MODER9_Pos);  // Enable alternate function for PA9
		GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;        //
		GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);  // Alternate function for USART1_RX
		GPIOA->MODER  &= ~GPIO_MODER_MODER10_Msk;       //
		GPIOA->MODER  |= (2 << GPIO_MODER_MODER10_Pos); // Enable alternate function for PA10
		NVIC_EnableIRQ(USART1_IRQn);                    // Enable USART1 IRQ in NVIC (see core_cm4.h)
		break;

#if defined (STM32F446xx)
	case USART3_BASE:                                   // AF = 7, TX = PC10, RX = PC11 (DS10693 p. 59)
		RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;           // Enable GPIOC clock
		RCC->APB1ENR  |= RCC_APB1ENR_USART3EN;          // Enable USART3 clock (APB1, RM0390 p. 59)
		GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;        //
		GPIOC->AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);  // Alternate function for USART3_TX
		GPIOC->MODER  &= ~GPIO_MODER_MODER10_Msk;       //
		GPIOC->MODER  |= (2 << GPIO_MODER_MODER10_Pos); // Enable alternate function for PC10
		GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;        //
		GPIOC->AFR[1] |= (7 << GPIO_AFRH_AFSEL11_Pos);  // Alternate function for USART3_RX
		GPIOC->MODER  &= ~GPIO_MODER_MODER11_Msk;       //
		GPIOC->MODER  |= (2 << GPIO_MODER_MODER11_Pos); // Enable alternate function for PC11
		NVIC_EnableIRQ(USART3_IRQn);                    // Enable USART3 IRQ in NVIC (see core_cm4.h)
		break;
#endif

#if defined (STM32F411xE)
	case USART6_BASE:                                   // AF = 8, TX = PA11, RX = PA12 (DS10314 p. 44)
		RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;           // Enable GPIOA clock
		RCC->APB2ENR  |= RCC_APB2ENR_USART6EN;          // Enable USART6 clock (APB2, RM0383 p. 39)
		GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;        //
		GPIOA->AFR[1] |= (8 << GPIO_AFRH_AFSEL11_Pos);  // Alternate function for USART6_TX
		GPIOA->MODER  &= ~GPIO_MODER_MODER11_Msk;       //
		GPIOA->MODER  |= (2 << GPIO_MODER_MODER11_Pos); // Enable alternate function for PA11
		GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL12_Msk;        //
		GPIOA->AFR[1] |= (8 << GPIO_AFRH_AFSEL12_Pos);  // Alternate function for USART6_RX
		GPIOA->MODER  &= ~GPIO_MODER_MODER12_Msk;       //
		GPIOA->MODER  |= (2 << GPIO_MODER_MODER12_Pos); // Enable alternate function for PA12
		NVIC_EnableIRQ(USART6_IRQn);                    // Enable USART6 IRQ in NVIC (see core_cm4.h)
		break;
#elif defined (STM32F446xx)
	case USART6_BASE:                                   // AF = 8, TX = PC6, RX = PC7 (DS10693 p. 59)
		RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;           // Enable GPIOC clock
		RCC->APB2ENR  |= RCC_APB2ENR_USART6EN;          // Enable USART6 clock (APB2, RM0390 p. 58)
		GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;         //
		GPIOC->AFR[0] |= (8 << GPIO_AFRL_AFSEL6_Pos);   // Alternate function for USART6_TX
		GPIOC->MODER  &= ~GPIO_MODER_MODER6_Msk;        //
		GPIOC->MODER  |= (2 << GPIO_MODER_MODER6_Pos);  // Enable alternate function for PC6
		GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;         //
		GPIOC->AFR[0] |= (8 << GPIO_AFRL_AFSEL7_Pos);   // Alternate function for USART6_RX
		GPIOC->MODER  &= ~GPIO_MODER_MODER7_Msk;        //
		GPIOC->MODER  |= (2 << GPIO_MODER_MODER7_Pos);  // Enable alternate function for PC7
		NVIC_EnableIRQ(USART6_IRQn);                    // Enable USART6 IRQ in NVIC (see core_cm4.h)
		break;
#endif

	default:
		/* We will not get here */
		return;
	}

	/* Get APB prescaler */
	switch(ui32Base)
	{
	case USART1_BASE:
	case USART6_BASE:
		// APB2
		ui8PPRE = (RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;
		break;

#if defined (STM32F446xx)
	case USART3_BASE:
		// APB1
		ui8PPRE = (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
		break;
#endif

	default:
		/* We will not get here */
		return;
	}

	/* Convert PPRE for clock calculation (RM0383 p. 106 f.; RM0390 p. 131 f.) */
	switch(ui8PPRE)
	{
	case 0x04: ui8PPRE = 2;  // AHB clock (core clock) divided by 2
	           break;
	case 0x05: ui8PPRE = 4;  // AHB clock (core clock) divided by 4
	           break;
	case 0x06: ui8PPRE = 8;  // AHB clock (core clock) divided by 8
	           break;
	case 0x07: ui8PPRE = 16; // AHB clock (core clock) divided by 16
	           break;
	default  : ui8PPRE = 1;  // AHB clock (core clock) not divided
	           break;
	}

	/* Update SystemCoreClock variable with the current core clock and calculate APB clock */
	SystemCoreClockUpdate();
	ui32APBClock = SystemCoreClock / ui8PPRE;

	/* Calculate Mantissa and Fraction for baud rate register */
	ui32Numerator   = ui32APBClock;
	ui32Denominator = 16 * ui32BaudRate;
	ui32Mantissa    = ui32Numerator / ui32Denominator;
	ui32Numerator   = (ui32Numerator - ui32Mantissa * ui32Denominator) * 16; // Remainder * 16
	ui32Fraction    = ROUND_DIVIDE(ui32Numerator,ui32Denominator);

	/* Fraction has 4 bits in the baud rate register, therefore only values between 0d and 15d are possible */
	if (ui32Fraction == 16)
	{
		ui32Mantissa++;
		ui32Fraction = 0;
	}

	/* Set baud rate register, BRR (RM0383 p. 550; RM0390 p. 838) */
	HWREG(ui32Base + USART_O_BRR) &= ~(USART_BRR_DIV_Mantissa_Msk << USART_BRR_DIV_Mantissa_Pos);
	HWREG(ui32Base + USART_O_BRR) |= (ui32Mantissa << USART_BRR_DIV_Mantissa_Pos);
	HWREG(ui32Base + USART_O_BRR) &= ~(USART_BRR_DIV_Fraction_Msk << USART_BRR_DIV_Fraction_Pos);
	HWREG(ui32Base + USART_O_BRR) |= (ui32Fraction << USART_BRR_DIV_Fraction_Pos);

	/* Configure control register 1, CR1 (RM0383 p. 550 ff.; RM0390 p. 839 f.)
	 * - Clear OVER8 bit => Oversampling by 16 (oversampling by 8 not supported here)
	 * - Clear M bit     => 1 start bit, 8 data bits, n stop bit (the latter is configured in register CR2)
	 * - Clear PCE bit   => Parity is disabled
	 * - Set TE bit      => Transmitter is enabled
	 * - Set RE bit      => Receiver is enabled */
	HWREG(ui32Base + USART_O_CR1)  = 0x00000000; // Reset value => OVER8, M and PCE are cleared
	HWREG(ui32Base + USART_O_CR1) |= (USART_CR1_TE | USART_CR1_RE);

	/* Configure control register 2, CR2 (RM0383 p. 553 f.; RM0390 p. 841 f.)
	 * - Clear STOP bits => 1 stop bit */
	HWREG(ui32Base + USART_O_CR2) = 0x00000000; // Reset value => STOP is cleared

	/* Configure control register 3, CR3 (RM0383 p. 554 f.; RM0390 p. 842 ff.)
	 * - Clear CTSE bit  => CTS hardware flow control disabled
	 * - Clear RTSE bit  => RTS hardware flow control disabled */
	HWREG(ui32Base + USART_O_CR3) = 0x00000000; // Reset value => CTSE and RTSE are cleared

	/* Initialization of TxBuffer and RxBuffer */
	TxBuffer.buffer = _TxBuffer;
	TxBuffer.head   = 0;
	TxBuffer.tail   = 0;

	RxBuffer.buffer = _RxBuffer;
	RxBuffer.head   = 0;
	RxBuffer.tail   = 0;

	/* Interrupt handling */
	count = 0;  // Private variable is changed in instanceISR

	/* Mapping between base address and index of myInstance */
	switch(ui32Base)
	{
	case USART1_BASE: ui8Index = 0;
	                  break;
#if defined (STM32F411xE)
	case USART6_BASE: ui8Index = 1;
	                  break;
#elif defined (STM32F446xx)
	case USART3_BASE: ui8Index = 1;
	                  break;
	case USART6_BASE: ui8Index = 2;
	                  break;
#else
#error "Please select first the target device used in your application (via preprocessor)"
#endif
	default         : /* We will not get here */
		              return;
	}

//	myInstance[ui8Index] = nullptr;
	myInstance[ui8Index] = this;

	/* DMA initialization (if used) */
	if (boDMA)
	{
		switch(ui32Base)
		{
		case USART1_BASE:
			/* USART1_TX: DMA2, Stream 7, Channel 4
			 * USART1_RX: DMA2, Stream 5, Channel 4 (RM0383 p. 170; RM0390 p. 207) */
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable DMA2 controller clock

			DMAConfigTx.DMAx    = (DMA_TypeDef *)DMA2_BASE;  // Code line is equal to "DMA2"
			DMAConfigTx.Stream  = 7;                         // Code line is equal to "LL_DMA_STREAM_7"
			DMAConfigTx.Channel = 4;                         // Set channel 4 => USART1_TX
			DMAConfigTx.OffsISR = DMA_O_HISR;                // Stream 0...3 --> LISR; Stream 4...7 --> HISR
			DMAConfigTx.HTIF    = DMA_HISR_HTIF7;            // Stream 7 half transfer interrupt flag
			DMAConfigTx.TCIF    = DMA_HISR_TCIF7;            // Stream 7 transfer complete interrupt flag
			DMAConfigTx.IRQn    = DMA2_Stream7_IRQn;

			DMAConfigRx.DMAx    = (DMA_TypeDef *)DMA2_BASE;  // Code line is equal to "DMA2"
			DMAConfigRx.Stream  = 5;                         // Code line is equal to "LL_DMA_STREAM_5"
			DMAConfigRx.Channel = 4;                         // Set channel 4 => USART1_RX
			DMAConfigRx.OffsISR = DMA_O_HISR;                // Stream 0...3 --> LISR; Stream 4...7 --> HISR
			DMAConfigRx.HTIF    = DMA_HISR_HTIF5;            // Stream 5 half transfer interrupt flag
			DMAConfigRx.TCIF    = DMA_HISR_TCIF5;            // Stream 5 transfer complete interrupt flag
			DMAConfigRx.IRQn    = DMA2_Stream5_IRQn;
			break;

#if defined (STM32F446xx)
		case USART3_BASE:
			/* USART3_TX: DMA1, Stream 3, Channel 4
			 * USART3_RX: DMA1, Stream 1, Channel 4 (RM0390 p. 207) */
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // Enable DMA1 controller clock

			DMAConfigTx.DMAx    = (DMA_TypeDef *)DMA1_BASE;  // Code line is equal to "DMA1"
			DMAConfigTx.Stream  = 3;                         // Code line is equal to "LL_DMA_STREAM_3"
			DMAConfigTx.Channel = 4;                         // Set channel 4 => USART3_TX
			DMAConfigTx.OffsISR = DMA_O_LISR;                // Stream 0...3 --> LISR; Stream 4...7 --> HISR
			DMAConfigTx.HTIF    = DMA_LISR_HTIF3;            // Stream 3 half transfer interrupt flag
			DMAConfigTx.TCIF    = DMA_LISR_TCIF3;            // Stream 3 transfer complete interrupt flag
			DMAConfigTx.IRQn    = DMA1_Stream3_IRQn;

			DMAConfigRx.DMAx    = (DMA_TypeDef *)DMA1_BASE;  // Code line is equal to "DMA1"
			DMAConfigRx.Stream  = 1;                         // Code line is equal to "LL_DMA_STREAM_1"
			DMAConfigRx.Channel = 4;                         // Set channel 4 => USART3_RX
			DMAConfigRx.OffsISR = DMA_O_LISR;                // Stream 0...3 --> LISR; Stream 4...7 --> HISR
			DMAConfigRx.HTIF    = DMA_LISR_HTIF1;            // Stream 1 half transfer interrupt flag
			DMAConfigRx.TCIF    = DMA_LISR_TCIF1;            // Stream 1 transfer complete interrupt flag
			DMAConfigRx.IRQn    = DMA1_Stream1_IRQn;
			break;
#endif

		case USART6_BASE:
			/* USART6_TX: DMA2, Stream 6, Channel 5
			 * USART6_RX: DMA2, Stream 1, Channel 5 (RM0383 p. 170; RM0390 p. 207) */
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable DMA2 controller clock

			DMAConfigTx.DMAx    = (DMA_TypeDef *)DMA1_BASE;  // Code line is equal to "DMA1"
			DMAConfigTx.Stream  = 6;                         // Code line is equal to "LL_DMA_STREAM_6"
			DMAConfigTx.Channel = 5;                         // Set channel 5 => USART6_TX
			DMAConfigTx.OffsISR = DMA_O_HISR;                // Stream 0...3 --> LISR; Stream 4...7 --> HISR
			DMAConfigTx.HTIF    = DMA_HISR_HTIF6;            // Stream 6 half transfer interrupt flag
			DMAConfigTx.TCIF    = DMA_HISR_TCIF6;            // Stream 6 transfer complete interrupt flag
			DMAConfigTx.IRQn    = DMA2_Stream6_IRQn;

			DMAConfigRx.DMAx    = (DMA_TypeDef *)DMA1_BASE;  // Code line is equal to "DMA1"
			DMAConfigRx.Stream  = 1;                         // Code line is equal to "LL_DMA_STREAM_1"
			DMAConfigRx.Channel = 5;                         // Set channel 5 => USART6_RX
			DMAConfigRx.OffsISR = DMA_O_LISR;                // Stream 0...3 --> LISR; Stream 4...7 --> HISR
			DMAConfigRx.HTIF    = DMA_LISR_HTIF1;            // Stream 1 half transfer interrupt flag
			DMAConfigRx.TCIF    = DMA_LISR_TCIF1;            // Stream 1 transfer complete interrupt flag
			DMAConfigRx.IRQn    = DMA2_Stream1_IRQn;
			break;

		default:
			/* We will not get here */
			return;
		}

		/* DMA transmitter initialization
		 * Disable selected DMA stream and wait until selected DMA stream is disabled */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_EN;
		while (((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR & DMA_SxCR_EN){};

		/* Set channel */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_CHSEL_Msk;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR |= (DMAConfigTx.Channel << DMA_SxCR_CHSEL_Pos);

		/* Set data transfer direction to "memory-to-peripheral; Set priority level to "low" */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_DIR_Msk;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR |= DMA_SxCR_DIR_0;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_PL_Msk;

		/* Disable circular mode; Peripheral address pointer is fixed; Memory address pointer is incremented */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_CIRC;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_PINC;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR |= DMA_SxCR_MINC;

		/* Set peripheral data size to "byte"; Set memory data size to "byte" */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_PSIZE_Msk;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_MSIZE_Msk;

		/* Direct mode is enabled (no FIFO) */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->FCR &= ~DMA_SxFCR_DMDIS;

		/* Set peripheral address (USARTx DR register) */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->PAR = USARTConfig.ui32Base + USART_O_DR;

		/* DMA interrupt enable in NVIC; USART transmit complete interrupt enable */
		NVIC_EnableIRQ(DMAConfigTx.IRQn);
		HWREG(ui32Base + USART_O_CR1) |= USART_CR1_TCIE;


		/* DMA receiver initialization
		 * Disable selected DMA stream and wait until selected DMA stream is disabled */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR &= ~DMA_SxCR_EN;
		while (((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR & DMA_SxCR_EN){};

		/* Set channel */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR &= ~DMA_SxCR_CHSEL_Msk;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR |= (DMAConfigRx.Channel << DMA_SxCR_CHSEL_Pos);

		/* Set data transfer direction to "peripheral-to-memory"; Set priority level to "low" */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR &= ~DMA_SxCR_DIR_Msk;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR &= ~DMA_SxCR_PL_Msk;

		/* Enable circular mode; Peripheral address pointer is fixed; Memory address pointer is incremented */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR |= DMA_SxCR_CIRC;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR &= ~DMA_SxCR_PINC;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR |= DMA_SxCR_MINC;

		/* Set peripheral data size to "byte"; Set memory data size to "byte" */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR &= ~DMA_SxCR_PSIZE_Msk;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR &= ~DMA_SxCR_MSIZE_Msk;

		/* Direct mode is enabled (no FIFO) */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->FCR &= ~DMA_SxFCR_DMDIS;

		/* Set peripheral address (USARTx DR register); Set memory base address; Set number of data items to transfer */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->PAR = ui32Base + USART_O_DR;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->M0AR = (uint32_t)_RxBuffer;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->NDTR = USART_RX_BUFFER_SIZE;

		/* Enable HT and TC interrupt */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR |= DMA_SxCR_HTIE;
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR |= DMA_SxCR_TCIE;

		/* DMA interrupt enable in NVIC */
		NVIC_EnableIRQ(DMAConfigRx.IRQn);

		/* USART DMA mode is enabled for reception; USART IDLE interrupt enable */
		HWREG(ui32Base + USART_O_CR3) |= USART_CR3_DMAR;
		HWREG(ui32Base + USART_O_CR1) |= USART_CR1_IDLEIE;

		/* Enable selected DMA stream */
		((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR |= DMA_SxCR_EN;
	}

	/* Set UE bit in register CR1 to enable UART */
	HWREG(ui32Base + USART_O_CR1) |= USART_CR1_UE;

	USARTConfig.boStatusOK = true;

	return;
}


/**
  @brief  Destructor
  @note  To be continued...
**/
USART::~USART()
{
	disableTxInt();
	disableRxInt();
}


/* Public methods */
/**
  @brief  Writes a single data byte to the TDR register. Don't use this function together with writeBuffer()!
  @param  ui8Data
  @retval None
**/
void USART::transmitData(uint8_t ui8Data)
{
	/* The TXE flag (transmit empty bit) is set to 1 as soon as the DR register is ready to accept another character */
	while (!(HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_TXE)); // Wait until the TXE flag of the SR register is set to 1
	HWREG(USARTConfig.ui32Base + USART_O_DR) = ui8Data;

	/* The data register performs a double function (read and write) since it is composed of two registers, one for
	 * transmission (TDR) and one for reception (RDR) (RM0383 p. 550; RM0390 p. 838).
	 * The byte has been written into the register TDR, but it is still present in the shift register until all bits
	 * are transmitted! */
}


/**
  @brief  Writes data bytes to the TDR register. Don't use this function together with writeBuffer()!
  @param  *pData  : Pointer to data array
  @param  ui16Size: Number of bytes to transmit
  @retval None
**/
void USART::transmitData(uint8_t *pData, uint16_t ui16Size)
{
	for (uint16_t i = 0; i < ui16Size; i++)
	{
		/* If a previous character was written into the TDR register (TXE flag automatically cleared in this case), the
		 * TXE flag (transmit empty bit) is set to 1 as soon as the character is moved to the transmit shift register and
		 * indicates that the TDR register is ready to accept another character. */
		while (!(HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_TXE)); // Wait until the TXE flag of the SR register is set to 1
		HWREG(USARTConfig.ui32Base + USART_O_DR) = *pData;
		pData++;
	}

	/* The last byte has been written into the TDR register, but it is still present in the shift register until all bits
	 * are transmitted! */
}


/**
  @brief  Wait until the Transmission Complete bit is set in the USART_SR register
  @param  None
  @retval None
**/
void USART::waitUntilTransmitComplete()
{
	/* Wait until the Transmission Complete bit is set */
	while(!(HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_TC));

	/* The last data written into the TDR register has been transmitted out of the shift register
	 * --> Transmission complete (RM0383 p. 547 f.; RM0390 p. 835 f.) */
}


/**
  @brief  Reads a single data byte from the RDR register. Use this function only in case of disabled Rx interrupt!
  @param  *pData: Pointer to store the data byte
  @retval -1 if RXNEIE bit in USART_CR1 register is set or RDR register is empty (no byte was received), otherwise 0
**/
int16_t USART::receiveData(uint8_t *pData)
{
	uint32_t ui32Aux;

	/* Check if RXNEIE bit in USART_CR1 register is set (RM0383 p. 550 ff.; RM0390 p. 839 f.) */
	if (HWREG(USARTConfig.ui32Base + USART_O_CR1) & USART_CR1_RXNEIE) {return -1;}

	/* RXNE bit in USART_SR register is set by hardware when the content of the RDR shift register has been transferred to the
	 * USART_DR register, and cleared by reading the USART_DR register (RM0383 p. 547 f.; RM0390 p. 835 f.) */
	if (HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_ORE)
	{
		/* An Overrun error (ORE) occurred. The ORE bit is cleared by a software sequence (a read to the USART_SR register
		 * followed by a read to the USART_DR register (RM0383 p. 547 f.; RM0390 p. 835 ff.) */
		ui32Aux = HWREG(USARTConfig.ui32Base + USART_O_SR);
		ui32Aux = HWREG(USARTConfig.ui32Base + USART_O_DR);
		(void) ui32Aux; // Work around compiler warning
		return -1;
	}
	else if (HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_RXNE)
	{
		*pData = HWREG(USARTConfig.ui32Base + USART_O_DR);
		return 0;
	}

	return -1; /* No byte was received */
}


/**
  @brief  Determine the number of bytes that can be written into TxBuffer
  @param  None
  @retval Number of empty bytes (0 to USART_TX_BUFFER_SIZE - 1)
**/
uint16_t USART::availableForWrite()
{
	return (uint16_t)(USART_TX_BUFFER_SIZE + TxBuffer.tail - TxBuffer.head - 1) % USART_TX_BUFFER_SIZE;
}


/**
  @brief  Determine the number of bytes that can be read from RxBuffer
  @param  None
  @retval Number of stored bytes (0 to USART_RX_BUFFER_SIZE - 1)
**/
uint16_t USART::availableForRead()
{
	return (uint16_t)(USART_RX_BUFFER_SIZE + RxBuffer.head - RxBuffer.tail) % USART_RX_BUFFER_SIZE;
}


/**
  @brief  Writes data bytes into TxBuffer if free memory space is available
  @param  *pData: Pointer to a data array
          ui16Size: Number of bytes to write
  @retval Number of bytes that could be written (0 to USART_TX_BUFFER - 1)
**/
uint16_t USART::writeBuffer(uint8_t *pData, uint16_t ui16Size)
{
	uint16_t ui16ByteCounter = 0;
	uint16_t ui16Aux;


	while(ui16ByteCounter < ui16Size)
	{
		/* Check if data can be stored in TxBuffer */
		ui16Aux = (uint16_t)(TxBuffer.head + 1) % USART_TX_BUFFER_SIZE;
		if (ui16Aux == TxBuffer.tail)
		{
			/* USART_TX_BUFFER_SIZE - 1 bytes are already stored, do not overwrite data! */
			return ui16ByteCounter;
		}
		else
		{
			/* Write byte into TxBuffer */
			TxBuffer.buffer[TxBuffer.head] = *pData;
			TxBuffer.head = ui16Aux;
			pData++;
			ui16ByteCounter++;
		}
	}

	if (ui16ByteCounter && !(HWREG(USARTConfig.ui32Base + USART_O_CR1) & USART_CR1_TXEIE))
	{
		/* Write the first byte into the TDR register, AFTER THAT enable Tx interrupt */
		HWREG(USARTConfig.ui32Base + USART_O_DR) = TxBuffer.buffer[TxBuffer.tail];  // tail is updated in ISR
		enableTxInt();
	}

	return ui16ByteCounter;
}


/**
  @brief  Writes data bytes into TxBuffer if free memory space is available
  @param  *pData: Pointer to a data array
          ui16Size: Number of bytes to write
  @retval Number of bytes that could be written (0 to USART_TX_BUFFER - 1)
**/
uint16_t USART::writeBuffer(char *pData, uint16_t ui16Size)
{
	uint16_t ui16ByteCounter = 0;
	uint16_t ui16Aux;

	while(ui16ByteCounter < ui16Size)
	{
		/* Check if data can be stored in TxBuffer */
		ui16Aux = (uint16_t)(TxBuffer.head + 1) % USART_TX_BUFFER_SIZE;
		if (ui16Aux == TxBuffer.tail)
		{
			/* USART_TX_BUFFER_SIZE - 1 bytes are already stored, do not overwrite data! */
			return ui16ByteCounter;
		}
		else
		{
			/* Write byte into TxBuffer */
			TxBuffer.buffer[TxBuffer.head] = (uint8_t)(*pData);
			TxBuffer.head = ui16Aux;
			pData++;
			ui16ByteCounter++;
		}
	}

	if (ui16ByteCounter && !(HWREG(USARTConfig.ui32Base + USART_O_CR1) & USART_CR1_TXEIE))
	{
		/* Write the first byte into the TDR register, AFTER THAT enable Tx interrupt */
		HWREG(USARTConfig.ui32Base + USART_O_DR) = TxBuffer.buffer[TxBuffer.tail];  // tail is updated in ISR
		enableTxInt();
	}

	return ui16ByteCounter;
}


/**
  @brief  Reads data bytes from RxBuffer if data is available
  @param  *pData: Pointer to a data array
          ui16Size: Number of bytes to read
  @retval Number of bytes that could be read (0 to USART_RX_BUFFER - 1)
**/
uint16_t USART::readBuffer(uint8_t *pData, uint16_t ui16Size)
{
	uint16_t ui16ByteCounter = 0;


	while(ui16ByteCounter < ui16Size)
	{
		if (RxBuffer.head == RxBuffer.tail)
		{
			/* No (more) data available in RxBuffer */
			return ui16ByteCounter;
		}
		else
		{
			/* Read byte from RxBuffer */
			*pData = RxBuffer.buffer[RxBuffer.tail];
			RxBuffer.tail = (uint16_t)(RxBuffer.tail + 1) % USART_RX_BUFFER_SIZE;
			pData++;
			ui16ByteCounter++;
		}
	}

	return ui16ByteCounter;
}


/**
  @brief  xx
  @param  None
  @retval None
**/
void USART::transmitDmaData(uint8_t *pData, uint16_t ui16Size)
{
	if (!USARTConfig.boDMA) {return;}

	/* Disable selected DMA stream and wait until selected DMA stream is disabled */
	((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_EN;
	while (((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR & DMA_SxCR_EN){};

	/* Set memory base address; Set number of data items to transfer */
	((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->M0AR = (uint32_t)pData;
	((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->NDTR = ui16Size;

	/* Enable HT and TC interrupt */
	((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR |= DMA_SxCR_HTIE;
	((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR |= DMA_SxCR_TCIE;

	/* Enable USART DMA mode for transmission */
	HWREG(USARTConfig.ui32Base + USART_O_CR3) |= USART_CR3_DMAT;

	/* Enable selected DMA stream */
	((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR |= DMA_SxCR_EN;
}


/**
  @brief  Enable USART Rx interrupt
  @param  None
  @retval None
**/
void USART::enableRxInt()
{
	/* Set RXNEIE bit in control register 1 (CR1; RM0383 p. 550 ff.; RM0390 p. 839 f.) */
	HWREG(USARTConfig.ui32Base + USART_O_CR1) |= USART_CR1_RXNEIE;
}


/**
  @brief  Disable USART Rx interrupt
  @param  None
  @retval None
**/
void USART::disableRxInt()
{
	/* Clear RXNEIE bit in control register 1 (CR1; RM0383 p. 550 ff.; RM0390 p. 839 f.) */
	HWREG(USARTConfig.ui32Base + USART_O_CR1) &= ~USART_CR1_RXNEIE;
}


/**
  @brief  Get private variable USARTConfig
  @param  None
  @retval USARTConfig
**/
USARTConfig_t USART::getUSARTConfig()
{
	return USARTConfig;
}


/************************************************************************************************************************/
/* Private methods */
/**
  @brief  Check private variable USARTConfig.ui32Base (USART base address)
          (STM32F411xE: USART1_BASE, USART6_BASE
           STM32F446xx: USART1_BASE, USART3_BASE, USART6_BASE)
          Other base addresses are not supported
  @param  None
  @retval true if the base address is valid and false otherwise
**/
inline bool USART::isValidBase()
{
#if defined (STM32F411xE)
	return ((USARTConfig.ui32Base == USART1_BASE) || (USARTConfig.ui32Base == USART6_BASE));
#elif defined (STM32F446xx)
	return ((USARTConfig.ui32Base == USART1_BASE) || (USARTConfig.ui32Base == USART3_BASE) ||
			(USARTConfig.ui32Base == USART6_BASE));
#else
    #error "Please select first the target device used in your application (via preprocessor)"
#endif
}


/**
  @brief  Check private variable USARTConfig.ui32BaudRate
          (1200, 2400, 4800, 9600, 19200, 38400, 57600,
           115200, 230400, 460800, 921600)
          Other baud rates are not supported
  @param  None
  @retval true if the baud rate is valid and false otherwise
**/
inline bool USART::isValidBaudRate()
{
	return ((USARTConfig.ui32BaudRate ==   USART_1200) || (USARTConfig.ui32BaudRate ==   USART_2400) ||
			(USARTConfig.ui32BaudRate ==   USART_4800) || (USARTConfig.ui32BaudRate ==   USART_9600) ||
			(USARTConfig.ui32BaudRate ==  USART_19200) || (USARTConfig.ui32BaudRate ==  USART_38400) ||
			(USARTConfig.ui32BaudRate ==  USART_57600) || (USARTConfig.ui32BaudRate == USART_115200) ||
			(USARTConfig.ui32BaudRate == USART_230400) || (USARTConfig.ui32BaudRate == USART_460800) ||
			(USARTConfig.ui32BaudRate == USART_921600));
}


/**
  @brief  Enable USART Tx interrupt
  @param  None
  @retval None
**/
inline void USART::enableTxInt()
{
	/* Set TXEIE bit in control register 1 (CR1; RM0383 p. 550 f.; RM0390 p. 839 f.) */
	HWREG(USARTConfig.ui32Base + USART_O_CR1) |= USART_CR1_TXEIE;
}


/**
  @brief  Disable USART Tx interrupt
  @param  None
  @retval None
**/
inline void USART::disableTxInt()
{
	/* Clear TXEIE bit in control register 1 (CR1; RM0383 p. 550 f.; RM0390 p. 839 f.) */
	HWREG(USARTConfig.ui32Base + USART_O_CR1) &= ~USART_CR1_TXEIE;
}


/**
  @brief  xx
  @param  None
  @retval None
**/
void USART::checkReceivedDmaData(bool bEndOfFrame)
{
	RxBuffer.head = USART_RX_BUFFER_SIZE - (((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->NDTR & DMA_SxNDT);

	if (RxBuffer.head != RxBuffer.tail)
	{
		if (RxBuffer.head > RxBuffer.tail)
		{
			/* Processing is done in "linear" mode
			 *
			 * [      0      ]  <- RxBuffer.tail
			 * [      1      ]
			 * [      2      ]
			 * [      3      ]
			 * [      4      ]
			 * [      5      ]
			 * [      6      ]  <- RxBuffer.head
			 * [      7      ]
			 * ...
			 * [BUFFER_SIZE-1]
			 *
			 */

			processReceivedDmaData(&RxBuffer.buffer[RxBuffer.tail], RxBuffer.head - RxBuffer.tail, bEndOfFrame);
		}
		else
		{
			/* Processing is done in "overflow" mode
			 * Application must process data twice, since there are 2 linear memory blocks to handle
			 *
			 * [      0      ]                    |----------------------------------------|
			 * [      1      ]                    | Second block (length = RxBuffer.head)  |
			 * [      2      ]                    |----------------------------------------|
			 * [      3      ]  <- RxBuffer.head
			 * [      4      ]  <- RxBuffer.tail  |----------------------------------------|
			 * [      5      ]                    |                                        |
			 * [      6      ]                    | First block                            |
			 * [      7      ]                    | (length = BUFFER_SIZE - RxBuffer.tail) |
			 * ...                                |                                        |
			 * [BUFFER_SIZE-1]                    |----------------------------------------|
			 *
			 */

			processReceivedDmaData(&RxBuffer.buffer[RxBuffer.tail], USART_RX_BUFFER_SIZE - RxBuffer.tail, bEndOfFrame);
			if (RxBuffer.head > 0)
			{
				processReceivedDmaData(&RxBuffer.buffer[0], RxBuffer.head, bEndOfFrame);
			}
		}
		RxBuffer.tail = RxBuffer.head;
	}
	else
	{
		/* New data wasn't received, check end of frame */
		if (bEndOfFrame)
		{
			setDmaRxFlag();
		}
	}
}


/**
  @brief  xx
  @param  None
  @retval None
**/
void USART::processReceivedDmaData(uint8_t *pData, uint16_t ui16Size, bool bEndOfFrame)
{
	/* This function must be overridden in the derived class! */
	uint32_t ui32Debug;

	ui32Debug = 0;
	(void) ui32Debug; // Work around compiler warning
}


/**
  @brief  xx
  @param  None
  @retval None
**/
inline void USART::setDmaRxFlag(void)
{
	/* This function must be overridden in the derived class! */
	uint32_t ui32Debug;

	ui32Debug = 0;
	(void) ui32Debug; // Work around compiler warning
}


/**
  @brief  xx
  @param  None
  @retval None
**/
inline void USART::clearDmaTxFlag(void)
{
	/* This function must be overridden in the derived class! */
	uint32_t ui32Debug;

	ui32Debug = 0;
	(void) ui32Debug; // Work around compiler warning
}


void USART::instanceISRa()
{
	uint8_t ui8Data;
	uint16_t ui16Aux;
	uint32_t ui32Aux;


	/* Check if RXNE interrupt is enabled */
	if ((HWREG(USARTConfig.ui32Base + USART_O_CR1) & USART_CR1_RXNEIE) == USART_CR1_RXNEIE)
	{
		/* Check if RXNE flag set in USART_SR register (RM0383 p. 547 f.; RM0390 p. 835 f.) */
		if (HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_RXNE)
		{
			ui8Data = HWREG(USARTConfig.ui32Base + USART_O_DR);  // Read access clears RXNE flag

			count++; // Test variable

			/* Check if ui8Data can be stored in RxBuffer */
			ui16Aux = (uint16_t)(RxBuffer.head + 1) % USART_RX_BUFFER_SIZE;
			if (ui16Aux == RxBuffer.tail)
			{
				/* USART_RX_BUFFER_SIZE-1 bytes are already stored, ui8Data is lost! */
			}
			else
			{
				/* Write ui8Data into RxBuffer */
				RxBuffer.buffer[RxBuffer.head] = ui8Data;
				RxBuffer.head = ui16Aux;
			}
		}
	}

	/* Check if TXE interrupt is enabled */
	if ((HWREG(USARTConfig.ui32Base + USART_O_CR1) & USART_CR1_TXEIE) == USART_CR1_TXEIE)
	{
		/* Check if TXE flag is set in USART_SR register (RM0383 p. 547 f.; RM0390 p. 835 f.) */
		if (HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_TXE)
		{
			TxBuffer.tail = (uint16_t)(TxBuffer.tail + 1) % USART_TX_BUFFER_SIZE;
			if (TxBuffer.head != TxBuffer.tail)
			{
				HWREG(USARTConfig.ui32Base + USART_O_DR) = TxBuffer.buffer[TxBuffer.tail];
			}
			else
			{
				/* No more data to transmit */
				disableTxInt();
			}
		}
	}

	/* Check if IDLE line interrupt is enabled */
	if ((HWREG(USARTConfig.ui32Base + USART_O_CR1) & USART_CR1_IDLEIE) == USART_CR1_IDLEIE)
	{
		/* Check if IDLE flag is set in USART_SR register (RM0383 p. 547 f.; RM0390 p. 835 f.) */
		if ((HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_IDLE) == USART_SR_IDLE)
		{
			/* Clear IDLE flag by reading USART_SR register followed by USART_DR register */
			ui32Aux = HWREG(USARTConfig.ui32Base + USART_O_SR);
			//ui32Aux = ((USART_TypeDef*)USARTConfig.ui32Base)->SR;  // Also works
			(void)ui32Aux;
			ui32Aux = HWREG(USARTConfig.ui32Base + USART_O_DR);
			//ui32Aux = ((USART_TypeDef*)USARTConfig.ui32Base)->DR;  // Also works
			(void)ui32Aux;

			/* Check for data to process */
			checkReceivedDmaData(true);
		}
	}

	/* Check if TC interrupt is enabled */
	if ((HWREG(USARTConfig.ui32Base + USART_O_CR1) & USART_CR1_TCIE) == USART_CR1_TCIE)
	{
		/* Check if TC flag is set in USART_SR register (RM0383 p. 547 f.; RM0390 p. 835 f.) */
		if ((HWREG(USARTConfig.ui32Base + USART_O_SR) & USART_SR_TC) == USART_SR_TC)
		{
			/* Clear TC flag by writing a '0' to the corresponding bit in USART_SR register */
			HWREG(USARTConfig.ui32Base + USART_O_SR) &= ~USART_SR_TC;

			// Something to do?
		}
	}

//	(void) ui32Aux; // Work around compiler warning
}


void USART::instanceISRb()
{
	if (!USARTConfig.boDMA) {return;}

	/* Check half transfer complete interrupt */
	if ((((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR & DMA_SxCR_HTIE) == DMA_SxCR_HTIE)
	{
		if ((HWREG((uint32_t)DMAConfigTx.DMAx + DMAConfigTx.OffsISR) & DMAConfigTx.HTIF) == DMAConfigTx.HTIF)
		{
			/* Clear interrupt flag */
			HWREG((uint32_t)DMAConfigTx.DMAx + DMAConfigTx.OffsISR + 0x00000008) |= DMAConfigTx.HTIF;

			// Something to do?
		}
	}

	/* Check transfer complete interrupt */
	if ((((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR & DMA_SxCR_TCIE) == DMA_SxCR_TCIE)
	{
		if ((HWREG((uint32_t)DMAConfigTx.DMAx + DMAConfigTx.OffsISR) & DMAConfigTx.TCIF) == DMAConfigTx.TCIF)
		{
			/* Clear interrupt flag */
			HWREG((uint32_t)DMAConfigTx.DMAx + DMAConfigTx.OffsISR + 0x00000008) |= DMAConfigTx.TCIF;

			/* Disable selected DMA stream */
			((DMA_Stream_TypeDef*)((uint32_t)DMAConfigTx.DMAx + STREAM_O_TAB[DMAConfigTx.Stream]))->CR &= ~DMA_SxCR_EN;

			clearDmaTxFlag();
		}
	}
}


void USART::instanceISRc()
{
	if (!USARTConfig.boDMA) {return;}

	/* Check half transfer complete interrupt */
	if ((((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR & DMA_SxCR_HTIE) == DMA_SxCR_HTIE)
	{
		if ((HWREG((uint32_t)DMAConfigRx.DMAx + DMAConfigRx.OffsISR) & DMAConfigRx.HTIF) == DMAConfigRx.HTIF)
		{
			/* Clear interrupt flag */
			HWREG((uint32_t)DMAConfigRx.DMAx + DMAConfigRx.OffsISR + 0x00000008) |= DMAConfigRx.HTIF;

			/* Check for data to process */
			checkReceivedDmaData(false);
		}
	}

	/* Check transfer complete interrupt */
	if ((((DMA_Stream_TypeDef*)((uint32_t)DMAConfigRx.DMAx + STREAM_O_TAB[DMAConfigRx.Stream]))->CR & DMA_SxCR_TCIE) == DMA_SxCR_TCIE)
	{
		if ((HWREG((uint32_t)DMAConfigRx.DMAx + DMAConfigRx.OffsISR) & DMAConfigRx.TCIF) == DMAConfigRx.TCIF)
		{
			/* Clear interrupt flag */
			HWREG((uint32_t)DMAConfigRx.DMAx + DMAConfigRx.OffsISR + 0x00000008) |= DMAConfigRx.TCIF;

			/* Check for data to process */
			checkReceivedDmaData(false);
		}
	}

}


