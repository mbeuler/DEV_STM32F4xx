/*
 * stm32f4xx_usart.hpp
 *
 *  Created on: Aug 2, 2022
 *      Author: marcel.beuler
 */

#ifndef STM32F4XX_INCLUDE_STM32F4XX_USART_HPP_
#define STM32F4XX_INCLUDE_STM32F4XX_USART_HPP_

#include "stm32f4xx.h"


/* Supported baud rate settings */
#define   USART_1200	 1200
#define   USART_2400	 2400
#define   USART_4800	 4800
#define   USART_9600     9600
#define  USART_19200    19200
#define  USART_38400    38400
#define  USART_57600    57600
#define USART_115200   115200
#define USART_230400   230400
#define USART_460800   460800
#define USART_921600   921600

/* USART register offsets */
#define USART_O_SR     0x00000000  // Status register
#define USART_O_DR     0x00000004  // Data register
#define USART_O_BRR    0x00000008  // Baud rate register
#define USART_O_CR1    0x0000000C  // Control register 1
#define USART_O_CR2    0x00000010  // Control register 2
#define USART_O_CR3    0x00000014  // Control register 3
#define USART_O_GTPR   0x00000018  // Guard time and prescaler register

/* DMA interrupt register offsets */
#define DMA_O_LISR     0x00000000  // Low interrupt status register  (stream 0...3)
#define DMA_O_HISR     0x00000004  // High interrupt status register (stream 4...7)
#define DMA_O_LIFCR    0x00000008  // Low interrupt flag clear register  (stream 0...3)
#define DMA_O_HIFCR    0x0000000C  // High interrupt flag clear register (stream 4...7)


/* Array used to get the DMA stream register offset versus stream index 0...7
 * This array is identical to STEAM_OFFSET_TAB[] in stm32f4xx_ll_dma.h */
static const uint8_t STREAM_O_TAB[] =
{
  (uint8_t)(DMA1_Stream0_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream1_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream2_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream3_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream4_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream5_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream6_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream7_BASE - DMA1_BASE)
};

/* Macro to access hardware register */
#ifndef HWREG
#define HWREG(x)  (*((volatile uint32_t *)(x)))
#endif

#define ROUND_DIVIDE(x,y)  (((x) + (y) / 2) / (y))

#if defined (STM32F411xE)
#define USART_MAX_INSTANCES   2
#elif defined (STM32F446xx)
#define USART_MAX_INSTANCES   3
#else
#error "Please select first the target device used in your application (via preprocessor)"
#endif

#define USART_TX_BUFFER_SIZE   80  // 64
#define USART_RX_BUFFER_SIZE   80  // 64

typedef struct
{
	bool     boStatusOK;
	uint32_t ui32Base;
	uint32_t ui32BaudRate;
	bool     boDMA;
}USARTConfig_t;

typedef struct
{
	DMA_TypeDef *DMAx;
	uint32_t Stream;   // DMA stream
	uint32_t Channel;  // DMA channel
	uint32_t OffsISR;
	uint32_t HTIF;
	uint32_t TCIF;
	IRQn_Type IRQn;
}DMAConfig_t;

typedef struct
{
	uint8_t  *buffer;
	volatile uint16_t head;
	volatile uint16_t tail;
}CircularBuffer_t;


class USART
{
public:
	/* Constructor, Destructor */
	USART(uint32_t ui32Base, uint32_t ui32BaudRate, bool boDMA);
	~USART();

	/* Methods */
	void transmitData(uint8_t ui8Data);                   // Without interrupt
	void transmitData(uint8_t *pData, uint16_t ui16Size); // Without interrupt
	void waitUntilTransmitComplete();
	int16_t receiveData(uint8_t *pData);                  // Without interrupt

	uint16_t availableForWrite();
	uint16_t availableForRead();

	uint16_t writeBuffer(uint8_t *pData, uint16_t ui16Size);
	uint16_t writeBuffer(char *pData, uint16_t ui16Size);
	uint16_t readBuffer(uint8_t *pData, uint16_t ui16Size);

	void transmitDmaData(uint8_t *pData, uint16_t ui16Size);

	void enableRxInt();
	void disableRxInt();

	USARTConfig_t getUSARTConfig();

	/* Declare all encoder ISRs */
	static void globalISR1a();
	static void globalISR1b();
	static void globalISR1c();
#if defined (STM32F446xx)
	static void globalISR3a();
	static void globalISR3b();
	static void globalISR3c();
#endif
	static void globalISR6a();
	static void globalISR6b();
	static void globalISR6c();


protected:
	/* Methods */
	virtual void processReceivedDmaData(uint8_t *pData, uint16_t ui16Size, bool boEndOfFrame);
	inline virtual void setDmaRxFlag(void);
	inline virtual void clearDmaTxFlag(void);


private:
	/* Variables */
	USARTConfig_t USARTConfig;
	DMAConfig_t   DMAConfigTx, DMAConfigRx;

	uint8_t _TxBuffer[USART_TX_BUFFER_SIZE];
	uint8_t _RxBuffer[USART_RX_BUFFER_SIZE];

	CircularBuffer_t TxBuffer;
	CircularBuffer_t RxBuffer;

	volatile uint16_t count;
	static USART* myInstance[]; // Callback instance for the ISR to reach instanceISR()

	/* Methods */
	inline bool isValidBase();
	inline bool isValidBaudRate();

	inline void enableTxInt();
	inline void disableTxInt();

	void checkReceivedDmaData(bool bEndOfFrame);

	void instanceISRa();
	void instanceISRb();
	void instanceISRc();
};


#endif /* STM32F4XX_INCLUDE_STM32F4XX_USART_HPP_ */
