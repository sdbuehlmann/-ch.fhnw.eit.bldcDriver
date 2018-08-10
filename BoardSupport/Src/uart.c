/*
 * uart.c
 *
 *  Created on: Aug 5, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAdapter.h"
#include "platformAPI.h"
#include "platformModules.h"

// =============== Defines ============================================================================================================
#define TX_BUFFERSIZE 							1000
#define RX_BUFFERSIZE 							20
#define HALF_RX_BUFFERSIZE 						RX_BUFFERSIZE/2
#define NR_OF_RX_BYTES_TO_PROCEED_PER_CYCLE 	10

// =============== Typdefs =============================================================================================================
typedef enum{
	bufferHalf_upper, bufferHalf_lower
}BufferHalf;

// =============== Variables ===========================================================================================================
uint8_t pTXBufferOne[TX_BUFFERSIZE] = { 0 };
uint8_t pTXBufferTwo[TX_BUFFERSIZE] = { 0 };

uint8_t *pNotBusyTXBuffer;
uint32_t notBusyTXBufferCnt = 0;

Boolean txIsBusy = false;

uint8_t pRXBuffer[RX_BUFFERSIZE] = { 0 };
uint32_t firstNotHandledRXByte = 0;
BufferHalf activeBufferHalf = bufferHalf_upper;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================
static void switchTXBuffers();

// =============== Functions ===========================================================================================================
static void switchTXBuffers() {
	if (pTXBufferOne == pNotBusyTXBuffer) {
		pNotBusyTXBuffer = pTXBufferTwo;
	} else {
		pNotBusyTXBuffer = pTXBufferOne;
	}

	notBusyTXBufferCnt = 0;
}

// --------------- platformModules.h -------------------------------------------------------------------------------------------------------
void initUART() {
	pNotBusyTXBuffer = pTXBufferOne;

	HAL_UART_Receive_IT(pUART_handle, pRXBuffer, HALF_RX_BUFFERSIZE);
}

void proceedUART() {
	wait_ms(200); /* ToDo: entfernen, nur temporär. */

	// handle UART
	uint8_t pReceivedBuffer[NR_OF_RX_BYTES_TO_PROCEED_PER_CYCLE];

	uint8_t cnt = 0;
	for (; cnt < NR_OF_RX_BYTES_TO_PROCEED_PER_CYCLE; cnt++) {
		if (pRXBuffer[firstNotHandledRXByte] != 0) {
			// new received byte
			pReceivedBuffer[cnt] = pRXBuffer[firstNotHandledRXByte];
			pRXBuffer[firstNotHandledRXByte] = 0;

			firstNotHandledRXByte = (firstNotHandledRXByte + 1) % RX_BUFFERSIZE;
		} else {
			break;
		}
	}

	if (cnt > 0) {
		event_uartDataReceived(pReceivedBuffer, cnt);
	}
}

// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
UARTStatus sendUartData(uint8_t *pData, uint8_t size) {
	// check size
	if(notBusyTXBufferCnt + size > TX_BUFFERSIZE){
		return uartStatus_tooMouchData;
	}

	// copy data to buffer
	for (uint8_t cnt = 0; cnt < size; cnt++) {
		pNotBusyTXBuffer[notBusyTXBufferCnt] = pData[cnt];
		notBusyTXBufferCnt++;
		if (notBusyTXBufferCnt > TX_BUFFERSIZE) {
			if (!txIsBusy) {
				// uart not busy, send first buffer and write data in second
				HAL_UART_Transmit_IT(pUART_handle, pNotBusyTXBuffer, notBusyTXBufferCnt);
				switchTXBuffers();
				txIsBusy = true;
			} else {
				// buffer overflow
				PLATFORM_ERROR;
			}
		}
	}

	if (!txIsBusy) {
		// start
		HAL_UART_Transmit_IT(pUART_handle, pNotBusyTXBuffer, notBusyTXBufferCnt);
		switchTXBuffers();
		txIsBusy = true;
	}
	return uartStatus_ok;
}

// --------------- platformAdapter.h-------- -------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (notBusyTXBufferCnt > 0) {
		// start
		HAL_UART_Transmit_IT(pUART_handle, pNotBusyTXBuffer, notBusyTXBufferCnt);
		switchTXBuffers();
	} else {
		txIsBusy = false;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	switch(activeBufferHalf){
	case bufferHalf_upper:
		// check: is second half of RX buffer free?
		if (firstNotHandledRXByte > HALF_RX_BUFFERSIZE) {
			// --> not free
			PLATFORM_ERROR;
		}

		HAL_UART_Receive_IT(pUART_handle, pRXBuffer + HALF_RX_BUFFERSIZE, HALF_RX_BUFFERSIZE); // restart with lower buffer half
		activeBufferHalf = bufferHalf_lower;
		break;

	case bufferHalf_lower:
		// check: is first half of RX buffer free?
		if (firstNotHandledRXByte < HALF_RX_BUFFERSIZE) {
			// --> not free
			PLATFORM_ERROR;
		}

		HAL_UART_Receive_IT(pUART_handle, pRXBuffer, HALF_RX_BUFFERSIZE); // restart with upper buffer half
		activeBufferHalf = bufferHalf_upper;
		break;
	}



}

