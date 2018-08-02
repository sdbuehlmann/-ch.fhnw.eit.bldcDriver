/*
 * uart.c
 *
 *  Created on: Aug 2, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAdapter.h"
#include "platformAPI.h"
#include "platformModules.h"
#include "platformAPIConfig.h"

// =============== Defines =============================================================================================================
#define UART_NR_BYTES 10

// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================
static uint8_t uartBufferA[UART_NR_BYTES];
static uint8_t uartBufferB[UART_NR_BYTES];
static uint8_t *pActiveUartBuffer;
static uint8_t *pFirstFreeBufferSpace;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================
// uart
void sendUartData(uint8_t *pData, uint8_t size) {
	HAL_UART_Transmit_IT(pUART_handle, pData, size);
	//HAL_UART_Transmit(pUART_handle, &data, 1, 1);
}

void uartIRQ_dataSendet(){

}
void uartIRQ_dataReceived(){

	event_uartDataReceived(uartBufferA[0]);
}
