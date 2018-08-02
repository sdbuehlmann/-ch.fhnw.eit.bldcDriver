/*
 * test_uart.c
 *
 *  Created on: Jul 31, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAPI.h"

// =============== Defines =============================================================================================================

// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================
uint8_t testString[] = "This is a test.\n";
volatile uint32_t lengthOfTeststring = 0;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================
// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
void assert(uint8_t isTrue) {
	if (!isTrue) {
		// stop test and signal
		while (1) {
			setLED(boolean_true);
			wait_ms(200);
			setLED(boolean_false);
			wait_ms(200);
		}
	}
}

uint32_t getNrBytes(uint8_t *msg) {
	uint32_t nrBytes = 0;
	while (*msg) // point on first char
	{
		msg++; // increase pointer to next char
		nrBytes++;
	}

	return nrBytes;
}

void startup() {
	lengthOfTeststring = getNrBytes(testString);
	sendUartData(testString, lengthOfTeststring);
	//wait_ms(500);
}
void proceed() {
	setLED(boolean_true); // passed all tests
}
void platformError(char msg[], char file[], char line[]) {

}

void event_uartDataReceived(uint8_t data) {
	static uint32_t receiveCnt;

	uint8_t origSign = testString[receiveCnt];
	receiveCnt++;
	assert(data == origSign);
}
