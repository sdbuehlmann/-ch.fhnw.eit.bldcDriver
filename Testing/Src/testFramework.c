/*
 * testFramework.c
 *
 *  Created on: Aug 2, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAPI.h"
#include "testFramework.h"

// =============== Defines =============================================================================================================
#define STOP_LED_BLINK_TIME_ms 100

// =============== Typdefs =============================================================================================================
typedef enum{
	test_beginning, test_uart, test_systime, test_adc, test_pwm, test_extIRQ, test_sync, test_end
}Test;

// =============== Variables ===========================================================================================================
static Test testState = test_beginning;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================
// --------------- testFramework.h -----------------------------------------------------------------------------------------------------
void assert(uint8_t isTrue, char file[], uint32_t line){
	if(!isTrue){
		uint8_t buffer[200] = { 0 };
		sprintf(buffer, "\n\rTest failed! File: %s Line: %u\n\r", file, line);
		sendUartData(buffer, 200);

		// stop and blink LED
		while(1){
			setLED(boolean_true);
			wait_ms(STOP_LED_BLINK_TIME_ms);
			setLED(boolean_false);
			wait_ms(STOP_LED_BLINK_TIME_ms);
		}
	}
}
void logString(char pMsg[]){
	uint32_t cnt = 0;
	uint8_t * pOrigMsg = pMsg;
	while(*pMsg){
		pMsg++;
		cnt++;
	}

	sendUartData(pOrigMsg, cnt);
}

// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
void startup(){
	logString("\n\r*** START BLDC MOTOR DRIVER PLATFORM TESTS ***\n\r");

	testState = test_systime;
	testSystime();



	logString("*** All tests passed ***\n\r");
}
void proceed(){
	setLED(boolean_true);
}
void platformError(char msg[], char file[], char line[]){
	switch(testState){
	case test_systime:
		testSystime_platformError(msg, file, line);
		break;
	default:
		break;
	}
}
