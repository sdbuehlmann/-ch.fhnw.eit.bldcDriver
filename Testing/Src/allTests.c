/*
 * allTests.c
 *
 *  Created on: Jul 30, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAPI.h"

// =============== Defines =============================================================================================================
#define LONG_TIME_us 		1000000
#define MEDIUM_TIME_us 		500000
#define SHORT_TIME_us		250000
#define ULTRA_SHORT_TIME_us 125000

#define LONG_TIME_ms 		1000
#define MEDIUM_TIME_ms 		500
#define SHORT_TIME_ms		250
#define ULTRA_SHORT_TIME_ms 125

// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================
static uint32_t timestamp_startDelayedCallbackTest = 0;
static uint8_t platformErrorFlag = 0;

static uint8_t callback1Flag = 0;
static uint8_t callback2Flag = 0;
static uint8_t callback3Flag = 0;
static uint8_t callback4Flag = 0;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================
// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
void assert(uint8_t isTrue, char file[], uint32_t line){
	if(!isTrue){
		uint8_t buffer[200] = { 0 };
		sprintf(buffer, "\n\rTest failed! File: %s Line: %u\n\r", file, line);
		sendUartData(buffer, 200);
		// stop
		while(1){
			setLED(boolean_true);
			wait_ms(200);
			setLED(boolean_false);
			wait_ms(200);
		}
	}
}

void callback1(){
	callback1Flag = 1;
	assert((timestamp_startDelayedCallbackTest + LONG_TIME_us) < getSystimeUs(), __FILE__, __LINE__);
}
void callback2(){
	callback2Flag = 1;
	assert((timestamp_startDelayedCallbackTest + MEDIUM_TIME_us) < getSystimeUs(), __FILE__, __LINE__);
}
void callback3(){
	callback3Flag = 1;
	assert((timestamp_startDelayedCallbackTest + SHORT_TIME_us) < getSystimeUs(), __FILE__, __LINE__);
}
void callback4(){
	callback4Flag = 1;
	assert((timestamp_startDelayedCallbackTest + ULTRA_SHORT_TIME_us) < getSystimeUs(), __FILE__, __LINE__);
}
void callback5(){
	// not to call
	assert(0, __FILE__, __LINE__);
}


void startTestDelayedCallback(){
	timestamp_startDelayedCallbackTest = getSystimeUs();

	startDelayedCallback(LONG_TIME_us, &callback1);
	startDelayedCallback(MEDIUM_TIME_us, &callback2);
	startDelayedCallback(SHORT_TIME_us, &callback3);
	startDelayedCallback(ULTRA_SHORT_TIME_us, &callback4);
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



void startup(){
	logString("\n\r*** START BLDC MOTOR DRIVER PLATFORM TESTS ***\n\r");
	logString("Testing of time functions... ");
	// ******** test first use *********************************************************************************
	startTestDelayedCallback();
	wait_ms(LONG_TIME_ms*2);
	// check & reset flags
	assert(callback1Flag, __FILE__, __LINE__);
	callback1Flag = 0;
	assert(callback2Flag, __FILE__, __LINE__);
	callback2Flag = 0;
	assert(callback3Flag, __FILE__, __LINE__);
	callback3Flag = 0;
	assert(callback4Flag, __FILE__, __LINE__);
	callback4Flag = 0;

	// ******** test second use *********************************************************************************
	startTestDelayedCallback();
	wait_ms(LONG_TIME_ms*2);
	// check & reset flags
	assert(callback1Flag, __FILE__, __LINE__);
	callback1Flag = 0;
	assert(callback2Flag, __FILE__, __LINE__);
	callback2Flag = 0;
	assert(callback3Flag, __FILE__, __LINE__);
	callback3Flag = 0;
	assert(callback4Flag, __FILE__, __LINE__);
	callback4Flag = 0;

	// ******** test platform error (not enough delayed callbacks) **********************************************
	assert(!platformErrorFlag, __FILE__, __LINE__);
	startDelayedCallback(LONG_TIME_us, &callback1);
	startDelayedCallback(LONG_TIME_us, &callback2);
	startDelayedCallback(LONG_TIME_us, &callback3);
	startDelayedCallback(LONG_TIME_us, &callback4);
	startDelayedCallback(LONG_TIME_us, &callback5);
	// check flags
	assert(platformErrorFlag, __FILE__, __LINE__);
	logString("Passed! \n\r");

	logString("Passed all tests.\n\r");
}
void proceed(){
	setLED(boolean_true); // passed all tests
}
void platformError(char msg[], char file[], char line[]){
	platformErrorFlag = 1;
}


void event_uartDataReceived(uint8_t data){

}
