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
void assert(uint8_t isTrue){
	if(!isTrue){
		// stop test and signal
		while(1){
			setLED(boolean_true);
			wait_ms(200);
			setLED(boolean_false);
			wait_ms(200);
		}
	}
}

void testSystime(){
	uint32_t lastTimestamp_us = getSystimeUs();

	wait_ms(MEDIUM_TIME_ms);

	assert((lastTimestamp_us + MEDIUM_TIME_us) < getSystimeUs());
}

void callback1(){
	callback1Flag = 1;
	assert((timestamp_startDelayedCallbackTest + LONG_TIME_us) < getSystimeUs());
}
void callback2(){
	callback2Flag = 1;
	assert((timestamp_startDelayedCallbackTest + MEDIUM_TIME_us) < getSystimeUs());
}
void callback3(){
	callback3Flag = 1;
	assert((timestamp_startDelayedCallbackTest + SHORT_TIME_us) < getSystimeUs());
}
void callback4(){
	callback4Flag = 1;
	assert((timestamp_startDelayedCallbackTest + ULTRA_SHORT_TIME_us) < getSystimeUs());
}
void callback5(){
	// not to call
	assert(0);
}


void startTestDelayedCallback(){
	timestamp_startDelayedCallbackTest = getSystimeUs();

	startDelayedCallback(LONG_TIME_us, &callback1);
	startDelayedCallback(MEDIUM_TIME_us, &callback2);
	startDelayedCallback(SHORT_TIME_us, &callback3);
	startDelayedCallback(ULTRA_SHORT_TIME_us, &callback4);
}



void startup(){
	testSystime();
	testSystime();
	testSystime();

	// ******** test first use *********************************************************************************
	startTestDelayedCallback();
	wait_ms(LONG_TIME_ms*2);
	// check & reset flags
	assert(callback1Flag);
	callback1Flag = 0;
	assert(callback2Flag);
	callback2Flag = 0;
	assert(callback3Flag);
	callback3Flag = 0;
	assert(callback4Flag);
	callback4Flag = 0;

	// ******** test second use *********************************************************************************
	startTestDelayedCallback();
	wait_ms(LONG_TIME_ms*2);
	// check & reset flags
	assert(callback1Flag);
	callback1Flag = 0;
	assert(callback2Flag);
	callback2Flag = 0;
	assert(callback3Flag);
	callback3Flag = 0;
	assert(callback4Flag);
	callback4Flag = 0;

	// ******** test platform error (not enough delayed callbacks) **********************************************
	assert(!platformErrorFlag);
	startTestDelayedCallback();
	// check flags
	assert(platformErrorFlag);
	assert(callback1Flag);
	assert(callback2Flag);
	assert(callback3Flag);
	assert(callback4Flag);
}
void proceed(){
	setLED(boolean_true); // passed all tests
}
void platformError(char msg[], char file[], char line[]){
	platformErrorFlag = 1;
}


void event_uartDataReceived(uint8_t data){

}
