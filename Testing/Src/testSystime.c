/*
 * allTests.c
 *
 *  Created on: Jul 30, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAPI.h"
#include "testFramework.h"
#include "tests.h"

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

static void callback1(){
	callback1Flag = 1;
	ASSERT(timestamp_startDelayedCallbackTest + LONG_TIME_us);
}
static void callback2(){
	callback2Flag = 1;
	ASSERT((timestamp_startDelayedCallbackTest + MEDIUM_TIME_us) < getSystimeUs());
}
static void callback3(){
	callback3Flag = 1;
	ASSERT((timestamp_startDelayedCallbackTest + SHORT_TIME_us) < getSystimeUs());
}
static void callback4(){
	callback4Flag = 1;
	ASSERT((timestamp_startDelayedCallbackTest + ULTRA_SHORT_TIME_us) < getSystimeUs());
}
static void callback5(){
	// not to call
	ASSERT(0);
}

static void startTestDelayedCallback(){
	timestamp_startDelayedCallbackTest = getSystimeUs();

	startDelayedCallback(LONG_TIME_us, &callback1);
	startDelayedCallback(MEDIUM_TIME_us, &callback2);
	startDelayedCallback(SHORT_TIME_us, &callback3);
	startDelayedCallback(ULTRA_SHORT_TIME_us, &callback4);
}

// --------------- tests.h ----------------------------------------------------------------------------------------------------------
TestFeedback testSystime(){
	logString("Testing of time functions...\n\r");
	// ******** test first use *********************************************************************************
	startTestDelayedCallback();
	wait_ms(LONG_TIME_ms*2);
	// check & reset flags
	ASSERT(callback1Flag);
	callback1Flag = 0;
	ASSERT(callback2Flag);
	callback2Flag = 0;
	ASSERT(callback3Flag);
	callback3Flag = 0;
	ASSERT(callback4Flag);
	callback4Flag = 0;

	// ******** test second use *********************************************************************************
	startTestDelayedCallback();
	wait_ms(LONG_TIME_ms*2);
	// check & reset flags
	ASSERT(callback1Flag);
	callback1Flag = 0;
	ASSERT(callback2Flag);
	callback2Flag = 0;
	ASSERT(callback3Flag);
	callback3Flag = 0;
	ASSERT(callback4Flag);
	callback4Flag = 0;

	// ******** test platform error (not enough delayed callbacks) **********************************************
	ASSERT(!platformErrorFlag);
	startDelayedCallback(LONG_TIME_us, &callback1);
	startDelayedCallback(LONG_TIME_us, &callback2);
	startDelayedCallback(LONG_TIME_us, &callback3);
	startDelayedCallback(LONG_TIME_us, &callback4);
	startDelayedCallback(LONG_TIME_us, &callback5);
	// check flags
	ASSERT(platformErrorFlag);
	logString("Passed!\n\n\r");

	return testFeedback_finished;
}
void testSystime_platformError(char file[], uint32_t line){
	platformErrorFlag = 1;
}
