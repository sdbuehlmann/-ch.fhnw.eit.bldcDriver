/*
 * testFramework.c
 *
 *  Created on: Aug 2, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include <stdio.h>

#include "platformAPI.h"
#include "testFramework.h"
#include "tests.h"

// =============== Defines =============================================================================================================
#define STOP_LED_BLINK_TIME_ms 100

// =============== Typdefs =============================================================================================================
typedef enum {
	test_beginning, test_uart, test_systime, test_adc, test_pwm, test_extIRQ, test_sync, test_end
} Test;

// =============== Variables ===========================================================================================================
static Test testState = test_beginning;

static ToBeCalled_Function toBeCalledFunctions[100];
static uint8_t toBeCalledFunctions_cnt = 0;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================
// --------------- testFramework.h -----------------------------------------------------------------------------------------------------
void assert(uint8_t isTrue, char file[], uint32_t line) {
	if (!isTrue) {
		uint8_t buffer[200] = { 0 };
		sprintf(buffer, "\n\rTest failed because of a wrong assert! File: %s Line: %lu\n\r", file, line);
		sendUartData(buffer, 200);

		stopTesting();
	}
}

void logString(char pMsg[]) {
	uint32_t cnt = 0;
	char * pOrigMsg = pMsg;
	while (*pMsg) {
		pMsg++;
		cnt++;
	}

	sendUartData((uint8_t*) pOrigMsg, cnt);
}
void logUnsigned(char *pName, uint8_t nameLenght, uint32_t var, char *pUnit, uint8_t unitLenght) {
	char str[nameLenght + 2 + 11 + unitLenght + 4];
	sprintf(str, "%s: %u%s\n\r", pName, var, pUnit);
	logString(str);
}
void logSigned(char *pName, uint8_t nameLenght, int32_t var, char *pUnit, uint8_t unitLenght) {
	char str[nameLenght + 2 + 11 + unitLenght + 4];
	sprintf(str, "%s: %i%s\n\r", pName, var, pUnit);
	logString(str);
}

void stopTesting() {
	while (1) {
		setLED(true);
		wait_ms(STOP_LED_BLINK_TIME_ms);
		setLED(false);
		wait_ms(STOP_LED_BLINK_TIME_ms);
	}
}

void mustBeCalled(Funcptr function, uint32_t nrToBeCalled, char file[], uint32_t line) {
	toBeCalledFunctions[toBeCalledFunctions_cnt].function = function;
	toBeCalledFunctions[toBeCalledFunctions_cnt].nrToBeCalled = nrToBeCalled;
	toBeCalledFunctions[toBeCalledFunctions_cnt].pFilename = file;
	toBeCalledFunctions[toBeCalledFunctions_cnt].line = line;

	toBeCalledFunctions_cnt++;
}
void call(Funcptr function) {
	for (uint8_t cnt = 0; cnt < toBeCalledFunctions_cnt; cnt++) {
		if (function == toBeCalledFunctions[cnt].function) {
			if (toBeCalledFunctions[cnt].nrToBeCalled > 0) {
				toBeCalledFunctions[cnt].nrToBeCalled--;
			} else {
				uint8_t buffer[200] = { 0 };
				sprintf(buffer, "\n\rToo much function calls, test failed! File: %s Line: %lu\n\r", toBeCalledFunctions[cnt].pFilename,
						toBeCalledFunctions[cnt].line);
				sendUartData(buffer, 200);

				stopTesting();
			}
		}
	}
}

void checkMustBeCalled() {
	for (uint8_t cnt = 0; cnt < toBeCalledFunctions_cnt; cnt++) {
		if (toBeCalledFunctions[cnt].nrToBeCalled != 0) {
			uint8_t buffer[200] = { 0 };
			sprintf(buffer, "\n\rTest failed because of not called functions! File: %s Line: %lu\n\r", toBeCalledFunctions[cnt].pFilename, toBeCalledFunctions[cnt].line);
			sendUartData(buffer, 200);

			stopTesting();
		}
	}
}

// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
void startup() {
	logString("\n\n\n\r*** START BLDC MOTOR DRIVER PLATFORM TESTS ***\n\r");

	testState = test_systime;
	testSystime();

	testState = test_adc;
	testADCs();

	logString("*** All tests passed ***\n\r");
}
void proceed() {
	setLED(true);
}
void platformError(char file[], uint32_t line) {
	switch (testState) {
	case test_systime:
		testSystime_platformError(file, line);
		break;
	case test_adc:
		testADCs_platformError(file, line);
		break;
	default:
		break;
	}
}

void newData_currentPhaseA(int32_t current, uint32_t range) {
	switch (testState) {
	case test_adc:
		testADCs_newData_currentPhaseA(current, range);
		break;
	default:
		break;
	}
}
void newData_currentPhaseB(int32_t current, uint32_t range) {
	switch (testState) {
	case test_adc:
		testADCs_newData_currentPhaseB(current, range);
		break;
	default:
		break;
	}
}

void newData_controlSignal(uint32_t controlSignal) {
	switch (testState) {
	case test_adc:
		testADCs_newData_controlSignal(controlSignal);
		break;
	default:
		break;
	}
}
void newData_mainVoltage(uint32_t mainVoltage) {
	switch (testState) {
	case test_adc:
		testADCs_newData_mainVoltage(mainVoltage);
		break;
	default:
		break;
	}
}
void newData_calibrateEncoder(uint32_t calibrateEncoder) {
	switch (testState) {
	case test_adc:
		testADCs_newData_calibrateEncoder(calibrateEncoder);
		break;
	default:
		break;
	}
}
