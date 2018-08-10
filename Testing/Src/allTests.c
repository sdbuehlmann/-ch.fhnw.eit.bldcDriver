/*
 * allTests.c
 *
 *  Created on: Aug 5, 2018
 *      Author: simon
 */


// =============== Includes ============================================================================================================
#include <stdio.h>

#include "platformAPI.h"
#include "testFramework.h"
#include "tests.h"

// =============== Defines =============================================================================================================


// =============== Typdefs =============================================================================================================
typedef enum {
	test_beginning, test_uart, test_systime, test_adc, test_pwm, test_extIRQ, test_sync, test_end
} Test;

// =============== Variables ===========================================================================================================
static Test testState = test_beginning;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================

// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
void startup() {

}
void proceed() {
	switch (testState) {
	case test_beginning:
		logString("\n\n\n\r******************** START BLDC MOTOR DRIVER PLATFORM TESTS ********************\n\r");

		testState = test_systime;
		break;

	case test_systime:
#ifdef TEST_SYSTIME
		if(testSystime() == testFeedback_finished){
			testState = test_uart;
		}
#else
		testState = test_uart;
#endif /* TEST_SYSTIME */
		break;

	case test_uart:
#ifdef TEST_UART
		if(testUART() == testFeedback_finished){
			testState = test_adc;
		}
#else
		testState = test_adc;
#endif /* TEST_UART */
		break;

	case test_adc:
#ifdef TEST_ADC
		if(testADCs() == testFeedback_finished){
			testState = test_pwm;
		}
#else
		testState = test_pwm;
#endif /* TEST_ADC */
		break;

	case test_pwm:
		testState = test_extIRQ;
		break;

	case test_extIRQ:
		testState = test_end;
		logString("******************** All tests passed ********************\n\r");
		setLED(true);
		break;

	case test_end:

		break;

	default:
		break;
	}

}
void platformError(char file[], uint32_t line) {
	switch (testState) {
	case test_systime:
		testSystime_platformError(file, line);
		break;
	case test_uart:
		testUART_platformError(file, line);
		break;
	case test_adc:
		testADCs_platformError(file, line);
		break;
	default:
		//logPlatformError(file, line);
		break;
	}
}

void event_uartDataReceived(uint8_t *pData, uint8_t nrData) {
	switch (testState) {
	case test_uart:
		testUART_event_uartDataReceived(pData, nrData);
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
