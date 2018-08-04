/*
 * testADCs.c
 *
 *  Created on: Aug 3, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAPI.h"
#include "testFramework.h"
#include "tests.h"

// =============== Defines =============================================================================================================
#define NR_OF_MEASURING 5
#define WAITING_TIME_FOR_ADC_ms 100
// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================

// --------------- tests.h ----------------------------------------------------------------------------------------------------------
void testADCs() {
	logString("Testing of ADC functions...\n\r");

	// ******** test current phase A *********************************************************************************
	logString(" Phase A:\n\r");
	MUST_BE_CALLED(&testADCs_newData_currentPhaseA, NR_OF_MEASURING);
	for (uint8_t cnt = 0; cnt < NR_OF_MEASURING; cnt++) {
		ASSERT(startMeasCurrentPhaseA() == adcStatus_started);
		wait_ms(WAITING_TIME_FOR_ADC_ms);
	}
	checkMustBeCalled();

	// ******** test current phase B *********************************************************************************
	logString(" Phase B:\n\r");
	MUST_BE_CALLED(&testADCs_newData_currentPhaseB, NR_OF_MEASURING);
	for (uint8_t cnt = 0; cnt < NR_OF_MEASURING; cnt++) {
		ASSERT(startMeasCurrentPhaseB() == adcStatus_started);
		wait_ms(WAITING_TIME_FOR_ADC_ms);
	}
	checkMustBeCalled();

	// ******** test control signal **********************************************************************************
	MUST_BE_CALLED(&testADCs_newData_controlSignal, NR_OF_MEASURING);
	for (uint8_t cnt = 0; cnt < NR_OF_MEASURING; cnt++) {
		ASSERT(startMeasControlSignal() == adcStatus_started);
		wait_ms(WAITING_TIME_FOR_ADC_ms);
	}
	checkMustBeCalled();

	// ******** test main voltage ************************************************************************************
	MUST_BE_CALLED(&testADCs_newData_mainVoltage, NR_OF_MEASURING);
	for (uint8_t cnt = 0; cnt < NR_OF_MEASURING; cnt++) {
		ASSERT(startMeasMainVoltage() == adcStatus_started);
		wait_ms(WAITING_TIME_FOR_ADC_ms);
	}
	checkMustBeCalled();

	// ******** test main voltage ************************************************************************************
	MUST_BE_CALLED(&testADCs_newData_calibrateEncoder, NR_OF_MEASURING);
	for (uint8_t cnt = 0; cnt < NR_OF_MEASURING; cnt++) {
		ASSERT(startMeasEncoderCalibration() == adcStatus_started);
		wait_ms(WAITING_TIME_FOR_ADC_ms);
	}
	checkMustBeCalled();

	logString("Passed!\n\n\r");
}

void testADCs_newData_currentPhaseA(int32_t current, uint32_t range) {
	CALL(&testADCs_newData_currentPhaseA);
	logSigned("   Current", 10, current, "mA", 2);
	logSigned("   Range", 8, range, "mA", 2);
}
void testADCs_newData_currentPhaseB(int32_t current, uint32_t range) {
	CALL(&testADCs_newData_currentPhaseB);
	logSigned("   Current", 10, current, "mA", 2);
	logSigned("   Range", 8, range, "mA", 2);
}
void testADCs_newData_controlSignal(uint32_t controlSignal) {
	CALL(&testADCs_newData_controlSignal);
	logSigned(" Control Signal", 15, controlSignal, "", 0);
}
void testADCs_newData_mainVoltage(uint32_t mainVoltage) {
	CALL(&testADCs_newData_mainVoltage);
	logSigned(" Main Voltage", 13, mainVoltage, "mV", 2);
}
void testADCs_newData_calibrateEncoder(uint32_t calibrateEncoder) {
	CALL(&testADCs_newData_calibrateEncoder);
	logSigned(" Encoder Calibration", 20, calibrateEncoder, "°", 1);
}

void testADCs_platformError(char file[], uint32_t line) {
	logString("Platform error: ");
	assert(0, file, line);
}
