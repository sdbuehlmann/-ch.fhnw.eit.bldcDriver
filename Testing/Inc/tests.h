/*
 * tests.h
 *
 *  Created on: Aug 3, 2018
 *      Author: simon
 */

#ifndef INC_TESTS_H_
#define INC_TESTS_H_

// =============== Defines =============================================================================================================
//#define TEST_SYSTIME
#define TEST_UART
#define TEST_ADC

// =============== Typdefs =============================================================================================================
typedef enum {
	testFeedback_running, testFeedback_finished
}TestFeedback;

// =============== Functions ===========================================================================================================
TestFeedback testUART();
void testUART_event_uartDataReceived(uint8_t *pData, uint8_t nrData);
void testUART_platformError(char file[], uint32_t line);

TestFeedback testSystime();
void testSystime_platformError(char file[], uint32_t line);

TestFeedback testADCs();
void testADCs_newData_currentPhaseA(int32_t current, uint32_t range);
void testADCs_newData_currentPhaseB(int32_t current, uint32_t range);
void testADCs_newData_controlSignal(uint32_t controlSignal);
void testADCs_newData_mainVoltage(uint32_t mainVoltage);
void testADCs_newData_calibrateEncoder(uint32_t calibrateEncoder);
void testADCs_platformError(char file[], uint32_t line);

TestFeedback testPWM(); /* ToDo: PWM Test implementieren */

TestFeedback testExtIRQ(); /* ToDo: Externe Interrupts Test implementieren */

TestFeedback testSync(); /* ToDo: Synchronisierungs Test implementieren */

/* ToDo: Encoder Test implementieren */
/* ToDo: GPIO Test implementieren */

#endif /* INC_TESTS_H_ */
