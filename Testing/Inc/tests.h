/*
 * tests.h
 *
 *  Created on: Aug 3, 2018
 *      Author: simon
 */

#ifndef INC_TESTS_H_
#define INC_TESTS_H_

void testSystime();
void testSystime_platformError(char file[], uint32_t line);

void testADCs();
void testADCs_newData_currentPhaseA(int32_t current, uint32_t range);
void testADCs_newData_currentPhaseB(int32_t current, uint32_t range);
void testADCs_newData_controlSignal(uint32_t controlSignal);
void testADCs_newData_mainVoltage(uint32_t mainVoltage);
void testADCs_newData_calibrateEncoder(uint32_t calibrateEncoder);
void testADCs_platformError(char file[], uint32_t line);

#endif /* INC_TESTS_H_ */
