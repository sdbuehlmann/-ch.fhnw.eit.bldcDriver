/*
 * adc.c
 *
 *  Created on: Jul 30, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAdapter.h"
#include "platformAPI.h"
#include "platformModules.h"
#include "platformAPIConfig.h"

#ifdef ADC
// =============== Defines ============================================================================================================

// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================
volatile uint8_t hallA_ADC_isRunning = 0;
volatile uint8_t hallB_ADC_isRunning = 0;
volatile uint8_t controlSignal_ADC_isRunning = 0;
volatile uint8_t mainVoltage_and_encoderCalibration_ADC_isRunning = 0;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================

// --------------- platformModules.h -------------------------------------------------------------------------------------------------------
void initADC() {

}

// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
ADCStatus startMeasCurrentPhaseA(uint32_t nr_measurements, uint32_t *pBuffer) {
	if (hallA_ADC_isRunning) {
		return adcStatus_error;
	}
	hallA_ADC_isRunning = 1;

	HAL_ADC_Start_DMA(pHallA_ADC_handle, pBuffer, nr_measurements);

	/* sources:
	 * https://www.youtube.com/watch?v=ts5SEoTg2oY
	 * https://visualgdb.com/tutorials/arm/stm32/adc/
	 */
	return adcStatus_started;
}
ADCStatus startMeasCurrentPhaseB(uint32_t nr_measurements, uint32_t *pBuffer) {
	if (hallB_ADC_isRunning) {
		return adcStatus_error;
	}
	hallB_ADC_isRunning = 1;

	HAL_ADC_Start_DMA(pHallB_ADC_handle, pBuffer, nr_measurements);

	return adcStatus_started;
}
ADCStatus startMeasControlSignal() {
	if (controlSignal_ADC_isRunning) {
		return adcStatus_error;
	}
	controlSignal_ADC_isRunning = 1;

	HAL_ADC_Start_IT(pUser_ADC_handle);

	return adcStatus_started;
}
ADCStatus startMeasMainVoltage() {
	if (mainVoltage_and_encoderCalibration_ADC_isRunning) {
		return adcStatus_error;
	}
	mainVoltage_and_encoderCalibration_ADC_isRunning = 1;

	HAL_ADC_Start_IT(pMainVoltage_EncoderPoti_ADC_handle);

	return adcStatus_started;
}
ADCStatus startMeasEncoderCalibration() {
	return startMeasMainVoltage();
}

// --------------- platformAdapter.h-------- -------------------------------------------------------------------------------------------
void dmaIRQ_newValuesHallA(){

}
void dmaIRQ_newValuesHallB(){

}



void adcIRQ_mainPower() {
	uint32_t tempVoltage = HAL_ADC_GetValue(pMainVoltage_EncoderPoti_ADC_handle);
	tempVoltage = (tempVoltage - 940) * 1.35;

	mainVoltage_and_encoderCalibration_ADC_isRunning = 0;
	newData_mainVoltage(tempVoltage);
}
void adcIRQ_userIn() {
	uint32_t tempVoltage = HAL_ADC_GetValue(pUser_ADC_handle);

	controlSignal_ADC_isRunning = 0;
	newData_controlSignal(tempVoltage);
}
void adcIRQ_encoderCalibration() {

}
#endif /* ADC */
