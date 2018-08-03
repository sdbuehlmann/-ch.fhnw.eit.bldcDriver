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
#define NR_DMA_MEASUREMENTS 							9
#define MAX_INIT_TIME_ms								1000

// phase current
#define HALLSENSOR_ADC_VALUE_OFFSET 					(0b111111111111/2) // 12bit ADC
#define HALLSENSOR_VOLTAGE_OFFSET_mV					(3300/2)
#define HALLSENSOR_VOLTAGE_OFFSET_TOL_mV				(36/2) // Datasheet Hall Sensor p.6 peak to peak noise: 36mV
#define HALLSENSOR_VOLTAGE_OFFSET_UPPER_TOL_LIMIT		(HALLSENSOR_VOLTAGE_OFFSET_mV + HALLSENSOR_VOLTAGE_OFFSET_TOL_mV)
#define HALLSENSOR_VOLTAGE_OFFSET_LOWER_TOL_LIMIT		(HALLSENSOR_VOLTAGE_OFFSET_mV - HALLSENSOR_VOLTAGE_OFFSET_TOL_mV)

// main voltage
#define MAIN_VOLTAGE_DIVIDER_NUMERATOR					34
#define MAIN_VOLTAGE_DIVIDER_DENOMINATOR				269

#define MAX_VOLTAGE_IN_mV 								3300
#define MAX_VALUE_ADC 									0b111111111111 // 12 bit ADC

#define CURRENT_TO_VOLTAGE_RATIO 						38

// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================
static uint8_t mainVoltageAndEncoderCal_callbackCnt = 0;

// flags
static Boolean isInitialized = false;

static Boolean measureHallA = false;
static Boolean measureHallB = false;
static Boolean measureControlSignal = false;
static Boolean measureMainVoltage = false;
static Boolean measureEncoderCalibration = false;

// data buffers
static uint32_t lastMainVoltageValue = 0;
static uint32_t lastEncoderCalibrationValue = 0;
static uint32_t lastControlValue = 0;

static uint32_t pDMABufferA[NR_DMA_MEASUREMENTS];
static uint32_t pDMABufferB[NR_DMA_MEASUREMENTS];

static int32_t lastCurrentAValue = 0;
static int32_t lastCurrentBValue = 0;
static uint32_t lastCurrentARange = 0;
static uint32_t lastCurrentBRange = 0;

static uint32_t hallSensorAOffset_mV = 0;
static uint32_t hallSensorBOffset_mV = 0;

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================
/**
 * Converts the 12 bit ADC Value from the main voltage measurement to the corresponding main voltage in millivolt.
 * @param ADC Value [0 - 0b1111 11111111]
 * @return Corresponding main voltage [0 - 26'109]mV
 */
static uint32_t convert_mainVoltage(uint16_t adcValue);

/**
 * Converts the 12 bit ADC Value from the encoder calibration measurement to the corresponding encoder position angle in degrees.
 * @param ADC Value [0 - 0b1111 11111111]
 * @return encoder position angle [0 - 365]Deg
 */
static uint32_t convert_encoderCalibration(uint16_t adcValue);
static uint32_t convert_controlValue(uint16_t adcValue);
static void convert_current(uint32_t dmaValues[], static uint32_t offsetCalibration, int32_t *pCurrent, uint32_t *pCurrentRange);

static uint32_t calculateOffset(uint32_t dmaValues[]);

/**
 * Converts the 12 bit ADC Value to a corresponding voltage in millivolt.
 * @param ADC Value [0 - 0b1111 11111111]
 * @return Corresponding voltage [0 - 3300]mV
 */
static uint16_t convertADCValueToVoltage(uint16_t adcValue);

/**
 * Converts the voltage from the hall sensors in the corresponding current in milliampere.
 * @param Hall sensor voltage output [0,3300]mV
 * @param Offset of hall sensor voltage.
 * @return Corresponding current [-50,50]mA
 */
static int32_t convertHallSensorVoltageToCurrent(uint16_t voltage_mV, uint16_t offset_mV);

// =============== Functions ===========================================================================================================

static uint32_t convert_mainVoltage(uint16_t adcValue) {
	uint32_t adcVoltage_mV = convertADCValueToVoltage(adcValue);
	uint32_t mainVoltage_mV = (adcVoltage_mV * MAIN_VOLTAGE_DIVIDER_DENOMINATOR) / MAIN_VOLTAGE_DIVIDER_NUMERATOR; // 3300 * 269 = 887'700 --> no overflow

	return mainVoltage_mV;
}
static uint16_t convert_encoderCalibration(uint16_t adcValue) {
	uint16_t pos_deg = (adcValue * 365) / MAX_VALUE_ADC;
	return pos_deg;
}
static uint8_t convert_controlValue(uint16_t adcValue) {
	uint16_t controlValue = (adcValue * 255) / MAX_VALUE_ADC;
	return controlValue;
}
static void convert_current(uint32_t dmaValues[], static uint32_t offset_mV, int32_t *pCurrent, uint32_t *pCurrentRange) {
	uint32_t median = 0;
	uint32_t range = 0;
	calculateMedianAndRange(dmaValues, NR_DMA_MEASUREMENTS, &median, &range);

	uint16_t medianVoltage_mV = convertADCValueToVoltage(median);
	*pCurrent = convertHallSensorVoltageToCurrent(medianVoltage_mV, offset_mV);

	uint16_t rangeVoltage_mV = convertADCValueToVoltage(range);
	*pCurrentRange = convertHallSensorVoltageToCurrent(rangeVoltage_mV, offset_mV);
}

static uint32_t calculateOffset(uint32_t dmaValues[]) {
	uint32_t median = 0;
	uint32_t range = 0;
	calculateMedianAndRange(dmaValues, NR_DMA_MEASUREMENTS, &median, &range);

	int32_t voltage_mv = convertADCValueToVoltage(median);

	// check offset
	if (voltage_mv > HALLSENSOR_VOLTAGE_OFFSET_UPPER_TOL_LIMIT || voltage_mv < HALLSENSOR_VOLTAGE_OFFSET_LOWER_TOL_LIMIT) {
		PLATFORM_ERROR;
	}

	return voltage_mv;
}

uint16_t convertADCValueToVoltage(uint16_t adcValue) {
	return (adcValue * MAX_VOLTAGE_IN_mV) / MAX_VALUE_ADC;
}

int32_t convertHallSensorVoltageToCurrent(uint16_t voltage_mV, uint16_t offset_mV) {
	int32_t offsetfreeVoltage_mV = (voltage_mV - offset_mV);
	return offsetfreeVoltage_mV * CURRENT_TO_VOLTAGE_RATIO;
}

// --------------- platformModules.h -------------------------------------------------------------------------------------------------------
void initADC() {
	// calibrate offset
	startMeasCurrentPhaseA();
	startMeasCurrentPhaseB();

	// waiting for results
	uint32_t cnt = 0;
	while (!isInitialized) {
		wait_ms(10);
		cnt++;
		if (cnt * 10 > MAX_INIT_TIME_ms) {
			PLATFORM_ERROR;
			break;
		}
	}
}

// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
ADCStatus startMeasCurrentPhaseA() {
	if (measureHallA) {
		return adcStatus_running;
	}
	measureHallA = true;

	HAL_ADC_Start_DMA(pHallA_ADC_handle, pDMABufferA, NR_DMA_MEASUREMENTS);

	/* tutorials:
	 * https://www.youtube.com/watch?v=ts5SEoTg2oY
	 * https://visualgdb.com/tutorials/arm/stm32/adc/
	 */
	return adcStatus_started;
}
ADCStatus startMeasCurrentPhaseB() {
	if (measureHallB) {
		return adcStatus_running;
	}
	measureHallB = true;

	HAL_ADC_Start_DMA(pHallB_ADC_handle, pDMABufferB, NR_DMA_MEASUREMENTS);

	return adcStatus_started;
}
ADCStatus startMeasControlSignal() {
	if (measureControlSignal) {
		return adcStatus_running;
	}
	measureControlSignal = 1;

	HAL_ADC_Start_IT(pUser_ADC_handle);

	return adcStatus_started;
}
ADCStatus startMeasMainVoltage() {
	// set flag to inform about results
	measureMainVoltage = true;

	if (measureMainVoltage || measureEncoderCalibration) {
		// already running
		return adcStatus_running;
	}

	// adc is ready to start measurement
	measureMainVoltage = true;
	mainVoltageAndEncoderCal_callbackCnt = 0; // reset count

	HAL_ADC_Start_IT(pMainVoltage_EncoderPoti_ADC_handle);

	return adcStatus_started;
}
ADCStatus startMeasEncoderCalibration() {
	// set flag to inform about results
	measureEncoderCalibration = true;

	if (measureMainVoltage || measureEncoderCalibration) {
		// already running
		return adcStatus_running;
	}

	// adc is ready to start measurement
	measureEncoderCalibration = true;
	mainVoltageAndEncoderCal_callbackCnt = 0; // reset count

	HAL_ADC_Start_IT(pMainVoltage_EncoderPoti_ADC_handle);

	return adcStatus_started;
}

// --------------- platformAdapter.h-------- -------------------------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* pADCHandler) {
	if (pADCHandler == pHallA_ADC_handle) {
		if (!isInitialized) {
			hallSensorAOffset_mV = calculateOffset(pDMABufferA);
		} else {
			convert_current(pDMABufferA, hallSensorAOffset_mV, &lastCurrentAValue, &lastCurrentARange);
		}

	} else if (pADCHandler == pHallB_ADC_handle) {
		if (!isInitialized) {
			hallSensorBOffset_mV = calculateOffset(pDMABufferB);
			isInitialized = true;
		} else {
			convert_current(pDMABufferB, hallSensorBOffset_mV, &lastCurrentBValue, &lastCurrentBRange);
		}

	} else if (pADCHandler == pUser_ADC_handle) {
		uint16_t adcValue = HAL_ADC_GetValue(pADCHandler);
		lastControlValue = convert_controlValue(adcValue);
		if (measureControlSignal) {
			newData_controlSignal(lastMainVoltageValue);
		}

	} else if (pADCHandler == pMainVoltage_EncoderPoti_ADC_handle) {
		uint16_t adcValue = HAL_ADC_GetValue(pADCHandler);
		switch (mainVoltageAndEncoderCal_callbackCnt) {
		case 0:
			// rank 1 --> main voltage
			lastMainVoltageValue = convert_mainVoltage(adcValue);
			if (measureMainVoltage) {
				newData_mainVoltage(lastMainVoltageValue);
			}
			break;
		case 1:
			// rank 2 --> encoder calibration
			lastMainVoltageValue = convert_encoderCalibration(adcValue);
			if (measureEncoderCalibration) {
				newData_calibrateEncoder(lastEncoderCalibrationValue);
			}
			break;
		default:
			PLATFORM_ERROR;
			break;
		}
		mainVoltageAndEncoderCal_callbackCnt++;
	}
}

#endif /* ADC */
