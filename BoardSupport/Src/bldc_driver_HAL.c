/*
 * bldc_driver_HAL.c
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#include "main.h"

#include "platformAPI.h"
#include "platformAdapter.h"

// decoder callbacks
static void (*listener_encoderInReferencePosition)(void);
static void (*listener_encoderSignalChanged)(void);
static void (*listener_rotated180Deg)(void);

// comperator interrupt listeners
static void (*listenerPhaseA)(uint8_t edge);
static void (*listenerPhaseB)(uint8_t edge);
static void (*listenerPhaseC)(uint8_t edge);


// timer callbacks


void initBLDCDriver(
		ADC_HandleTypeDef *pHallB_ADC_handle_param,
		ADC_HandleTypeDef *pHallA_ADC_handle_param,
		ADC_HandleTypeDef *pUser_ADC_handle_param,
		ADC_HandleTypeDef *pMainVoltage_EncoderPoti_ADC_handle_param,

		TIM_HandleTypeDef *pPWM_handle_A_param,
		TIM_HandleTypeDef *pPWM_handle_B_and_C_param,

		TIM_HandleTypeDef *pSystemtime_Timer_param,
		TIM_HandleTypeDef *pEncoder_Counter_handle_param,

		SPI_HandleTypeDef *pSPI_handle_param,
		UART_HandleTypeDef *pUART_handle_param){

	pHallB_ADC_handle = pHallB_ADC_handle_param;
	pHallA_ADC_handle = pHallA_ADC_handle_param;
	pUser_ADC_handle = pUser_ADC_handle_param;
	pMainVoltage_EncoderPoti_ADC_handle =
			pMainVoltage_EncoderPoti_ADC_handle_param;

	pPWM_handle_A = pPWM_handle_A_param;
	pPWM_handle_B_and_C = pPWM_handle_B_and_C_param;

	pSystemtime_Timer_handle = pSystemtime_Timer_param;
	pEncoder_Counter_handle = pEncoder_Counter_handle_param;

	pUART_handle = pUART_handle_param;
	pSPI_handle = pSPI_handle_param;

}
void startupBLDCDriver() {
	startup();
}
void proceedBLDCDriver() {
	proceed();
}

//========================= GPIO'S ===================================
void initGPIOs() {
}

// led's
void switch_PowerLED(uint8_t state) {
	HAL_GPIO_WritePin(DO_LED_1_GPIO_Port, DO_LED_1_Pin, state);
}

// bridge driver
void switch_Enable_BridgeDriver(uint8_t state) {
	HAL_GPIO_WritePin(DO_DRIVER_EN_GPIO_Port, DO_DRIVER_EN_Pin, state);
}
void switch_DCCal_BridgeDriver(uint8_t state) {
	HAL_GPIO_WritePin(DO_DRIVER_DC_CAL_GPIO_Port, DO_DRIVER_DC_CAL_Pin, state);
}
uint8_t read_NFault_BridgeDriver() {
	return HAL_GPIO_ReadPin(DI_DRIVER_NFAULT_GPIO_Port, DI_DRIVER_NFAULT_Pin);
}
uint8_t read_NOCTW_BridgeDriver() {
	return HAL_GPIO_ReadPin(DI_DRIVER_NOCTW_GPIO_Port, DI_DRIVER_NOCTW_Pin);
}
uint8_t read_PWRGD_BridgeDriver() {
	return HAL_GPIO_ReadPin(DI_DRIVER_PWRGD_GPIO_Port, DI_DRIVER_PWRGD_Pin);
}

// main interface
uint8_t read_StateButton() {
	return HAL_GPIO_ReadPin(DI_USER_IN_GPIO_Port, DI_USER_IN_Pin);
}

uint8_t read_MainButton() {
	return HAL_GPIO_ReadPin(DI_ENABLE_PRINT_GPIO_Port, DI_ENABLE_PRINT_Pin);
}

//========================= UART ===================================
void initUART() {

}

void sendByteOverUART(uint8_t data) {
	HAL_UART_Transmit(pUART_handle, &data, 1, 100);
}

//========================= COMPERATORS ==============================
void initComp() {
}

void register_comperatorListener_phaseA(void (*listener)(uint8_t)) {
	listenerPhaseA = listener;
}
void register_comperatorListener_phaseB(void (*listener)(uint8_t)) {
	listenerPhaseB = listener;
}
void register_comperatorListener_phaseC(void (*listener)(uint8_t)) {
	listenerPhaseC = listener;
}

void enableCompA(uint8_t enable) {
	if (enable) {
		HAL_NVIC_EnableIRQ(IR_COMP_A_EXTI_IRQn);
	} else {
		HAL_NVIC_DisableIRQ(IR_COMP_A_EXTI_IRQn);
	}
}
void enableCompB(uint8_t enable) {
	if (enable) {
		HAL_NVIC_EnableIRQ(IR_COMP_B_EXTI_IRQn);
	} else {
		HAL_NVIC_DisableIRQ(IR_COMP_B_EXTI_IRQn);
	}
}
void enableCompC(uint8_t enable) {
	if (enable) {
		HAL_NVIC_EnableIRQ(IR_COMP_C_EXTI_IRQn);
	} else {
		HAL_NVIC_DisableIRQ(IR_COMP_C_EXTI_IRQn);
	}
}

void phaseAComp_interrupt() {
	if (listenerPhaseA != 0) {
		listenerPhaseA(read_signal_compA());
	}
}
void phaseBComp_interrupt() {
	if (listenerPhaseB != 0) {
		listenerPhaseB(read_signal_compB());
	}
}
void phaseCComp_interrupt() {
	if (listenerPhaseC != 0) {
		listenerPhaseC(read_signal_compC());
	}
}

uint8_t read_signal_compA() {
	return HAL_GPIO_ReadPin(IR_COMP_A_GPIO_Port, IR_COMP_A_Pin);
}
uint8_t read_signal_compB() {
	return HAL_GPIO_ReadPin(IR_COMP_B_GPIO_Port, IR_COMP_B_Pin);
}
uint8_t read_signal_compC() {
	return HAL_GPIO_ReadPin(IR_COMP_C_GPIO_Port, IR_COMP_C_Pin);
}

//========================= PWM ===================================

//========================= ENCODER ==============================
void initEncoder() {
	HAL_TIM_Base_Start_IT(pEncoder_Counter_handle);
}

void enableSingleIRQ_encoderSignalA(void (*listener)(void)) {
	listener_encoderSignalChanged = listener;
	HAL_NVIC_EnableIRQ(IR_INC_A_EXTI_IRQn);
}

void enableIRQ_encoderSignalReferencePos(void (*listener)(void)) {
	listener_encoderInReferencePosition = listener;
	HAL_NVIC_EnableIRQ(IR_INC_REF_EXTI_IRQn);
}
void disableIRQ_encoderSignalReferencePos() {
	HAL_NVIC_DisableIRQ(IR_INC_REF_EXTI_IRQn);
	listener_encoderInReferencePosition = 0;
}

void registerListener_rotated180Deg(void (*listener)(void)) {
	listener_rotated180Deg = listener;
}

uint8_t read_encoderSignalA() {
	return HAL_GPIO_ReadPin(IR_INC_A_GPIO_Port, IR_INC_A_Pin);
}
uint8_t read_encoderSignalB() {
	return HAL_GPIO_ReadPin(DI_INC_B_GPIO_Port, DI_INC_B_Pin);
}
uint8_t read_encoderEnable() {
	return HAL_GPIO_ReadPin(DI_INC_ENABLE_GPIO_Port, DI_INC_ENABLE_Pin);
}
uint8_t read_encoderCalibrate() {
	return HAL_GPIO_ReadPin(DI_INC_CALIBRATE_GPIO_Port, DI_INC_CALIBRATE_Pin);
}

uint32_t getNrImpulses_encoderSignalA() {
	return __HAL_TIM_GET_COUNTER(pEncoder_Counter_handle);
}
void resetNrImpulses_encoderSignalA() {
	__HAL_TIM_SET_COUNTER(pEncoder_Counter_handle, 0); // reset timer
}
void setNrImpulses_encoderSignalA(uint16_t nrImpulses) {
	__HAL_TIM_SET_COUNTER(pEncoder_Counter_handle, nrImpulses);
}

// calibration
uint32_t measAnalog_encoderCalibrationPoti_BLOCKING() {
	HAL_ADC_Start(pMainVoltage_EncoderPoti_ADC_handle);
	while (1) {
		if (HAL_ADC_PollForConversion(pMainVoltage_EncoderPoti_ADC_handle, 0)
				== HAL_OK) {
			return HAL_ADC_GetValue(pMainVoltage_EncoderPoti_ADC_handle);
		}
	}
}
void switch_encoderPositionPin(uint8_t state) {
	HAL_GPIO_WritePin(DO_INC_POSITION_GPIO_Port, DO_INC_POSITION_Pin, state);
}

// interrupt callbacks
void encoderReferencePosition_IRQ() {
	if (listener_encoderInReferencePosition != 0) {
		listener_encoderInReferencePosition();
	}
}
void encoderSignalA_IRQ() {
	if (listener_encoderSignalChanged != 0) {
		void (*temp)(void);
		temp = listener_encoderSignalChanged;
		listener_encoderSignalChanged = 0;
		temp();
	}
}
void encoderTicksCompare_IRQ() {
	if (listener_rotated180Deg != 0) {
		listener_rotated180Deg();
	}
}
