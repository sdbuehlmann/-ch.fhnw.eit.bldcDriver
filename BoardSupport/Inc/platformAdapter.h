/*
 * bldc_driver_adapter.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_PLATFORMADAPTER_H_
#define INC_PLATFORMADAPTER_H_

#include "stm32f3xx_hal.h"

#define PWM_A_LS_channel 					TIM_CHANNEL_1 // TIM1
#define PWM_A_HS_channel 					TIM_CHANNEL_2 // TIM1
#define PWM_B_LS_channel 					TIM_CHANNEL_3 // TIM8
#define PWM_B_HS_channel 					TIM_CHANNEL_4 // TIM8
#define PWM_C_LS_channel 					TIM_CHANNEL_1 // TIM8
#define PWM_C_HS_channel 					TIM_CHANNEL_2 // TIM8

#define ADC_CURRENT_A_channel				ADC_CHANNEL_3
#define ADC_CURRENT_B_channel				ADC_CHANNEL_4
#define ADC_MAIN_VOLTAGE_channel			ADC_CHANNEL_7
#define ADC_ENCODER_CALIBRATION_channel		ADC_CHANNEL_8
#define ADC_CONTROL_VOLTAGE_channel			ADC_CHANNEL_1


#define DELAYED_CALLBACK_A_channel 			TIM_CHANNEL_1 // TIM2
#define DELAYED_CALLBACK_B_channel 			TIM_CHANNEL_2 // TIM2
#define DELAYED_CALLBACK_C_channel 			TIM_CHANNEL_3 // TIM2
#define DELAYED_CALLBACK_D_channel 			TIM_CHANNEL_4 // TIM2

#define DELAYED_CALLBACK_A_ir_flag 			TIM_FLAG_CC1 // TIM2
#define DELAYED_CALLBACK_B_ir_flag 			TIM_FLAG_CC2 // TIM2
#define DELAYED_CALLBACK_C_ir_flag 			TIM_FLAG_CC3 // TIM2
#define DELAYED_CALLBACK_D_ir_flag 			TIM_FLAG_CC4 // TIM2

#define DECODER_COUNT_channel 				TIM_CHANNEL_1 // TIM15
#define DECODER_COUNT_ir_flag 				TIM_FLAG_CC1 // TIM15

// =============== variables ==================================================================================================
ADC_HandleTypeDef *pHallB_ADC_handle;
ADC_HandleTypeDef *pHallA_ADC_handle;
ADC_HandleTypeDef *pUser_ADC_handle;
ADC_HandleTypeDef *pMainVoltage_EncoderPoti_ADC_handle;

TIM_HandleTypeDef *pPWM_handle_A;
TIM_HandleTypeDef *pPWM_handle_B_and_C;

TIM_HandleTypeDef *pSystemtime_Timer_handle;
TIM_HandleTypeDef *pEncoder_Counter_handle;

SPI_HandleTypeDef *pSPI_handle;

UART_HandleTypeDef *pUART_handle;

// =============== functions ==================================================================================================
void startupPlatform();
void proceedPlatform();

void extIRQ_compPhaseA();
void extIRQ_compPhaseB();
void extIRQ_compPhaseC();
void extIRQ_encoderInReferencePos();

void adcIRQ_mainPower_and_encoderCalibration();
void adcIRQ_userIn();

void dmaIRQ_newValuesHallA();
void dmaIRQ_newValuesHallB();

void timerIRQ_systime();
void timerIRQ_encoderPulses();

void uartIRQ_dataSendet();
void uartIRQ_dataReceived();

#endif /* INC_PLATFORMADAPTER_H_ */
