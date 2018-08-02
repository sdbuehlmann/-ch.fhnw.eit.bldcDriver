/*
 * bldc_driver_adapter.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_PLATFORMADAPTER_H_
#define INC_PLATFORMADAPTER_H_

#include "stm32f3xx_hal.h"

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

void adcIRQ_mainPower();
void adcIRQ_userIn();
void adcIRQ_encoderCalibration();

void dmaIRQ_newValuesHallA();
void dmaIRQ_newValuesHallB();

void timerIRQ_systime();
void timerIRQ_encoderPulses();

void uartIRQ_dataSendet();
void uartIRQ_dataReceived();

#endif /* INC_PLATFORMADAPTER_H_ */
