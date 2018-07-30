/*
 * bldc_driver_adapter.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_BLDC_DRIVER_ADAPTER_H_
#define INC_BLDC_DRIVER_ADAPTER_H_

#include "stm32f3xx_hal.h"

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
		UART_HandleTypeDef *pUART_handle_param);

void startupBLDCDriver();
void proceedBLDCDriver();

void phaseAComp_interrupt();
void phaseBComp_interrupt();
void phaseCComp_interrupt();

void hallB_shuntA_adc_interrupt();
void hallA_shuntB_adc_interrupt();

void hallA_dma_new_values();
void hallB_dma_new_values();

void systime_interrupt();

void callback_ADC_mainPower_IRQ();
void callback_ADC_userIn_IRQ();

void encoderReferencePosition_IRQ();
void encoderSignalA_IRQ();
void encoderTicksCompare_IRQ();

#endif /* INC_BLDC_DRIVER_ADAPTER_H_ */
