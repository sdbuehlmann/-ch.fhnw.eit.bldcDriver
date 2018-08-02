/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define IR_COMP_B_Pin GPIO_PIN_15
#define IR_COMP_B_GPIO_Port GPIOC
#define IR_COMP_B_EXTI_IRQn EXTI15_10_IRQn
#define DI_INC_ENABLE_Pin GPIO_PIN_0
#define DI_INC_ENABLE_GPIO_Port GPIOF
#define DI_INC_CALIBRATE_Pin GPIO_PIN_1
#define DI_INC_CALIBRATE_GPIO_Port GPIOF
#define IR_COMP_A_Pin GPIO_PIN_0
#define IR_COMP_A_GPIO_Port GPIOC
#define IR_COMP_A_EXTI_IRQn EXTI0_IRQn
#define ADC_MAIN_VOLTAGE_Pin GPIO_PIN_1
#define ADC_MAIN_VOLTAGE_GPIO_Port GPIOC
#define ADC_INC_CALIBRATE_Pin GPIO_PIN_2
#define ADC_INC_CALIBRATE_GPIO_Port GPIOC
#define DO_DRIVER_EN_Pin GPIO_PIN_1
#define DO_DRIVER_EN_GPIO_Port GPIOA
#define ADC_HALL_B_Pin GPIO_PIN_3
#define ADC_HALL_B_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define DI_DRIVER_PWRGD_Pin GPIO_PIN_4
#define DI_DRIVER_PWRGD_GPIO_Port GPIOC
#define DI_DRIVER_NOCTW_Pin GPIO_PIN_5
#define DI_DRIVER_NOCTW_GPIO_Port GPIOC
#define DI_DRIVER_NFAULT_Pin GPIO_PIN_0
#define DI_DRIVER_NFAULT_GPIO_Port GPIOB
#define ADC_USER_IN_Pin GPIO_PIN_1
#define ADC_USER_IN_GPIO_Port GPIOB
#define DI_ENABLE_PRINT_Pin GPIO_PIN_2
#define DI_ENABLE_PRINT_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_10
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_11
#define UART_RX_GPIO_Port GPIOB
#define ADC_HALL_A_Pin GPIO_PIN_12
#define ADC_HALL_A_GPIO_Port GPIOB
#define PWM_C_LS_Pin GPIO_PIN_6
#define PWM_C_LS_GPIO_Port GPIOC
#define PWM_C_HS_Pin GPIO_PIN_7
#define PWM_C_HS_GPIO_Port GPIOC
#define PWM_B_LS_Pin GPIO_PIN_8
#define PWM_B_LS_GPIO_Port GPIOC
#define PWM_B_HS_Pin GPIO_PIN_9
#define PWM_B_HS_GPIO_Port GPIOC
#define PWM_A_LS_Pin GPIO_PIN_8
#define PWM_A_LS_GPIO_Port GPIOA
#define PWM_A_HS_Pin GPIO_PIN_9
#define PWM_A_HS_GPIO_Port GPIOA
#define DO_SELECT_BRIDGE_DRIVER_Pin GPIO_PIN_15
#define DO_SELECT_BRIDGE_DRIVER_GPIO_Port GPIOA
#define DO_LED_1_Pin GPIO_PIN_10
#define DO_LED_1_GPIO_Port GPIOC
#define DO_DRIVER_DC_CAL_Pin GPIO_PIN_11
#define DO_DRIVER_DC_CAL_GPIO_Port GPIOC
#define DI_INC_B_Pin GPIO_PIN_12
#define DI_INC_B_GPIO_Port GPIOC
#define IR_INC_REF_Pin GPIO_PIN_2
#define IR_INC_REF_GPIO_Port GPIOD
#define IR_INC_REF_EXTI_IRQn EXTI2_TSC_IRQn
#define IR_INC_A_Pin GPIO_PIN_4
#define IR_INC_A_GPIO_Port GPIOB
#define IR_INC_A_EXTI_IRQn EXTI4_IRQn
#define DO_INC_POSITION_Pin GPIO_PIN_5
#define DO_INC_POSITION_GPIO_Port GPIOB
#define CNT_INC_A_Pin GPIO_PIN_6
#define CNT_INC_A_GPIO_Port GPIOB
#define DI_USER_IN_Pin GPIO_PIN_8
#define DI_USER_IN_GPIO_Port GPIOB
#define IR_COMP_C_Pin GPIO_PIN_9
#define IR_COMP_C_GPIO_Port GPIOB
#define IR_COMP_C_EXTI_IRQn EXTI9_5_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define PWM_A_LS_channel TIM_CHANNEL_1 // TIM1
#define PWM_A_HS_channel TIM_CHANNEL_2 // TIM1
#define PWM_B_LS_channel TIM_CHANNEL_3	// TIM8
#define PWM_B_HS_channel TIM_CHANNEL_4 // TIM8
#define PWM_C_LS_channel TIM_CHANNEL_1 // TIM8
#define PWM_C_HS_channel TIM_CHANNEL_2 // TIM8

#define DELAYED_CALLBACK_A_channel TIM_CHANNEL_1 // TIM2
#define DELAYED_CALLBACK_B_channel TIM_CHANNEL_2 // TIM2
#define DELAYED_CALLBACK_C_channel TIM_CHANNEL_3 // TIM2
#define DELAYED_CALLBACK_D_channel TIM_CHANNEL_4 // TIM2

#define DELAYED_CALLBACK_A_ir_flag TIM_FLAG_CC1 // TIM2
#define DELAYED_CALLBACK_B_ir_flag TIM_FLAG_CC2 // TIM2
#define DELAYED_CALLBACK_C_ir_flag TIM_FLAG_CC3 // TIM2
#define DELAYED_CALLBACK_D_ir_flag TIM_FLAG_CC4 // TIM2

#define DECODER_COUNT_channel TIM_CHANNEL_1 // TIM15
#define DECODER_COUNT_ir_flag TIM_FLAG_CC1 // TIM15
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
