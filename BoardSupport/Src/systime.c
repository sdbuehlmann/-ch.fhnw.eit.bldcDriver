/*
 * systime.c
 *
 *  Created on: Jul 30, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAdapter.h"
#include "platformAPI.h"
#include "platformModules.h"

// =============== Defines ============================================================================================================
#define INDEX_A 0
#define INDEX_B 1
#define INDEX_C 2
#define INDEX_D 3

// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================
static DelayedCallbackHandle available[4];

// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================
static void startChannelA(uint32_t timeToCall_us);
static void startChannelB(uint32_t timeToCall_us);
static void startChannelC(uint32_t timeToCall_us);
static void startChannelD(uint32_t timeToCall_us);

static void callback(uint8_t index);

static void resetHandle(DelayedCallbackHandle *pHandle);

static DelayedCallbackHandle * getNextWhoIsReady();

// =============== Functions ===========================================================================================================
static void startChannelA(uint32_t timeToCall_us) {
	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_A_channel,
			timeToCall_us);
}
static void startChannelB(uint32_t timeToCall_us) {
	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_B_channel,
			timeToCall_us);
}
static void startChannelC(uint32_t timeToCall_us) {
	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_C_channel,
			timeToCall_us);
}
static void startChannelD(uint32_t timeToCall_us) {
	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_D_channel,
			timeToCall_us);
}

static void callback(uint8_t index) {
	DelayedCallback functionToCall = available[index].callback;
	resetHandle(&available[index]);

	if (functionToCall != 0) {
		functionToCall();
	}
}

static void resetHandle(DelayedCallbackHandle *pHandle) {
	pHandle->callback = 0;
	pHandle->timeUntilCallback_us = 0;
	pHandle->timestampRegistered_us = 0;

	pHandle->status = delayedCallbackStatus_ready;
}

static DelayedCallbackHandle * getNextWhoIsReady() {
	entryNonInterruptableSection();

	for (uint8_t cnt = 0; cnt < 4; cnt++) {
		if (available[cnt].status == delayedCallbackStatus_ready) {
			available[cnt].status = delayedCallbackStatus_running;
			return &available[cnt];
		}
	}

	leaveNonInterruptableSection();

	return 0;
}

// --------------- platformModules.h -------------------------------------------------------------------------------------------------------
void initSystime() {
	//HAL_TIM_Base_Start(pSystemtime_Timer_handle);
	HAL_TIM_OC_Start_IT(pSystemtime_Timer_handle, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(pSystemtime_Timer_handle, TIM_CHANNEL_2);
	HAL_TIM_OC_Start_IT(pSystemtime_Timer_handle, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(pSystemtime_Timer_handle, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_A_channel,
			0);
	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_B_channel,
			0);
	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_C_channel,
			0);
	__HAL_TIM_SET_COMPARE(pSystemtime_Timer_handle, DELAYED_CALLBACK_D_channel,
			0);

	//DelayedCallbackHandle A;
	available[INDEX_A].start = &startChannelA;
	resetHandle(&available[INDEX_A]);

	available[INDEX_B].start = &startChannelB;
	resetHandle(&available[INDEX_B]);

	available[INDEX_C].start = &startChannelC;
	resetHandle(&available[INDEX_C]);

	available[INDEX_D].start = &startChannelD;
	resetHandle(&available[INDEX_D]);

	/*DelayedCallbackHandle B;
	 B.start = &startChannelB;
	 resetHandle(&B);
	 pAvailable[INDEX_B] = &B;

	 DelayedCallbackHandle C;
	 C.start = &startChannelC;
	 resetHandle(&C);
	 pAvailable[INDEX_C] = &C;

	 DelayedCallbackHandle D;
	 D.start = &startChannelD;
	 resetHandle(&D);
	 pAvailable[INDEX_D] = &D;*/
}

// --------------- platformAPI.h -------------------------------------------------------------------------------------------------------
uint32_t getSystimeUs() {
	return __HAL_TIM_GET_COUNTER(pSystemtime_Timer_handle);
}

DelayedCallbackFeedback startDelayedCallback(uint32_t timeUntilCallback_us,
		DelayedCallback callback_param) {
	uint32_t timestamp = getSystimeUs();

	DelayedCallbackHandle *pHandle = getNextWhoIsReady();
	if (pHandle == 0) {
		// no delayed callback in 'ready state'
		PLATFORM_ERROR;

		return delayedCallbackFeedback_error;
	}

	pHandle->callback = callback_param;
	pHandle->timeUntilCallback_us = timeUntilCallback_us;
	pHandle->timestampRegistered_us = timestamp;
	StartDelayedCallback start = pHandle->start;
	start(timestamp + timeUntilCallback_us);

	return delayedCallbackFeedback_registered;
}

void wait_ms(uint32_t ms) {
	HAL_Delay(ms);
}

// --------------- platformAdapter.h-------- -------------------------------------------------------------------------------------------
void timerIRQ_systime() {
	if (__HAL_TIM_GET_FLAG(pSystemtime_Timer_handle,
			DELAYED_CALLBACK_A_ir_flag)) {
		__HAL_TIM_CLEAR_FLAG(pSystemtime_Timer_handle,
				DELAYED_CALLBACK_A_ir_flag);
		callback(INDEX_A);
	}
	if (__HAL_TIM_GET_FLAG(pSystemtime_Timer_handle,
			DELAYED_CALLBACK_B_ir_flag)) {
		__HAL_TIM_CLEAR_FLAG(pSystemtime_Timer_handle,
				DELAYED_CALLBACK_B_ir_flag);
		callback(INDEX_B);
	}
	if (__HAL_TIM_GET_FLAG(pSystemtime_Timer_handle,
			DELAYED_CALLBACK_C_ir_flag)) {
		__HAL_TIM_CLEAR_FLAG(pSystemtime_Timer_handle,
				DELAYED_CALLBACK_C_ir_flag);
		callback(INDEX_C);
	}
	if (__HAL_TIM_GET_FLAG(pSystemtime_Timer_handle,
			DELAYED_CALLBACK_D_ir_flag)) {
		__HAL_TIM_CLEAR_FLAG(pSystemtime_Timer_handle,
				DELAYED_CALLBACK_D_ir_flag);
		callback(INDEX_D);
	}
}

