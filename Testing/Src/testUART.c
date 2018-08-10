/*
 * testUART.c
 *
 *  Created on: Aug 3, 2018
 *      Author: simon
 */

// =============== Includes ============================================================================================================
#include "platformAPI.h"
#include "testFramework.h"
#include "tests.h"

// =============== Defines =============================================================================================================
#define NR_OF_SENDING_TESTTXT 11
// TX buffer has the size of 1000 bytes -> when both buffers are empty, 1100 bytes could be buffered

#define RX_TEST_TXT "Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magn"
#define RX_TEST_TXT_LENGTH 120

// =============== Typdefs =============================================================================================================

// =============== Variables ===========================================================================================================
static uint8_t testID = 0;

static uint8_t pReceivedData[300];
static uint8_t receivedDataCnt = 0;

uint8_t rxTestTxt[] = RX_TEST_TXT;
// =============== Function pointers ===================================================================================================

// =============== Function declarations ===============================================================================================

// =============== Functions ===========================================================================================================

// --------------- tests.h ----------------------------------------------------------------------------------------------------------
TestFeedback testUART() {
	switch (testID) {
	case 0:
		logString("Testing of UART functions...\n\r");
		testID++;
		return testFeedback_running;

	case 1: {
		// ******** test sending lot data ********************************************************************************
		logString(" TX test:\n\r");
		wait_ms(500); // wait for empty uart buffers
		// 100 chars. Source: http://www.loremipsum.de/
		uint8_t pLoremIpsum[] = "  Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt\n\r";

		uint32_t timeBeforSending_us = getSystimeUs();
		for (uint8_t cnt = 0; cnt < NR_OF_SENDING_TESTTXT; cnt++) {
			sendUartData(pLoremIpsum, 100);
		}
		uint32_t timeAfterSending_us = getSystimeUs();
		wait_ms(500); // wait for empty uart buffers
		logUnsigned("-> Delta time", 13, timeAfterSending_us - timeBeforSending_us, "us", 2);

		testID++;
		return testFeedback_running;
	}

	case 2:
		;// ******** test receiving data **********************************************************************************
		static uint32_t cnt;

		logString(" RX test:\n\r");
		logString("  Please send 8 chars: ");

		testID++;
		cnt++;
		return testFeedback_running;

	case 3:
		if (receivedDataCnt >= cnt*8) {
			logString("\n\r");

			if(cnt < 3){
				testID--;
			}else{
				testID++;
			}
		}
		return testFeedback_running;

	case 4:
		logString("Passed!\n\n\r");
		return testFeedback_finished;
	}

	return testFeedback_finished;
}
void testUART_event_uartDataReceived(uint8_t *pData, uint8_t nrData) {
	sendUartData(pData, nrData);

	for (uint8_t cnt = 0; cnt < nrData; cnt++) {
		pReceivedData[receivedDataCnt] = pData[cnt];
		receivedDataCnt++;
		//ASSERT(receivedDataCnt < 300);
	}

}

void testUART_platformError(char file[], uint32_t line) {
	logPlatformError(file, line);
}
