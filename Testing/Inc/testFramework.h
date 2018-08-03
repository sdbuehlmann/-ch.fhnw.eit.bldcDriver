/*
 * testFramework.h
 *
 *  Created on: Aug 2, 2018
 *      Author: simon
 */

#ifndef INC_TESTFRAMEWORK_H_
#define INC_TESTFRAMEWORK_H_

#include <stdint.h>

#define ASSERT(check) (assert((check), __FILE__, __LINE__))

void assert(uint8_t isTrue, char file[], uint32_t line);
void logString(char pMsg[]);

#endif /* INC_TESTFRAMEWORK_H_ */
