/*
 * testFramework.h
 *
 *  Created on: Aug 2, 2018
 *      Author: simon
 */

#ifndef INC_TESTFRAMEWORK_H_
#define INC_TESTFRAMEWORK_H_

#include <stdint.h>

#define ASSERT(check) 								(assert((check), __FILE__, __LINE__))
#define MUST_BE_CALLED(function, nrToBeCalled)		mustBeCalled(((Funcptr)function),(nrToBeCalled), __FILE__, __LINE__)
#define CALL(function)								call((Funcptr)function)

typedef void (*Funcptr)(void);
typedef struct{
	Funcptr function;
	uint32_t nrToBeCalled;
	uint32_t line;
	char *pFilename;
}ToBeCalled_Function;

void assert(uint8_t isTrue, char file[], uint32_t line);

void mustBeCalled(Funcptr function, uint32_t nrToBeCalled, char file[], uint32_t line);
void call(Funcptr function);
void checkMustBeCalled();

void stopTesting();

void logString(char pMsg[]);
void logUnsigned(char *pName, uint8_t nameLenght, uint32_t var, char *pUnit, uint8_t unitLenght);
void logSigned(char *pName, uint8_t nameLenght, int32_t var, char *pUnit, uint8_t unitLenght);
void logPlatformError(char file[], uint32_t line);

#endif /* INC_TESTFRAMEWORK_H_ */
