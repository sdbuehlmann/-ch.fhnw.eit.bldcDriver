/*
 * utils.h
 *
 *  Created on: Jul 17, 2018
 *      Author: simon
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>

void bubblesort(uint32_t *array, uint32_t length);
void calculateMedianAndRange(uint32_t pValues[], uint32_t length, uint32_t *pMedian, uint32_t *pRange);

#endif /* INC_UTILS_H_ */
