/*
 * utils.c
 *
 *  Created on: Jul 17, 2018
 *      Author: simon
 */

#include "utils.h"

// source: https://de.wikibooks.org/wiki/Algorithmen_und_Datenstrukturen_in_C/_Bubblesort
void bubblesort(uint32_t *array, uint32_t length) {
	uint32_t i, j, tmp;

	for (i = 1; i < length; i++) {
		for (j = 0; j < length - i; j++) {
			if (array[j] > array[j + 1]) {
				tmp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = tmp;
			}
		}
	}
}

void calculateMedianAndRange(uint32_t pValues[], uint32_t length, uint32_t *pMedian, uint32_t *pRange){
	bubblesort(pValues, length);

	*pMedian = pValues[length/2];
	*pRange = pValues[length - 1] - pValues[0];
}
