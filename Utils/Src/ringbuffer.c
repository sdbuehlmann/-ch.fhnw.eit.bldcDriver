/*
 * ringbuffer.c
 *
 *  Created on: Dec 26, 2017
 *      Author: simon
 */

#include <stdint.h>
#include <stdlib.h>

#include "ringbuffer.h"

Ringbuffer* allocRingbuffer(uint32_t bufferSize) {
	Ringbuffer *p;
	p = malloc(sizeof(*p) + bufferSize + 1); // alloc more space for the array (last position in struct)
	p->capacity = bufferSize;
	p->next_read = 0;
	p->next_write = 0;

	return p;
}

unsigned char bufferIn(Ringbuffer *pBuffer, int32_t data) {
	int after_next_write = (pBuffer->next_write + 1) % pBuffer->capacity;

	if (after_next_write == pBuffer->next_read) {
		return BUFFER_OVERFLOW;
	}

	// write data in array
	pBuffer->data[pBuffer->next_write] = data;

	// increase write pointer
	pBuffer->next_write = after_next_write;

	return BUFFER_SUCCESS;
}

uint8_t bufferOut(Ringbuffer *pBuffer, int32_t *pData) {
	if (pBuffer->next_write == pBuffer->next_read) {
		return BUFFER_EMPTY;
	}

	*pData = pBuffer->data[pBuffer->next_read];

	pBuffer->data[pBuffer->next_read] = 0;

	pBuffer->next_read = (pBuffer->next_read + 1) % pBuffer->capacity;

	return BUFFER_SUCCESS;
}

void bufferReset(Ringbuffer *pBuffer){
	pBuffer->next_read = 0;
	pBuffer->next_write = 0;
}
