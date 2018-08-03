/*
 * ringbuffer.h
 *
 *  Created on: Dec 26, 2017
 *      Author: simon
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include <stdint.h>

#define BUFFER_OVERFLOW 0
#define BUFFER_SUCCESS 1
#define BUFFER_EMPTY 2

// source: https://stackoverflow.com/questions/246977/is-using-flexible-array-members-in-c-bad-practice
typedef struct
{
    uint16_t next_read;
    uint16_t next_write;
    uint16_t capacity;
    int32_t data[];
} Ringbuffer;

Ringbuffer * allocRingbuffer(uint32_t bufferSize);

uint8_t bufferIn(Ringbuffer *pBuffer, int32_t data);

uint8_t bufferOut(Ringbuffer *pBuffer, int32_t *pData);

void bufferReset(Ringbuffer *pBuffer);

#endif /* INC_RINGBUFFER_H_ */
