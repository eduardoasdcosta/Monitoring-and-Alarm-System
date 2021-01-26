#ifndef RINGBUF_H
#define RINGBUF_H

#include "defs.h"


typedef struct _register
{
    uint8_t h, m, s;
    uint8_t temp;
    uint8_t lum;
}Register;

typedef struct _ring_buffer
{
	uint8_t nr;
	uint8_t ri;
	uint8_t wi;
	bool empty;
	Register buffer[NRBUF];
}RingBuffer;

RingBuffer Rb;

Register RingBuffer_ReadRegisterFromIndex(uint8_t index);
Register RingBuffer_ReadRegister(void);
void RingBuffer_WriteRegister(Register r);
void RingBuffer_DeleteRegisters();
#endif
