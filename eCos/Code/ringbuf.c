#include "ringbuf.h"

void RingBuffer_WriteRegister(Register r)
{
	if(Rb.empty)
		Rb.empty = false;
	
	Rb.buffer[Rb.wi++] = r;
	
	if(Rb.nr < NRBUF)
		Rb.nr++;
		
	if(Rb.wi > NRBUF - 1)
		Rb.wi = 0;
}

Register RingBuffer_ReadRegister(void)
{
	Register result;
	
	result = Rb.buffer[Rb.ri++];
	
	if(Rb.ri > NRBUF - 1)
		Rb.ri = 0;
	
	return result;
}

void RingBuffer_DeleteRegisters()
{
	uint8_t i;
	
	for(i=0; i< NRBUF; i++)
	{
		Rb.buffer[i].h = 0;
		Rb.buffer[i].m = 0;
		Rb.buffer[i].s = 0;
		Rb.buffer[i].temp = 0;
		Rb.buffer[i].lum = 0;
	}
	
	Rb.empty = 1;
	Rb.ri = 0;
	Rb.wi = 0;
	Rb.nr = 0;
}














Register RingBuffer_ReadRegisterFromIndex(uint8_t index)
{
	uint8_t oldest_pointer;
	Register result;
	
	if(Rb.ri < NRBUF)
		oldest_pointer = 0;
	else
		oldest_pointer = Rb.wi;
	
	if(oldest_pointer > NRBUF - 1)
		oldest_pointer = 0;
		
	while(index > 0)
	{
		oldest_pointer++;
		if(oldest_pointer > NRBUF - 1)
		oldest_pointer = 0;
		index--;
	}
	
	result = Rb.buffer[oldest_pointer];
	
	return result;
}














