// Drivers to fill in the functions that mmc.c needs to work correctly
// Originally these were implemented by ChaN in assembly, but I wanted C for readability

#include <avr/io.h>


void xmit_spi(uint8_t)
{
	asm
	{
	ldi	r24, _BV(2)	
	.rept 16				;Toggle SCK 16 times
	out	_SFR_IO_ADDR(PINA), r24		;
	.endr					;/
	nop					;Read shift register
	in	r24, _SFR_IO_ADDR(USIDR)	;/
	}
}

uint8_t rcv_spi()
{


}

void fwd_blk_part(void*, uint16_t, uint16_t)
{



}
