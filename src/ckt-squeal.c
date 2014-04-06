#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "pff.h"


#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

EMPTY_INTERRUPT(PCINT_vect);

/*---------------------------------------------------------*/
/* Work Area                                               */
/*---------------------------------------------------------*/

volatile uint8_t FifoRi, FifoWi, FifoCt;	/* FIFO controls */
uint8_t Buff[256];		/* Audio output FIFO */


FATFS Fs;			/* File system object */
DIR Dir;			/* Directory object */
FILINFO Fno;		/* File information */


/*---------------------------------------------------------*/
/* Sub-routines                                            */
/*---------------------------------------------------------*/

static
void ramp (		/* Ramp-up/down audio output (anti-pop feature) */
	int dir		/* 0:Ramp-down, 1:Ramp-up */
)
{
	uint16_t v, d, n;


	if (dir) {
		v = 0; d = 1;
	} else {
		v = 128; d = 0xFF;
	}

	n = 128;
	do {
		v += d;
		OCR1A = v; OCR1B = v;
		_delay_us(100);
	} while (--n);
}

volatile uint8_t ticks250kHz = 0;
volatile uint8_t ticks1kHz = 0;
volatile uint8_t doInputSample = 0;

static void audio_on (void)	/* Enable audio output functions */
{
	if (!TCCR0B) {
		FifoCt = 0; FifoRi = 0; FifoWi = 0;		/* Reset audio FIFO */
		PLLCSR = 0b00000110;	/* Select PLL clock for TC1.ck */
		TCCR1A = 0b10100011;	/* Start TC1 with OC1A/OC1B PWM enabled */
		TCCR1B = 0b00000001;
		ramp(1);				/* Ramp-up to center level */
		TCCR0A = 0b00000001;	/* Enable TC0.ck = 2MHz as interval timer */
		TCCR0B = 0b00000010;
		TIMSK = _BV(OCIE0A);
	}
}

static uint32_t load_header (void)	/* 2:I/O error, 4:Invalid file, >=1024:Ok(number of samples) */
{
	uint32_t sz, f;
	uint8_t b, al = 0;
	uint16_t rb;			/* Return value. Put this here to avoid avr-gcc's bug */

	/* Check RIFF-WAVE file header */
	if (pf_read(Buff, 12, &rb)) return 2;
	if (rb != 12 || LD_DWORD(Buff+8) != FCC('W','A','V','E')) return 4;

	while (1)
	{
		if (pf_read(Buff, 8, &rb)) return 2;		/* Get Chunk ID and size */
		if (rb != 8) return 4;
		sz = LD_DWORD(&Buff[4]);		/* Chunk size */

		/* Switch by chunk type */
		switch (LD_DWORD(&Buff[0])) 
		{
			case FCC('f','m','t',' ') :		/* 'fmt ' chunk */
				if (sz & 1) sz++;
				if (sz > 128 || sz < 16) return 4;		/* Check chunk size */
				if (pf_read(Buff, sz, &rb)) return 2;	/* Get the chunk content */
				if (rb != sz) return 4;
				if (Buff[0] != 1) return 4;				/* Check coding type (1: LPCM) */
				b = Buff[2];
				if (b < 1 && b > 2) return 4; 			/* Check channels (1/2: Mono/Stereo) */
				GPIOR0 = al = b;						/* Save channel flag */
				b = Buff[14];
				if (b != 8 && b != 16) return 4;		/* Check resolution (8/16 bit) */
				GPIOR0 |= b;							/* Save resolution flag */
				if (b & 16) al <<= 1;
				f = LD_DWORD(&Buff[4]);					/* Check sampling freqency (8k-48k) */
				if (f < 8000 || f > 48000) return 4;
				OCR0A = (uint8_t)(16000000UL/8/f) - 1;		/* Set interval timer (sampling period) */
				break;

			case FCC('d','a','t','a') :		/* 'data' chunk (start to play) */
				if (!al) return 4;							/* Check if format valid */
				if (sz < 1024 || (sz & (al - 1))) return 4;	/* Check size */
				if (Fs.fptr & (al - 1)) return 4;			/* Check offset */
				return sz;

			case FCC('D','I','S','P') :		/* 'DISP' chunk (skip) */
			case FCC('f','a','c','t') :		/* 'fact' chunk (skip) */
			case FCC('L','I','S','T') :		/* 'LIST' chunk (skip) */
				if (sz & 1) sz++;
				if (pf_lseek(Fs.fptr + sz)) return 2;
				break;

			default :						/* Unknown chunk */
				return 4;
		}
	}
}

static inline void playIndicatorOn()
{
	PORTA |= _BV(PA3);
}

static inline void playIndicatorOff()
{
	PORTA &= ~_BV(PA3);
}

uint8_t io_input = 0;
uint8_t randomizer = 0;

uint8_t debounce_inputs()
{
	uint8_t io_rawInput = (PINA & 0xF0) >> 4;
	uint8_t delta = io_rawInput ^ io_input;
	uint8_t changes;
	static uint8_t clock_A=0, clock_B=0;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;
	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.
	changes = ~((~delta) | clock_A | clock_B);
	io_input ^= changes;
	randomizer += 1 + clock_A;
	return(changes);
}

uint8_t levelTriggerMask = 0;

uint8_t terminationCallback()
{
		debounce_inputs();
		return (levelTriggerMask & (io_input))?1:0;
}

static uint8_t play(uint8_t (*terminationCallback)())
{
	uint32_t sz;
	FRESULT res;
	uint16_t btr;
	uint16_t rb;

	res = pf_open((char*)Buff);		/* Open sound file */	
	FifoCt = 0; FifoRi = 0; FifoWi = 0;	/* Reset audio FIFO */

	if (FR_OK == res)
	{
		sz = load_header();			/* Check file format and ready to play */
		if (sz < 1024) return 255;	/* Cannot play this file */

		playIndicatorOn();

		pf_read(0, 512 - (Fs.fptr % 512), &rb);	/* Snip sector unaligned part */
		sz -= rb;
		do {	/* Data transfer loop */
			wdt_reset();
			if (NULL != terminationCallback)
			{
				if (terminationCallback())
				{
					FifoCt = 2;
					break;
				}
			}
			btr = (sz > 1024) ? 1024 : (WORD)sz;/* A chunk of audio data */
			res = pf_read(0, btr, &rb);	/* Forward the data into audio FIFO */
			if (rb != 1024)
				break;		/* Break on error or end of data */
			sz -= rb;					/* Decrease data counter */
		} while (1);

	
	}

	while (FifoCt) ;			/* Wait for audio FIFO empty */
	OCR1A = 0x80; OCR1B = 0x80;	/* Return DAC out to center */
	playIndicatorOff();
	return res;
}



uint8_t getFilenum(uint8_t eventNum, uint8_t fileNum)
{
	uint8_t fileCount = 0, i;
	FRESULT res;
	Buff[0] = 0;
	strcpy_P((char*)Buff, PSTR("eventX"));
	Buff[5] = eventNum + '1';

	if (FR_OK != pf_opendir(&Dir, (const char*)Buff))
		return 1;

	if (FR_OK != pf_readdir(&Dir, 0))
		return 1;
	
	do
	{
		res = pf_readdir(&Dir, &Fno);
		if (res || !Fno.fname[0])
			break;	// Break on error or end of dir
		if (!(Fno.fattrib & (AM_DIR|AM_HID)) && strstr(Fno.fname, ".WAV"))
		{
			if (fileCount == fileNum)
			{
				Buff[6] = '/';
				Buff[7] = 0;
				strcat((char*)Buff, Fno.fname);
				return 0;
			}
			fileCount++;
		}
	} while (FR_OK == res);	
	
	return 1;			
		
}

/*-----------------------------------------------------------------------*/
/* Main                                                                  */

int main (void)
{
	FRESULT res;
	char *dir;
	uint8_t changes, i;
	uint8_t fsActive = 0;
	uint8_t filesInDirs[4];

	
	MCUSR = 0;								/* Clear reset status */
//	WDTCR = _BV(WDE) | 0b110;			/* Enable WDT (1s) */
	WDTCR = 0b110;			/* Enable WDT (1s) */
	PCMSK0 = 0b11111000;					/* Select pin change interrupt pins (SW1..SW8) */
	PCMSK1 = 0b01110000;

	/* Initialize ports */
	PORTA = 0b11110011;		/* PORTA [pppppLHp]*/
	DDRA  = 0b00001110;
	PORTB = 0b00000001;		/* PORTB [-pppLLLH] */
	DDRB  = 0b00011111;

	sei();
	audio_on();		/* Enable audio output */

	while(1)
	{
		wdt_reset();
		randomizer++;

		if (!fsActive)
		{
			PORTB |= ~_BV(PB4);
			if (FR_OK != pf_mount(&Fs))
				continue;
			
			for (i=0; i<3; i++)
			{
				filesInDirs[i] = 0;
				memset(Buff, 0, sizeof(Buff));
				strcpy_P((char*)Buff, PSTR("eventX"));
				Buff[5] = i + '1';
								
				if (FR_OK != pf_opendir(&Dir, Buff))
					continue;

				if (FR_OK != pf_readdir(&Dir, 0))
					continue;
				
				do
				{
					if (FR_OK == (res = pf_readdir(&Dir, &Fno)))
					{
						if (!Fno.fname[0])
							break;	// Break on error or end of dir
						if (!(Fno.fattrib & (AM_DIR|AM_HID)) && strstr(Fno.fname, ".WAV"))
							filesInDirs[i]++;
					}
				
				} while (FR_OK == res);
			}
			fsActive = 1;
			PORTB |= _BV(PB4);
		}
		
		// If we can't get the filesystem, then it doesn't matter - we can't play anything anyway
		// Go back around and try again
		if (!fsActive)
			continue;
	
	
		if (FR_OK != pf_opendir(&Dir, ""))
		{
			fsActive = 0;
			continue;
		}
	
		if (1)
		{
			changes = debounce_inputs();
			doInputSample = 0;
		}


		if (changes & ~(io_input))
		{
			uint8_t buttonPushed = changes & (~io_input);

			for(i=0; i<3; i++)
			{
				if (buttonPushed & (1<<i))
				{
					levelTriggerMask = buttonPushed & (1<<i);
					if (0 == getFilenum(i, randomizer % filesInDirs[i]))
						play(&terminationCallback);
					break;
				}
			}
		}
		
		
	}
}

