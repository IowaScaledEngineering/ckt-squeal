/*************************************************************************
Title:    CKT-SQUEAL audio player, v1.1
Authors:  Nathan D. Holmes <maverick@drgw.net>
          Portions copyright ChaN (http://elm-chan.org) and used under 
           his/her free software terms
File:     $Id: $
License:  GNU General Public License v3

CREDIT:
    The basic idea behind this playback design, and much of the hard stuff -
    the filesystem layer, the hand-tuned ASM routines - were borrowed
    from an open source project by ChaN called the SD20P.  
    http://elm-chan.org/works/sd20p/report.html
    
    I ran across ChaN's design for a cheap, simple audio player when I was 
    working on much more complicated prototypes for the CKT-SQUEAL, and
    his/her attiny861 design was sheer beauty in its simplicity.  Whoever ChaN
    is, their work is both clever and amazing.
    
    To respect his terms, here is the license statement included at the top
    of his source code:
    
    /  Copyright (C) 2013, ChaN, all right reserved.
    / * This project is a free software and there is NO WARRANTY.
    / * No restriction on use. You can use, modify and redistribute it for
    /   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
    / * Redistributions of source code must retain the above copyright notice.
    
    The filesystem module - PetitFS - is also some of ChaN's work, and is available
    under an open license here:
    
    http://elm-chan.org/fsw/ff/00index_p.html

LICENSE:
    Copyright (C) 2014 Michael Petersen, Nathan Holmes, with portions
    copyright 2013 ChaN (http://elm-chan.org)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "pff.h"


#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

#define RED_LED    4
#define YELLOW_LED 5
#define GREEN_LED  6

#define LED_BLINK_MOD 128

EMPTY_INTERRUPT(PCINT_vect);

/*---------------------------------------------------------*/
/* Work Area                                               */
/*---------------------------------------------------------*/

volatile uint8_t FifoRi, FifoWi, FifoCt;	/* FIFO controls */
uint8_t audioFifoBuffer[256];		/* Audio output FIFO */
uint8_t eventWavFiles[4] = {0,0,0,0};
uint8_t eventTriggerOptions[4] = {0,0,0,0};

FATFS Fs;			/* File system object */
DIR Dir;			/* Directory object */
FILINFO Fno;		/* File information */

/*
void saveFS(FSSAVE* fssave)
{
	fssave->fptr = Fs.fptr;
	fssave->org_clust = Fs.org_clust;
	fssave->curr_clust = Fs.curr_clust;
	fssave->fsize = Fs.fsize;
}

void loadFS(FSSAVE* fssave)
{
	Fs.fptr = fssave->fptr;
	Fs.org_clust = fssave->org_clust;
	Fs.curr_clust = fssave->curr_clust;
	Fs.fsize = fssave->fsize;
}*/

void setLed(uint8_t whichLed)
{
	PORTB |= (1<<whichLed) & 0x70;
}

void clearLed(uint8_t whichLed)
{
	PORTB &= ~((1<<whichLed) & 0x70);
}


// Ramp-up/down audio output (anti-pop feature)
// dir = 0:Ramp-down, 1:Ramp-up

static void ramp (uint8_t dir)
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

void enableOutputAmplifier()
{
	PORTB &= ~_BV(PB2);
}

void disableOutputAmplifier()
{
	PORTB |= _BV(PB2);
}


void enableAudio(void)	/* Enable audio output functions */
{
	wdt_reset();
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
	_delay_ms(200);	
	enableOutputAmplifier();
}

void disableAudio(void)	/* Disable audio output functions */
{
	disableOutputAmplifier();
	wdt_reset();
	_delay_ms(100);
	if (TCCR0B) {
		TCCR0B = 0;				/* Stop audio timer */
		ramp(0);				/* Ramp-down to GND level */
		TCCR1A = 0;	TCCR1B = 0;	/* Stop PWM */
		OCR1A = OCR1B = 0x80;	/* Return DAC out to center */
	}
	
	
}
static uint32_t load_header (void)	/* 2:I/O error, 4:Invalid file, >=1024:Ok(number of samples) */
{
	uint32_t sz, f;
	uint8_t b, al = 0;
	uint16_t rb;			/* Return value. Put this here to avoid avr-gcc's bug */

	/* Check RIFF-WAVE file header */
	if (pf_read(audioFifoBuffer, 12, &rb)) return 2;
	if (rb != 12 || LD_DWORD(audioFifoBuffer+8) != FCC('W','A','V','E')) return 4;

	while (1)
	{
		if (pf_read(audioFifoBuffer, 8, &rb)) 
			return 2;		/* Get Chunk ID and size */

		if (rb != 8) 
			return 4;

		sz = LD_DWORD(audioFifoBuffer + 4);		/* Chunk size */

		/* Switch by chunk type */
		switch (LD_DWORD(audioFifoBuffer + 0)) 
		{
			case FCC('f','m','t',' ') :		/* 'fmt ' chunk */
				if (sz & 1) sz++;
				if (sz > 128 || sz < 16) return 4;		/* Check chunk size */
				if (pf_read(audioFifoBuffer, sz, &rb)) return 2;	/* Get the chunk content */
				if (rb != sz) return 4;
				if (audioFifoBuffer[0] != 1) return 4;				/* Check coding type (1: LPCM) */
				b = audioFifoBuffer[2];
				if (b < 1 && b > 2) return 4; 			/* Check channels (1/2: Mono/Stereo) */
				GPIOR0 = al = b;						/* Save channel flag */
				b = audioFifoBuffer[14];
				if (b != 8 && b != 16) return 4;		/* Check resolution (8/16 bit) */
				GPIOR0 |= b;							/* Save resolution flag */
				if (b & 16) al <<= 1;
				f = LD_DWORD(audioFifoBuffer + 4);					/* Check sampling freqency (8k-48k) */
				if (f < 8000 || f > 48000) return 4;
				OCR0A = (uint8_t)(16000000UL/8/f) - 1;		/* Set interval timer (sampling period) */
				break;

			case FCC('d','a','t','a') :		/* 'data' chunk (start to play) */
				if (!al) return 4;							/* Check if format valid */
				if (sz < 1024 || (sz & (al - 1))) return 4;	/* Check size */
				if (Fs.fptr & (al - 1)) return 4;			/* Check offset */
				return sz;

//			case FCC('D','I','S','P') :		/* 'DISP' chunk (skip) */
//			case FCC('f','a','c','t') :		/* 'fact' chunk (skip) */
//			case FCC('L','I','S','T') :		/* 'LIST' chunk (skip) */
			default :						// All chunks we don't understand or care about
				if (sz & 1) sz++;
				if (pf_lseek(Fs.fptr + sz)) return 2;
				break;
		}
	}
}

uint8_t io_input = 0xFF;
uint8_t last_io_input = 0xFF;	
uint8_t randomizer = 0;
uint8_t levelTriggerMask = 0;

uint8_t debounce_inputs()
{
	uint8_t delta = ((PINA & 0xF0) >> 4) ^ io_input;
	uint8_t changes;
	static uint8_t clock_A=0, clock_B=0;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;
	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.
	changes = ~((~delta) | clock_A | clock_B);
	io_input ^= changes;
	randomizer += 1;
	return(changes);
}

#define EVENT_TRIGGER_LEVEL      0x01
#define EVENT_RETRIGGERABLE      0x02
#define EVENT_RANDOM             0x04
#define EVENT_RANDOM_RETRIG      0x06
#define EVENT_BEGIN_MIDDLE_END   0x08
#define EVENT_ENABLE_AUDIO       0x10
uint8_t terminationCallback()
{
		debounce_inputs();
		return (levelTriggerMask & (io_input))?1:0;
}

uint8_t debouncingCallback()
{
		debounce_inputs();
		return 0;
}

uint32_t fsave, szsave;

void playInner(uint8_t (*terminationCallback)(), uint8_t flags, uint32_t sz)
{
	uint8_t blinkCntr = 0;
	uint16_t btr;
	uint16_t rb;
		
	FifoCt = FifoRi = FifoWi = 0;  // Reset audio FIFO 

	fsave = Fs.fptr;
	szsave = sz;

	if (flags & EVENT_ENABLE_AUDIO)
		enableAudio();
	else
		TCCR0B = 0b00000010;

	do
	{
		if (sz < 1024)
		{
			disableAudio();			
			clearLed(GREEN_LED);
			setLed(YELLOW_LED);
			return;
		}

		setLed(GREEN_LED);
		pf_read(0, 512 - (Fs.fptr % 512), &rb);	/* Snip sector unaligned part */
		sz -= rb;

		// Main playback loop
		do 
		{
			// This loop runs for the entire length of playback - keep watchdog happy
			wdt_reset();
			randomizer += (TCNT0L + TCNT1) ^ blinkCntr;
			// If there's a termination callback, go test it.  A true value will kick us out of playback
			if ((NULL != terminationCallback) && terminationCallback())
			{
				FifoCt = 0;  // Reset audio FIFO 
				flags &= ~EVENT_RETRIGGERABLE;
				break;
			}

			btr = (sz > 1024) ? 1024 : (WORD)sz;/* A chunk of audio data */
			pf_read(0, btr, &rb);	/* Forward the data into audio FIFO */

			if (0 == (++blinkCntr % LED_BLINK_MOD))
				PORTB ^= _BV(YELLOW_LED);

			if (rb != 1024)
				break;		/* Break on error or end of data */
			sz -= rb;					/* Decrease data counter */
		} while (sz > 0);
		
		if (flags & EVENT_RETRIGGERABLE)
		{
			pf_lseek(fsave);
			sz = szsave;
		}
		
	} while (flags & EVENT_RETRIGGERABLE);

	TCCR0B = 0;				/* Stop audio timer */

	setLed(YELLOW_LED);
}

static void play(uint8_t (*terminationCallback)(), uint8_t flags)
{
	FifoCt = FifoRi = FifoWi = 0;  // Reset audio FIFO 

	if (FR_OK == pf_open((char*)audioFifoBuffer))
	{
		uint32_t sz = load_header();
		if (sz < 1024)
		{
			setLed(RED_LED);
			_delay_ms(200);
			clearLed(RED_LED);
		} else {
			playInner(terminationCallback, flags | EVENT_ENABLE_AUDIO, sz);
			disableAudio();
			clearLed(GREEN_LED);
		}
	}
	
	return;
}

#define FILE_BEGIN  1
#define FILE_MIDDLE 2
#define FILE_END    3

uint8_t igetFile(uint8_t eventNum)
{
	strcpy_P((char*)audioFifoBuffer, PSTR("eventX"));
	audioFifoBuffer[5] = eventNum + '1';

	if (FR_OK != pf_opendir(&Dir, (const char*)audioFifoBuffer))
		return 1;

	if (FR_OK != pf_readdir(&Dir, 0))
		return 1;

	return 0;
}

uint8_t getBME(uint8_t eventNum, uint8_t bme)
{
	FRESULT res;

	if (0 != igetFile(eventNum))
		return 1;
		
	do
	{
		res = pf_readdir(&Dir, &Fno);
		if (res || !Fno.fname[0])
			break;	// Break on error or end of dir
		if (!(Fno.fattrib & (AM_DIR|AM_HID)))
		{
			switch(bme)
			{
				case FILE_BEGIN:
					eventNum = (uint8_t)(uint16_t)strcmp_P(Fno.fname, PSTR("BEGIN.WAV"));
					break;
				case FILE_MIDDLE:
					eventNum = (uint8_t)(uint16_t)strcmp_P(Fno.fname, PSTR("MIDDLE.WAV"));
					break;
				case FILE_END:
					eventNum = (uint8_t)(uint16_t)strcmp_P(Fno.fname, PSTR("END.WAV"));
					break;
			}

			if (0 == eventNum)
			{
				audioFifoBuffer[6] = '/';
				audioFifoBuffer[7] = 0;
				strcat((char*)audioFifoBuffer, Fno.fname);
				return 0;
			}
		}
	} while (FR_OK == res);			

	return 1;
}


uint8_t getFilenum(uint8_t eventNum, uint8_t fileNum)
{
	uint8_t fileCount = 0;
	uint8_t res;
	strcpy_P((char*)audioFifoBuffer, PSTR("eventX"));
	audioFifoBuffer[5] = eventNum + '1';

	if (0 != igetFile(eventNum))
		return 1;
	
	do
	{
		res = pf_readdir(&Dir, &Fno);
		if (res || !Fno.fname[0])
			break;	// Break on error or end of dir
		if (!(Fno.fattrib & (AM_DIR|AM_HID)) && strstr(Fno.fname, ".WAV") && ('.' != Fno.fname[0]))
		{
			if (fileCount == fileNum)
			{
				audioFifoBuffer[6] = '/';
				audioFifoBuffer[7] = 0;
				strcat((char*)audioFifoBuffer, Fno.fname);
				return 0;
			}
			fileCount++;
		}
	} while (FR_OK == res);	
	
	return 1;			
		
}

inline uint8_t isCardInserted()
{
	return (!(PINA & _BV(3)));
}

void blinkRedLed()
{
	setLed(RED_LED);
	_delay_ms(100);
	clearLed(RED_LED);
	_delay_ms(100);
}

inline uint8_t readConfigAndFiles()
{
	uint8_t i;
	for (i=0; i<4; i++)
	{
		eventWavFiles[i] = 0;
		eventTriggerOptions[i] = 0;
		strcpy_P((char*)audioFifoBuffer, PSTR("eventX"));
		audioFifoBuffer[5] = i + '1';
						
		if (FR_OK != pf_opendir(&Dir, (const char*)audioFifoBuffer) || FR_OK != pf_readdir(&Dir, 0))
			continue;

		do
		{
			if (FR_OK != pf_readdir(&Dir, &Fno))
				break;
			if (!Fno.fname[0])
				break;	// Break on error or end of dir
			if (!(Fno.fattrib & (AM_DIR|AM_HID)))
			{
				if(strstr_P(Fno.fname, PSTR(".WAV")) && '.' != Fno.fname[0])
				{
					eventWavFiles[i]++;
				}
				else
				{
					if (0 == strcmp_P(Fno.fname, PSTR("BME.OPT")))
					{
						// Beginning-middle-end mode precludes all other options
						eventTriggerOptions[i] |= EVENT_BEGIN_MIDDLE_END;
					}
					else
					{
						if (0 == strcmp_P(Fno.fname, PSTR("LVLTRIG.OPT")))
							eventTriggerOptions[i] |= EVENT_TRIGGER_LEVEL;
						if (0 == strcmp_P(Fno.fname, PSTR("RETRIG.OPT")))
							eventTriggerOptions[i] |= EVENT_RETRIGGERABLE;
						if (0 == strcmp_P(Fno.fname, PSTR("RNDRTRG.OPT")))
							eventTriggerOptions[i] |= EVENT_RANDOM_RETRIG;
					}
				}
			}
		
		} while (1);
	}
	return 1;
}

void saveFS(uint16_t eepromBase, uint32_t sz)
{
	eeprom_update_dword((uint32_t*)eepromBase, Fs.fptr);
	eeprom_update_dword((uint32_t*)(eepromBase+4), Fs.fsize);
	eeprom_update_dword((uint32_t*)(eepromBase+8), Fs.org_clust);
	eeprom_update_dword((uint32_t*)(eepromBase+12), Fs.curr_clust);	
	eeprom_update_dword((uint32_t*)(eepromBase+16), sz);
}

uint32_t loadFS(uint16_t eepromBase)
{
	Fs.fptr = eeprom_read_dword((uint32_t*)eepromBase);
	Fs.fsize = eeprom_read_dword((uint32_t*)(eepromBase + 4));
	Fs.org_clust = eeprom_read_dword((uint32_t*)(eepromBase + 8));
	Fs.curr_clust = eeprom_read_dword((uint32_t*)(eepromBase + 12));	
	pf_lseek(Fs.fptr);
	return eeprom_read_dword((uint32_t*)(eepromBase + 16));
}

int main (void)
{
	uint8_t i;
	uint8_t fsActive = 0;
	
	MCUSR = 0;								// Clear reset status
	wdt_reset();
//	wdt_disable();
	WDTCR = _BV(WDE) | _BV(WDP3);		// Enable WDT (2s)

	// Initialize ports 
	// Pin Assignments for PORTA/DDRA
	//  PA0 - SD Card Out (AVR input)
	//  PA1 - SD Card In (AVR output) - must initialize low
	//  PA2 - SD Card Clk (AVR output) - must initialize high
	//  PA3 - SD Card detect (AVR input, needs pullup on)
	//  PA4 - SW1 input (AVR input, needs pullup on)
	//  PA5 - SW2 input (AVR input, needs pullup on)
	//  PA6 - SW3 input (AVR input, needs pullup on)
	//  PA7 - SW4 input (AVR input, needs pullup on)
	PORTA = 0b11111011;		/* PORTA [pppppLHp]*/
	DDRA  = 0b00000110;
	
	//  PB0 - SD Card CS (AVR output) - must initialize high
	//  PB1 - Audio PWM LSB (AVR output)
	//  PB2 - LM4864 shutdown (AVR output, shutdown = high)
	//  PB3 - Audio PWM MSB (AVR output)
	//  PB4 - Red LED (AVR output, active high)
	//  PB5 - Yellow LED (AVR output, active high)
	//  PB6 - Green LED (AVR output, active high)
   //  PB7 - N/A, multiplexed with /RESET pin	
	
	PORTB = 0b00000101;		/* PORTB [-pppLLLH] */
	DDRB  = 0b01111111;

	setLed(RED_LED); // Initialize to the RED led on, indicating error
	sei();

	while(1)
	{
		wdt_reset();

		// If the card isn't there, don't really do much
		// Blink the red light and bump back to the top
		if (!isCardInserted())
		{
			clearLed(YELLOW_LED);
			disableAudio();
			fsActive = 0;
			blinkRedLed();
			continue;
		}

		// Now check if the filesystem is up and running
		// If it's not, start it up
		if (!fsActive)
		{
			setLed(RED_LED); // Set error indicator
			clearLed(YELLOW_LED);  // Clear indication that the FS is up and running

			// Test that we can mount the FS okay
			// If not, bail and go back around the main loop.  We haven't adjusted the fsActive or
			// anything yet.
			if (FR_OK == pf_mount(&Fs))
			{
				setLed(YELLOW_LED);
				fsActive = readConfigAndFiles();
			}
			
		}

		// If we can't get the filesystem, then it doesn't matter - we can't play anything anyway
		// Go back around and try again
		if (!fsActive)
		{
			clearLed(YELLOW_LED);
			continue;
		}
		
		// Okay, at this point the FS is hot and we're ready to go
		// Turn off any error indication that might be on
		clearLed(RED_LED);

//		_delay_ms(10);

		debounce_inputs();

		randomizer++;

		for(i=0; i<4 && isCardInserted(); i++)
		{
			uint8_t isDown = (~io_input) & (1<<i);
			// If there aren't any WAV files for this event, don't even consider it for playback
			if (0 == eventWavFiles[i])
				continue;

			levelTriggerMask = 0;
			
			if ((eventTriggerOptions[i] & EVENT_BEGIN_MIDDLE_END) && isDown)
			{
				if (0 == getBME(i, FILE_MIDDLE))
				{
					if (FR_OK == pf_open((char*)audioFifoBuffer))
					{
						szsave = load_header();
						saveFS(0x10, szsave);
					}
				}

				if (0 == getBME(i, FILE_END))
				{
					if(FR_OK == pf_open((char*)audioFifoBuffer))
					{
						szsave = load_header();
						saveFS(0x40, szsave);
					}
				}
				// Level triggered sound and line is grounded - play

				if (0 == getBME(i, FILE_BEGIN))
				{

					if (FR_OK == pf_open((char*)audioFifoBuffer))
					{
						playInner(&debouncingCallback, EVENT_ENABLE_AUDIO, load_header());

						// Only play the middle if we have a beginning

						while((szsave = loadFS(0x10)) && (~io_input) & (1<<i))
						{
							// Loop playback over middle audio until the pin rises
							playInner(&debouncingCallback, 0, szsave);
							debounce_inputs();
						}

						if (0 != (szsave = loadFS(0x40)))
						{
							playInner(&debouncingCallback, 0, szsave);
						}
						disableAudio();
						clearLed(GREEN_LED); 
					}
				}
			} else if ((eventTriggerOptions[i] & EVENT_TRIGGER_LEVEL) && isDown) {
				// Level triggered sound and line is grounded - play
				uint8_t chosenFile = randomizer % eventWavFiles[i];

				while (0 == getFilenum(i, chosenFile) && ((~io_input) & (1<<i)))
				{
					levelTriggerMask = (1<<i);
					// If it's level triggered but NOT RANDOM retrigger, we play the same track over and over again
					// Otherwise, we play a new track every
					play(&terminationCallback, (eventTriggerOptions[i] & EVENT_RANDOM)?0:EVENT_RETRIGGERABLE);
					// If it's not retriggerable, break
					if (0 == (EVENT_RETRIGGERABLE & eventTriggerOptions[i]))
						break;
					// If it's random, pick a new file
					if (EVENT_RANDOM & eventTriggerOptions[i])
						chosenFile = randomizer % eventWavFiles[i];
					debounce_inputs();
				}
				
			}
			else if (isDown & last_io_input)
			{
				// Edge triggered
				// Only trigger if the line fell since last time
				uint8_t chosenFile = randomizer % eventWavFiles[i];

				while ((0 == getFilenum(i, chosenFile)) && ((~io_input) & (1<<i)))
				{
					play(&debouncingCallback, 0);

					// If it's not retriggerable, break
					if (0 == (EVENT_RETRIGGERABLE & eventTriggerOptions[i]))
						break;
					// If it's random, pick a new file
					if (EVENT_RANDOM & eventTriggerOptions[i])
						chosenFile = randomizer % eventWavFiles[i];
					debounce_inputs();
				}
			}
		}

		last_io_input = io_input;
	}
}

