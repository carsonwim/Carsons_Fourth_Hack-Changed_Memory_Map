/*****************************************************************************
*
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*SOCKOPT_RECV_TIMEOUT
*****************************************************************************/
#include "cc3000.h"
#include "msp430fr5739.h"
#include "wlan.h"
#include "evnt_handler.h"    // callback function declaration
#include "socket.h"
#include "common.h"
#include "netapp.h"
#include "board.h"
#include "common.h"
#include "string.h"
#include "device_config.h"
#include "board.h"
#include "strlib.h"
#include "server.h"
#include "server_setup.h"


unsigned char * buffer_wiper_ptr;
int k;
int ammount_of_samples = (int)AMMOUNT_OF_SAMPLES_IN_PACKET;
unsigned char *rep_buffer0_ptr;
unsigned char *rep_buffer1_ptr;
char rep_packet_header_size;


//******************************************************************************
//This function refreshes the ADC registers and pins. It can be used before the
// ADC is setup. All that is left after this is to turn on the ADC, start and enable,
// and select which channel will be sampled.
//
//ADC10MCTL0 |= ADC10INCH_13;
//ADC10CTL0 |= (ADC10ON + ADC10ENC + ADC10SC);
//******************************************************************************
void refresh_ADC(void){
	//Start with ADC:
	//Stop ADC: Turn off, and stop conversion.
	ADC10CTL0 &= ~(ADC10ON + ADC10ENC + ADC10SC);
	__delay_cycles(1000);
	//TURN off the interrupt.
    ADC10IE &= ~ADC10OVIE;

	//Clear the pins
	P3OUT &= ~(BIT0+BIT1+BIT2+BIT3);
	P3DIR &= ~(BIT0+BIT1+BIT2+BIT3);
	P3REN |= (BIT0+BIT1+BIT2+BIT3);
	P3SEL0 |= (BIT0+BIT1+BIT2+BIT3);
	P3SEL1 |= (BIT0+BIT1+BIT2+BIT3);

	P1OUT &= ~(BIT0+BIT1+BIT2+BIT4+BIT5);
	P1DIR &= ~(BIT0+BIT1+BIT2+BIT4+BIT5);
	P1REN |= (BIT0+BIT1+BIT2+BIT4+BIT5);
	P1SEL0 |= (BIT0+BIT1+BIT2+BIT4+BIT5);
	P1SEL1 |= (BIT0+BIT1+BIT2+BIT4+BIT5);

	//4 Clock Cycles in sample, Continous Saample, Adc On
	ADC10CTL0 = ADC_CLOCK_CYCLES + ADC10MSC;
	//Repeat Single Channel, Sm Clock, Divide clock by 1
	ADC10CTL1 = ADC10SHP + ADC10CONSEQ_2 + ADC_CLOCK_SOURCE + ADC_CLOCK_DIVIDE;
	//8 bit resolution
	ADC10CTL2 &= ~ADC10RES;

	// Source Reference, Channel A15
	ADC10MCTL0 = ADC_REFERENCE_VOLTAGE;

//	ADC10CTL0 |= ADC10ON;
	//VIMP!!! I am going to turn on an interrupt for when a value is not collected!
	// Turn ADC interrupt on. This will catch the missed samples.
	ADC10IE |= ADC10OVIE;

}

//******************************************************************************
//This function refreshes the DMA registers and pins. It can be used before the
// ADC is setup. All that is left after this is to turn on the ADC, start and enable,
// and select which channel will be sampled.

//Must stop any ADC processes before this!

//Can be used as the setup of the DMA. Must enable the DMA's after this.
//******************************************************************************
void refresh_DMA(void){
	//Turn off the DMA. Reinitialise its registers. Whipe the DMA buffers.
	DMA0CTL &= ~(DMAIE + DMAEN);
	DMA1CTL &= ~(DMAIE + DMAEN);
	rep_buffer0_ptr 		= (unsigned char *)BUFFER0_STR_ADD;
	rep_buffer1_ptr 		= (unsigned char *)BUFFER1_STR_ADD;

	clear_buffers();

	// Configure DMA0
	DMACTL0 = DMA0TSEL__ADC10IFG + DMA1TSEL__ADC10IFG;            	// ADC10IFG trigger - Set for both DMA Channels
	__data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC10MEM0); 		// Source single address
	__data16_write_addr((unsigned short) &DMA0DA,(unsigned long) (rep_buffer0_ptr + 5));	// Destination array address
	DMA0SZ = (unsigned long)AMMOUNT_OF_SAMPLES_IN_PACKET;                            // Sets the ammount of transfers to be completed - Set above.
	DMA0CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL + DMASRCBYTE + DMADSTBYTE;
	// Single transfer process with DM0SZ itterations, inc dest, byte access in both registers,

	// Configure DMA1
	__data16_write_addr((unsigned short) &DMA1SA,(unsigned long) &ADC10MEM0);			// Source single address
	__data16_write_addr((unsigned short) &DMA1DA,(unsigned long) (rep_buffer1_ptr + 5));	// Destination array address
	DMA1SZ = (unsigned long)AMMOUNT_OF_SAMPLES_IN_PACKET;                            // Sets the ammount of transfers to be completed - Set above.
	DMA1CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL + DMASRCBYTE + DMADSTBYTE;
	// Single transfer process with DM1SZ itterations, inc dest, byte access in both registers,
}


//******************************************************************************
//This function clears the memory content of the DMA buffers.
//******************************************************************************
void clear_buffers(void){
	buffer_wiper_ptr = (unsigned char *)BUFFER0_STR_ADD;
	buffer_wiper_ptr +=5;

	for(k=0; k< ammount_of_samples; k++){
		*buffer_wiper_ptr = 0x41;
		buffer_wiper_ptr++;

	}

	buffer_wiper_ptr = (unsigned char *)BUFFER1_STR_ADD;
	buffer_wiper_ptr +=5;

	for(k=0; k< ammount_of_samples; k++){
		*buffer_wiper_ptr = 0x43;
		buffer_wiper_ptr++;

	}

}
//******************************************************************************
//This function is quite important. It is responsible for configuring
//which adc channel is sampled. This is called whenever the configeration of the
//device is changed.
//
//Before it is called the "Referesh ADC" function should be called.
//
//The input variable is a pointer to the values recieved by the incomming
//configeration message recieved from the iPad
//******************************************************************************
void configure_channel(unsigned long*ptr){

	switch(*ptr)
	{
	case channel1:
		ADC10MCTL0 |= ADC10INCH_0;
		break;
	case channel2:
		ADC10MCTL0 |= ADC10INCH_1;
		break;
	case channel3:
		ADC10MCTL0 |= ADC10INCH_2;
		break;
	case channel4:
		ADC10MCTL0 |= ADC10INCH_13;
		break;
	case channel5:
		ADC10MCTL0 |= ADC10INCH_14;
		break;
	case channel6:
		ADC10MCTL0 |= ADC10INCH_15;
		break;
	case channel7:
		ADC10MCTL0 |= ADC10INCH_12;
		break;
	case channel8:
		ADC10MCTL0 |= ADC10INCH_4;
		break;
	case channel9:
		ADC10MCTL0 |= ADC10INCH_5;
		break;
	case channel123:
		//Do a setup for all three channels
		ADC10MCTL0 |= ADC10INCH_5;
		break;
	default:
		ADC10MCTL0 |= ADC10INCH_15;
		// Do nothing. There was no setup info given.
		break;

	}
	ADC10CTL0 |= (ADC10ON);
}

