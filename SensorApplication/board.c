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

#include <msp430.h>
#include "wlan.h" 
#include "evnt_handler.h"    // callback function declaration
#include "socket.h"
#include "netapp.h"
#include "board.h"

void pio_init()
{

	 initClk(); // Init the device with 24MHz DCOCLCK.
    // P4.1 - WLAN enable full DS
 	WLAN_EN_PORT_OUT &= ~WLAN_EN_PIN;
	WLAN_EN_PORT_DIR |= WLAN_EN_PIN;
	WLAN_EN_PORT_SEL2 &= ~WLAN_EN_PIN; 
	WLAN_EN_PORT_SEL &= ~WLAN_EN_PIN;

    
    // Configure SPI IRQ line on P2.3
	SPI_IRQ_PORT_DIR  &= (~SPI_IRQ_PIN);	
	SPI_IRQ_PORT_SEL2 &= ~SPI_IRQ_PIN; 
	SPI_IRQ_PORT_SEL &= ~SPI_IRQ_PIN;

    
    // Configure the SPI CS to be on P1.3
   	SPI_CS_PORT_OUT |= SPI_CS_PIN;
	SPI_CS_PORT_DIR |= SPI_CS_PIN;
	SPI_CS_PORT_SEL2 &= ~SPI_CS_PIN; 
	SPI_CS_PORT_SEL &= ~SPI_CS_PIN;
    __delay_cycles(1200000);

    initLEDs();
}
//*****************************************************************************
//
//! ReadWlanInterruptPin
//!
//! \param  none
//!
//! \return none
//!
//! \brief  return wlan interrup pin
//
//*****************************************************************************

long ReadWlanInterruptPin(void)
{
    return (P2IN & BIT3);
}

//*****************************************************************************
//
//! Enable waln IrQ pin
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Nonr
//
//*****************************************************************************


void WlanInterruptEnable()
{
    __bis_SR_register(GIE);
    P2IES |= BIT3;
    P2IE |= BIT3;
}

//*****************************************************************************
//
//! Disable waln IrQ pin
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Nonr
//
//*****************************************************************************


void WlanInterruptDisable()
{
    P2IE &= ~BIT3;
}


//*****************************************************************************
//
//! WriteWlanPin
//!
//! \param  new val
//!
//! \return none
//!
//! \brief  void
//
//*****************************************************************************

void WriteWlanPin( unsigned char val )
{
    if (val)
    {
        P4OUT |= BIT1;	
    }
    else
    {
        P4OUT &= ~BIT1;
    }
}

//*****************************************************************************
//
//! unsolicicted_events_timer_init
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  The function initializes the unsolicited events timer handler
//
//*****************************************************************************
void unsolicicted_events_timer_init(void)
{
    TA1CCTL0 &= ~CCIE; 
    TA1CTL |= MC_0;
    
    // Configure teh timer for each 500 milli to handle un-solicited events
    TA1CCR0 = 0x4000;
    
    // run the timer from ACLCK, and enable interrupt of Timer A
    TA1CTL |= (TASSEL_1 + MC_1 + TACLR);
    
    TA1CCTL0 |= CCIE;
}


//*****************************************************************************
//
//! unsolicicted_events_timer_init
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  The function initializes a CC3000 device and triggers it to start operation
//
//*****************************************************************************
void unsolicicted_events_timer_disable(void)
{
	TA1CCTL0 &= ~CCIE; 
	TA1CTL |= MC_0;
}


// Timer A0 interrupt service routine
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{              

	__no_operation();                          // For debugger
}



//*****************************************************************************
//
//! init clk
//!
//!  \param  None
//!
//!  \return none
//!
//!  \Init the device with 16 MHz DCOCLCK.
//
//*****************************************************************************
void initClk(void)
{
	 
  // SMCLCK which will source also SPI will be sourced also by DCO
    // 
    CSCTL0_H = 0xA5;
    CSCTL1 |= DCORSEL + DCOFSEL0 + DCOFSEL1;	 // Set max. DCO setting sets Mclk to 24 Mhz
    CSCTL2 = SELA_1 + SELS_3 + SELM_3;		// set ACLK = VLO, the rest  = MCLK = DCO
    //Aux Clk set to VLO Clk
    //SM Clock and Main Clock set to DCO Clk
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;		// set all dividers to 0

}

//*****************************************************************************
//
//! Initialize LEDs
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Initializes LED Ports and Pins
//
//*****************************************************************************
void initLEDs()
{
    PJOUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);
//    P3OUT &= ~(BIT4 + BIT5 + BIT6 + BIT7);
    
    PJDIR |= (BIT0 + BIT1 + BIT2 + BIT3);
//    P3DIR |= (BIT4 + BIT5 + BIT6 + BIT7);
}

//*****************************************************************************
//
//! Turn LED On
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************
void turnLedOn(char ledNum)
{
    switch(ledNum)
    {
      case LED1:
//        PJOUT |= (BIT0);
        break;
      case LED2:
        PJOUT |= (BIT0);
        break;
      case LED3:
        PJOUT |= (BIT1);
        break;
      case LED4:
//        PJOUT |= (BIT3);
        break;
      case LED5:
        PJOUT |= (BIT2);
        break;
      case LED6:
        PJOUT |= (BIT3);
        break;
      case LED7:
//        P3OUT |= (BIT6);
        break;
      case LED8:
        PJOUT |= (BIT0);
        break;
    }
}

//*****************************************************************************
//
//! Turn LED Off
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************    
void turnLedOff(char ledNum)
{                     
    switch(ledNum)
    {
      case LED1:
//        PJOUT &= ~(BIT0);
        break;
      case LED2:
        PJOUT &= ~(BIT0);
        break;
      case LED3:
        PJOUT &= ~(BIT1);
        break;
      case LED4:
//        PJOUT &= ~(BIT3);
        break;
      case LED5:
        PJOUT &= ~(BIT2);
        break;
      case LED6:
        PJOUT &= ~(BIT3);
        break;
      case LED7:
//        P3OUT &= ~(BIT6);
        break;
      case LED8:
        PJOUT &= ~(BIT0);
        break;
    }
}

//*****************************************************************************
//
//! toggleLed
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Toggles a board LED
//
//*****************************************************************************    

void toggleLed(char ledNum)
{
    switch(ledNum)
    {
      case LED1:
//        PJOUT ^= (BIT0);
        break;
      case LED2:
        PJOUT ^= (BIT0);
        break;
      case LED3:
        PJOUT ^= (BIT1);
        break;
      case LED4:
//        PJOUT ^= (BIT3);
        break;
      case LED5:
        PJOUT ^= (BIT2);
        break;
      case LED6:
        PJOUT ^= (BIT3);
        break;
      case LED7:
//        P3OUT ^= (BIT6);
        break;
      case LED8:
        PJOUT ^= (BIT0);
        break;
    }
}


//*****************************************************************************
//
//! \brief  Restarts the MSP430
//!
//! Restarts the MSP430 completely. One must be careful
//!
//! \return never
//!
//
//*****************************************************************************    
void restartMSP430()
{
   
    PMMCTL0 |= PMMSWPOR;
        
    // This function will never exit since it forces a complete
    // restart of the MSP430.    
}

//Catch interrupt vectors that are not initialized.

#ifdef __CCS__
#pragma vector=PORT1_VECTOR, WDT_VECTOR, TIMER1_A1_VECTOR, TIMER0_A1_VECTOR, TIMER0_A0_VECTOR, UNMI_VECTOR,COMP_D_VECTOR, PORT3_VECTOR, RTC_VECTOR, TIMER0_B1_VECTOR, TIMER1_B0_VECTOR, TIMER1_B1_VECTOR, TIMER2_B0_VECTOR, TIMER2_B1_VECTOR,SYSNMI_VECTOR, USCI_A1_VECTOR, USCI_B0_VECTOR
__interrupt void Trap_ISR(void)
{
  while(1);
}

#endif


