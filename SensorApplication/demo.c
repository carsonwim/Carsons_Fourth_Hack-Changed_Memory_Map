/*****************************************************************************
*
*  demo.c - CC3000 Main Demo Application
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
*
*****************************************************************************/

#include <msp430.h>
#include "wlan.h" 
#include "evnt_handler.h"    // callback function declaration
#include "nvmem.h"
#include "socket.h"
#include "common.h"
#include "netapp.h"
#include "cc3000.h"
#include "sensors.h"
#include "board.h"
#include "string.h"
//#include "uart.h"
#include "demo_config.h"
//#include "terminal.h"
#include "server.h"
#include "version.h"
#include "spi.h"
#include "carsons_file.h"

/** \brief Flag indicating whether user requested to perform FTC */
char runFirstTimeConfig = 0;
unsigned char * ptrFtcAtStartup = (unsigned char *)0x1830;

/** \brief Flag indicating whether to print CC3000 Connection info */
unsigned char obtainIpInfoFlag = FALSE;

void errorHandler();

void main(void)
{

    WDTCTL = WDTPW + WDTHOLD;// Stop WDT
    resetCC3000StateMachine();// Start CC3000 State Machine
    initDriver();// Initialize Board and CC3000
    unsolicicted_events_timer_init();// Initialize CC3000 Unsolicited Events Timer
    __enable_interrupt();// Enable interrupts for UART

    DefaultWifiConnection();//    Do a default connection

    while (1)
    {

        hci_unsolicited_event_handler();// Handle any un-solicited event if required - the function shall be triggered few times in a second
        
        // Print out connection status
        if(currentCC3000State() & CC3000_IP_ALLOC && obtainIpInfoFlag == FALSE)// Checks if the IP given is a valid IP
        {            
            unsolicicted_events_timer_disable();
#ifndef CC3000_TINY_DRIVER//This checks if we are assigned a bad IP addres by the server. Ha ha ha. Ha been to Wits?
            // Check if we get IP of 50.xx.xx.xx
            if(getCC3000Info()->aucIP[3] == 50)
            {
               // We received a Bad IP
                errorHandler();
            }
#endif               
            // Set flag so we don't constantly obtain the IP info
            obtainIpInfoFlag = TRUE;
            turnLedOn(CC3000_IP_ALLOC_IND);
            unsolicicted_events_timer_init();
        }
        
        if(currentCC3000State() & CC3000_IP_ALLOC && obtainIpInfoFlag == TRUE)// THIS IS THE MAIN PIECE OF CODE!!!!
        {
            unsolicicted_events_timer_disable();
            
            // Attempt to start data server
            initServer();
            if(currentCC3000State() & CC3000_SERVER_INIT)
            {
                waitForConnection();
            }
            else//Wait for a bit, and try again.
            {
                __delay_cycles(100000);			//this should wait a second
            }
            unsolicicted_events_timer_init();
        }
    }
}















#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{    
    switch(__even_in_range(P4IV,P4IV_P4IFG1))
    {        
        case P4IV_P4IFG0:
            StartDebounceTimer();          //
            // disable switch interrupt
            DissableSwitch();
            break;
        
        default:
            break;
    }  
    P4IFG = 0;
}



// Timer B0 interrupt service routine
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer_B (void)
{
    // Check whether button is still pressed. If so, First Time Config
    // should be performed.
    
    if(!(switchIsPressed()))
    {
        // Button is still pressed, so FTC should be done        
        runFirstTimeConfig = 1;
        
        if(currentCC3000State() & CC3000_SERVER_INIT)
        {
            // Since accept and the server is blocking,
            // we will indicate in non-volatile FRAM that
            // First Time Config should be run at startup.
              SetFTCflag();
//              terminalPrint("Restarting MSP430...\r\n");
            restartMSP430();
        }
    }
    
    // Restore Switch Interrupt
        RestoreSwitch();
    
    StopDebounceTimer();
}

//*****************************************************************************
//
//!  \brief  Generic Error Handler that blinks LEDs
//!
//!  Look at the Call Stack to see who called the Error handler
//!
//!  \param  None
//!
//!  \return none
//!
//
//*****************************************************************************
void errorHandler()
{
    while(1)
    {
        turnLedOn(CC3000_UNUSED1_IND);
        __delay_cycles(100000);
        toggleLed(CC3000_UNUSED1_IND);
        __delay_cycles(100000);
        __no_operation();
    }
}
