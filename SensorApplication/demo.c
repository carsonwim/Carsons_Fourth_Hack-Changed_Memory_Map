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
*SOCKOPT_RECV_TIMEOUT
*****************************************************************************/
#include <msp430.h>
#include "wlan.h" 
#include "evnt_handler.h"
#include "socket.h"
#include "common.h"
#include "netapp.h"
#include "cc3000.h"
#include "board.h"
#include "string.h"
#include "device_config.h"
#include "server.h"
#include "spi.h"
#include "server_setup.h"

void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;				// Stop WDT
    resetCC3000StateMachine();				// Start CC3000 State Machine
    initDriver();							// Initialize Board and CC3000
    unsolicicted_events_timer_init();		// Initialize CC3000 Unsolicited Events Timer
    __enable_interrupt();					// Enable interrupts for UART
    DefaultWifiConnection();				// Do a default connection

    while (1)
    {

        hci_unsolicited_event_handler();	// Handle any un-solicited event if required - the function shall be triggered few times in a second
        unsolicicted_events_timer_init();

        if(currentCC3000State() & CC3000_IP_ALLOC)
        {
            turnLedOn(CC3000_IP_ALLOC_IND);
            unsolicicted_events_timer_disable();
            
            // Attempt to start data server
            initServer();
            if(currentCC3000State() & CC3000_SERVER_INIT)
            {
                waitForConnection();
            }
            else//Wait for a bit, and try again.
            {
                __delay_cycles(100000);
            }
            unsolicicted_events_timer_init();
        }
    }
}


