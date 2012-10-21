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
#include "spi.h"


long ulSocket;

/** \brief Indicates whether the Smart Config Process has finished */
unsigned long ulSmartConfigFinished;

unsigned char pucIP_Addr[4];
unsigned char pucIP_DefaultGWAddr[4];
unsigned char pucSubnetMask[4];
unsigned char pucDNS[4];

sockaddr tSocketAddr;
tNetappIpconfigRetArgs ipinfo;

char debugi = 0;
char cc3000state = CC3000_UNINIT;

//*****************************************************************************
// Crasons First FUnction
//! DefaultWifiConnection
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Connect to an Access Point using the default values "demo_config.h"
//
//*****************************************************************************
int DefaultWifiConnection(void)
{
    unsetCC3000MachineState(CC3000_ASSOC);
    // Disable Profiles and Fast Connect
    wlan_ioctl_set_connection_policy(0, 0, 0);
    wlan_disconnect();
    __delay_cycles(10000);
    wlan_connect(DEFAULT_AP_SECURITY, DEFAULT_OUT_OF_BOX_SSID, strlen(DEFAULT_OUT_OF_BOX_SSID), NULL, DFAULT_AP_SECURITY_KEY, strlen(DFAULT_AP_SECURITY_KEY));
    return 0;
}


//*****************************************************************************
//
//! sendDriverPatch
//!
//! \param  pointer to the length
//!
//! \return none
//!
//! \brief  The function returns a pointer to the driver patch: since there is no patch yet - 
//!				it returns 0
//
//*****************************************************************************
char *sendDriverPatch(unsigned long *Length)
{
    *Length = 0;
    return NULL;
}


//*****************************************************************************
//
//! sendBootLoaderPatch
//!
//! \param  pointer to the length
//!
//! \return none
//!
//! \brief  The function returns a pointer to the boot loader patch: since there is no patch yet -
//!				it returns 0
//
//*****************************************************************************
char *sendBootLoaderPatch(unsigned long *Length)
{
    *Length = 0;
    return NULL;
}

//*****************************************************************************
//
//! sendWLFWPatch
//!
//! \param  pointer to the length
//!
//! \return none
//!
//! \brief  The function returns a pointer to the FW patch: since there is no patch yet - it returns 0
//
//*****************************************************************************

char *sendWLFWPatch(unsigned long *Length)
{
    *Length = 0;
    return NULL;
}


//*****************************************************************************
//
//! CC3000_UsynchCallback
//!
//! \param  Event type
//!
//! \return none
//!
//! \brief  The function handles asynchronous events that come from CC3000 device 
//!		  and operates a LED4 to have an on-board indication
//
//*****************************************************************************

void CC3000_UsynchCallback(long lEventType, char * data, unsigned char length)
{
    if (lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
    {
        ulSmartConfigFinished = 1;        
    }
    
    if (lEventType == HCI_EVNT_WLAN_UNSOL_INIT)
    {
        setCC3000MachineState(CC3000_INIT);
    }
    if (lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
    {
        setCC3000MachineState(CC3000_ASSOC);
        
    }
    
    if (lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
    {
        if(currentCC3000State() & CC3000_SERVER_INIT)
        {
            // We're waiting for a client to connect. However,
            // because we just received the disconnect event, we're stuck
            // because of the blocking nature of accept. We restart the MSP430
            // so the CC3000 will wait to associate again.
//             terminalPrint("Restarting MSP430...\r\n");
            restartMSP430();
        }
        unsetCC3000MachineState(CC3000_ASSOC);
        
    }
    if (lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
    {
        setCC3000MachineState(CC3000_IP_ALLOC);        
    }
}




//*****************************************************************************
//
//! initDriver
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  The function initializes a CC3000 device and triggers it to start operation
//
//*****************************************************************************
int
initDriver(void)
{



    pio_init(); // Init GPIO's
    init_spi();
    wlan_init( CC3000_UsynchCallback, sendWLFWPatch, sendDriverPatch, sendBootLoaderPatch, ReadWlanInterruptPin, WlanInterruptEnable, WlanInterruptDisable, WriteWlanPin);
    wlan_start(0);

	if (IP_ALLOC_METHOD == USE_STATIC_IP){// The DHCP setting shave been removed.

		pucSubnetMask[0] = 0xFF;// Subnet mask is assumed to be 255.255.255.0
		pucSubnetMask[1] = 0xFF;
		pucSubnetMask[2] = 0xFF;
		pucSubnetMask[3] = 0x0;


		pucIP_Addr[0] = STATIC_IP_OCT1;    // CC3000's IP
		pucIP_Addr[1] = STATIC_IP_OCT2;
		pucIP_Addr[2] = STATIC_IP_OCT3;
		pucIP_Addr[3] = STATIC_IP_OCT4;

		pucIP_DefaultGWAddr[0] = STATIC_IP_OCT1;// Default Gateway/Router IP
		pucIP_DefaultGWAddr[1] = STATIC_IP_OCT2;
		pucIP_DefaultGWAddr[2] = STATIC_IP_OCT3;
		pucIP_DefaultGWAddr[3] = 1;

		pucDNS[0] = STATIC_IP_OCT1;// We assume the router is also a DNS server
		pucDNS[1] = STATIC_IP_OCT2;
		pucDNS[2] = STATIC_IP_OCT3;
		pucDNS[3] = 1;

		netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask,
					(unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);

		// reset the CC3000 to apply Static Setting
		wlan_stop();
		__delay_cycles(6000000);
		wlan_start(0);
	}
    
    // Mask out all non-required events from CC3000
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_ASYNC_PING_REPORT);
    
    unsolicicted_events_timer_init();
    
    // CC3000 has been initialized
    setCC3000MachineState(CC3000_INIT);
    
    return(0);
}
//*****************************************************************************
//
//!  \brief  Return the current state bits
//!
//!  \param  None
//!
//!  \return none
//!
//
//*****************************************************************************
char currentCC3000State()
{
    return cc3000state;
}

void setCC3000MachineState(char stat)
{	    
    char bitmask = stat;
    cc3000state |= bitmask;
    
    int i = FIRST_STATE_LED_NUM;
    
    // Find LED number which needs to be set
    while(bitmask < 0x80)
    {      
        bitmask  = bitmask << 1;
        i++;
    }    
    debugi = i;
    turnLedOn(NUM_STATES-i+2);    
}


//*****************************************************************************
//
//!  \brief  Unsets a state from the state machine
//!  Also handles LEDs
//!  
//!  \param  None
//!
//!  \return none
//!  
//
//*****************************************************************************
void unsetCC3000MachineState(char stat)
{
    char bitmask = stat;
    cc3000state &= ~bitmask;
    
    int i = FIRST_STATE_LED_NUM;
    int k = NUM_STATES; // Set to last element in state list
    
    // Set all upper bits to 0 as well since state machine cannot have
    // those states.
    while(bitmask < 0x80)
    {
        cc3000state &= ~bitmask;
        bitmask = bitmask << 1;
        i++;
    }
    
    // Turn off all upper state LEDs
    for(; i > FIRST_STATE_LED_NUM; i--)
    {
        turnLedOff(k);
        k--;
    }    
}

//*****************************************************************************
//
//!  \brief  Resets the State Machine
//!  
//!  \param  None
//!
//!  \return none
//!  
//
//*****************************************************************************
void resetCC3000StateMachine()
{
    cc3000state = CC3000_UNINIT;
    
    // Turn off all Board LEDs
    turnLedOff(CC3000_ON_IND);
    turnLedOff(CC3000_ASSOCIATED_IND);
    turnLedOff(CC3000_IP_ALLOC_IND);
    turnLedOff(CC3000_SERVER_INIT_IND);
    turnLedOff(CC3000_CLIENT_CONNECTED_IND);
    toggleLed(CC3000_SENDING_DATA_IND);
    turnLedOff(CC3000_UNUSED1_IND);
    turnLedOff(CC3000_FTC_IND);
}

//*****************************************************************************
//
//!  \brief  Obtains the CC3000 Connection Information from the CC3000
//!  
//!  \param  None
//!
//!  \return none
//!  
//
//*****************************************************************************
#ifndef CC3000_TINY_DRIVER
tNetappIpconfigRetArgs * getCC3000Info()
{
    if(!(currentCC3000State() & CC3000_SERVER_INIT))
    {
        // If we're not blocked by accept or others, obtain the latest
        netapp_ipconfig(&ipinfo);
    }    
    return &ipinfo;
}
#endif
