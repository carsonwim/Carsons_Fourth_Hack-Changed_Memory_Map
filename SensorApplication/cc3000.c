

#include "cc3000.h"
#include "msp430fr5739.h"
#include "wlan.h" 
#include "evnt_handler.h"    // callback function declaration
#include "nvmem.h"
#include "socket.h"
#include "common.h"
#include "netapp.h"
#include "board.h"
#include "common.h"
#include "string.h"
#include "demo_config.h"
//#include "uart.h"
#include "sensors.h"
#include "board.h"
//#include "terminal.h"
#include "spi.h"
#include "carsons_file.h"

long ulSocket;

/** \brief Indicates whether the Smart Config Process has finished */
unsigned long ulSmartConfigFinished;

unsigned char pucIP_Addr[4];
unsigned char pucIP_DefaultGWAddr[4];
unsigned char pucSubnetMask[4];
unsigned char pucDNS[4];

sockaddr tSocketAddr;

unsigned char prefixChangeFlag = 0;
unsigned char prefixFromUser[3] = {0};
char * ftcPrefixptr;

tNetappIpconfigRetArgs ipinfo;

char debugi = 0;
// First Time Config Prefix - Texas Instruments
// NOTE: the actual value of the prefix may change
// if the Prefix Change process is performed
const char aucCC3000_prefix[3] = {'T', 'T', 'T'};

#define FRAM_FTC_INFO_ADDRESS       0x1800

unsigned char *FRAM_FIRST_TIME_CONFIG_WRITTEN_ptr = (unsigned char *)FRAM_FTC_INFO_ADDRESS;   


char cc3000state = CC3000_UNINIT;



//*****************************************************************************
//
//! ConnectUsingSSID
//!
//! \param  ssidName is a string of the AP's SSID
//!
//! \return none
//!
//! \brief  Connect to an Access Point using the specified SSID
//
//*****************************************************************************
//int ConnectUsingSSID(char * ssidName)
//{
////    terminalPrint("Attempting to connect using SSID: ");
////    terminalPrint((char*)ssidName);
////    terminalPrint("\r\n");
//
//    unsetCC3000MachineState(CC3000_ASSOC);
//
//    // Disable Profiles and Fast Connect
//    wlan_ioctl_set_connection_policy(0, 0, 0);
//
//    wlan_disconnect();
//
//    __delay_cycles(10000);
//
//    // This triggers the CC3000 to connect to specific AP with certain parameters
//    //sends a request to connect (does not necessarily connect - callback checks that for me)
//#ifndef CC3000_TINY_DRIVER
//    wlan_connect(DEFAULT_AP_SECURITY, ssidName, strlen(ssidName), NULL, DFAULT_AP_SECURITY_KEY, strlen(DFAULT_AP_SECURITY_KEY));
//#else
//    wlan_connect(ssidName, strlen(ssidName));
//#endif
//    // We don't wait for connection. This is handled somewhere else (in the main
//    // loop for example).
//
//    return 0;
//}

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
//    terminalPrint("Attempting default connection: ");
//    terminalPrint((char*)DEFAULT_OUT_OF_BOX_SSID);
//    terminalPrint("\r\n");

    unsetCC3000MachineState(CC3000_ASSOC);

    // Disable Profiles and Fast Connect
    wlan_ioctl_set_connection_policy(0, 0, 0);

    wlan_disconnect();

    __delay_cycles(10000);

    // This triggers the CC3000 to connect to specific AP with certain parameters
    //sends a request to connect (does not necessarily connect - callback checks that for me)
#ifndef CC3000_TINY_DRIVER
    wlan_connect(DEFAULT_AP_SECURITY, DEFAULT_OUT_OF_BOX_SSID, strlen(DEFAULT_OUT_OF_BOX_SSID), NULL, DFAULT_AP_SECURITY_KEY, strlen(DFAULT_AP_SECURITY_KEY));
#else
    wlan_connect(DEFAULT_OUT_OF_BOX_SSID, strlen(DEFAULT_OUT_OF_BOX_SSID));
#endif
    // We don't wait for connection. This is handled somewhere else (in the main
    // loop for example).

    return 0;
}


//*****************************************************************************
//
//! SetupIPAddress
//!
//! \param  buffer
//!
//! \return none
//!
//! \brief  Configure's the CC3000's IP Address
//
//*****************************************************************************
//void SetupIPAddress (unsigned char ub, unsigned char mub, unsigned char mlb,unsigned char lb)
//{
//    //
//    // Subnet mask is assumed to be 255.255.255.0
//    //
//    pucSubnetMask[0] = 0xFF;
//    pucSubnetMask[1] = 0xFF;
//    pucSubnetMask[2] = 0xFF;
//    pucSubnetMask[3] = 0x0;
//
//    // CC3000's IP
//    pucIP_Addr[0] = ub;
//    pucIP_Addr[1] = mub;
//    pucIP_Addr[2] = mlb;
//    pucIP_Addr[3] = lb;
//
//    // Default Gateway/Router IP
//    // 192.168.1.1
//    pucIP_DefaultGWAddr[0] = 192;
//    pucIP_DefaultGWAddr[1] = 168;
//    pucIP_DefaultGWAddr[2] = 10;
//    pucIP_DefaultGWAddr[3] = 1;
//
//    //
//    // Currently no implementation of DHCP in hte demo
//    //
//    pucDNS[0] = 0;
//    pucDNS[1] = 0;
//    pucDNS[2] = 0;
//    pucDNS[3] = 0;
//
//    netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask, (unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);
//}
//
//
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
//       // Init the device with 24MHz DCOCLCK.
//          initClk();

    // Init GPIO's
    pio_init();
    
    //
    //init all layers
    //
    init_spi();
    
    // this is done for debug purpose only
    
    //
    // WLAN On API Implementation
    //
    wlan_init( CC3000_UsynchCallback, sendWLFWPatch, sendDriverPatch, sendBootLoaderPatch, ReadWlanInterruptPin, WlanInterruptEnable, WlanInterruptDisable, WriteWlanPin);
    
    //
    // Trigger a WLAN device
    //
    wlan_start(0);
        
#if IP_ALLOC_METHOD == USE_DHCP
    
    // DHCP is used by default
    
    // Subnet mask is assumed to be 255.255.255.0

    pucSubnetMask[0] = 0;
    pucSubnetMask[1] = 0;
    pucSubnetMask[2] = 0;
    pucSubnetMask[3] = 0;
    
    // CC3000's IP
    pucIP_Addr[0] = 0;
    pucIP_Addr[1] = 0;
    pucIP_Addr[2] = 0;
    pucIP_Addr[3] = 0;
    
    // Default Gateway/Router IP
    // 192.168.1.1
    pucIP_DefaultGWAddr[0] = 0;
    pucIP_DefaultGWAddr[1] = 0;
    pucIP_DefaultGWAddr[2] = 0;
    pucIP_DefaultGWAddr[3] = 0;
    
    // We assume the router is also a DNS server
    pucDNS[0] = 0;
    pucDNS[1] = 0;
    pucDNS[2] = 0;
    pucDNS[3] = 0;
    
    // Force DHCP
    netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask, 
                (unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);
    
    //
    // reset the CC3000
    // 
    wlan_stop();
    __delay_cycles(6000000);            
    wlan_start(0);
    
#elif IP_ALLOC_METHOD == USE_STATIC_IP

    // Subnet mask is assumed to be 255.255.255.0

    pucSubnetMask[0] = 0xFF;
    pucSubnetMask[1] = 0xFF;
    pucSubnetMask[2] = 0xFF;
    pucSubnetMask[3] = 0x0;
    
    // CC3000's IP
    pucIP_Addr[0] = STATIC_IP_OCT1;
    pucIP_Addr[1] = STATIC_IP_OCT2;
    pucIP_Addr[2] = STATIC_IP_OCT3;
    pucIP_Addr[3] = STATIC_IP_OCT4;
    
    // Default Gateway/Router IP
    // 192.168.1.1
    pucIP_DefaultGWAddr[0] = STATIC_IP_OCT1;
    pucIP_DefaultGWAddr[1] = STATIC_IP_OCT2;
    pucIP_DefaultGWAddr[2] = STATIC_IP_OCT3;
    pucIP_DefaultGWAddr[3] = 1;
    
    // We assume the router is also a DNS server
    pucDNS[0] = STATIC_IP_OCT1;
    pucDNS[1] = STATIC_IP_OCT2;
    pucDNS[2] = STATIC_IP_OCT3;
    pucDNS[3] = 1;
    
    netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask, 
                (unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);    
    //
    // reset the CC3000 to apply Static Setting
    // 
    wlan_stop();
    __delay_cycles(6000000);
    wlan_start(0);
    
#else 
#error No IP Configuration Method Selected. One must be configured.
#endif    
    
    //
    // Mask out all non-required events from CC3000
    //
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_ASYNC_PING_REPORT);
    
    unsolicicted_events_timer_init();
    
    // CC3000 has been initialized
    setCC3000MachineState(CC3000_INIT);
    
    return(0);
}



//*****************************************************************************
//
//! StartFirstTimeConfig
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  The function triggers a smart configuration process on CC3000.
//!			it exists upon completion of the process
//
//*****************************************************************************
//
//void StartFirstTimeConfig(void)
//{
//    ulSmartConfigFinished = 0;
//    unsetCC3000MachineState(CC3000_ASSOC);
//
//    //
//    // Trigger the First Time Config process
//    //
//    turnLedOn(CC3000_FTC_IND);
//
//    //
//    // Reset all the previous configuration
//    //
//    wlan_ioctl_set_connection_policy(0,0,0);
//
//    wlan_ioctl_del_profile(0);
//    wlan_ioctl_del_profile(1);
//    wlan_ioctl_del_profile(2);
//
//    __delay_cycles(5000);
//
////    terminalPrint("Starting First Time Config\r\n");
//
//    wlan_first_time_config_set_prefix((char *)aucCC3000_prefix);
//
//    //
//    // Start the First Time Config process
//    //
//    wlan_first_time_config_start();
//
//    //
//    // Wait for First Time config to finish
//    //
//    while (ulSmartConfigFinished == 0)
//    {
//        __delay_cycles(6000000);
//
//        turnLedOff(CC3000_FTC_IND);
//
//        __delay_cycles(6000000);
//
//      	turnLedOn(CC3000_FTC_IND);
//
//        hci_unsolicited_event_handler();
//    }
//
//    turnLedOn(CC3000_FTC_IND);
//
//    // Set flag in FRAM to indicate Smart Config finished successfully
//    *FRAM_FIRST_TIME_CONFIG_WRITTEN_ptr = FIRST_TIME_CONFIG_SET;
//
//    //
//    // Configure to connect automatically to the AP retrieved in the
//    // simple config process
//    //
//    wlan_ioctl_set_connection_policy(0, 0, 1);
//
//    //
//    // reset the CC3000
//    //
//    wlan_stop();
//
//    __delay_cycles(600000);
//
//    wlan_start(0);
//
//    //
//    // Mask out all non-required events
//    //
//    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_ASYNC_PING_REPORT);
//
//    while (!(currentCC3000State() & CC3000_ASSOC))
//    {
//        __delay_cycles(10000);
//
//        hci_unsolicited_event_handler();
//    }
//
////    terminalPrint("First Time Config Complete\r\n");
//}


//*****************************************************************************
//
//! Disconnect All
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  Gracefully disconnects us completely
//
//*****************************************************************************
//void disconnectAll()
//{
//
//}

//*****************************************************************************
//
//!  \brief  Return the highest state which we're in
//!
//!  \param  None
//!
//!  \return none
//!
//
//*****************************************************************************
//char highestCC3000State()
//{
//    // We start at the highest state and go down, checking if the state
//    // is set.
//    char mask = 0x80;
//    while(!(cc3000state & mask))
//    {
//        mask = mask >> 1;
//    }
//
//    return mask;
//}

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
//*****************************************************************************
//
//!  \brief  Checks whether First Time Config information is set in the CC3000
//!  
//!  \param  None
//!
//!  \return none
//!  \warning This function assumes that the CC3000 module was previously
//!           run with this same board. If not, it may incorrectly assume that
//!           the CC3000 was previously configured for FTC since it stores the
//!           flag in the FRAM.
//!  
//
//*****************************************************************************
//char isFTCSet()
//{
//    if(*FRAM_FIRST_TIME_CONFIG_WRITTEN_ptr == FIRST_TIME_CONFIG_SET)
//    {
//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}

//*****************************************************************************
//
//!  \brief  Sets the flag indicating First Time Config information is set in the 
//!          CC3000
//!  
//!  \param  None
//!
//!  \return none
//!  \warning This function assumes that the CC3000 module was previously
//!           run with this same board. If not, it may incorrectly assume that
//!           the CC3000 was previously configured for FTC since it stores the
//!           flag in the FRAM.
//!  
//
//*****************************************************************************
//void  setFTCFlag()
//{
//    *FRAM_FIRST_TIME_CONFIG_WRITTEN_ptr = FIRST_TIME_CONFIG_SET;
//}
