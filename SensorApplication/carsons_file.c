#include "cc3000.h"
#include "msp430fr5739.h"
#include "wlan.h"
#include "evnt_handler.h"    // callback function declaration
//#include "nvmem.h"
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


void demoPacketCreator(void)
{
    SetupAccelXXX();
    TakeADCMeas(X_AXIS_MEAS);      // Take 1 ADC Sample for X-AXIS


    SetupAccelYYY();
    TakeADCMeas(Y_AXIS_MEAS);      // Take 1 ADC Sample for Y-AXIS


    SetupAccelZZZ();
    TakeADCMeas(Z_AXIS_MEAS);      // Take 1 ADC Sample for Y-AXIS


    SetupThermistorADC();  // One time setup and calibration
    TakeADCMeas(TEMP_MEAS);         // Take 1 ADC Sample FOR TEMP

    SetupVcc();
    TakeADCMeas(VCC_MEAS);

    __no_operation();
}

//void Wifi_About_Device(tNetappIpconfigRetArgs * inf)
//{
//    if(highestCC3000State() == CC3000_INIT)
//    {
//        sendString("CC3000 Initialized\r\n");
//    }
//    else if(!(currentCC3000State() & CC3000_INIT))
//    {
//        sendString("Status: CC3000 Uninitialized\r\n");
//    }
//    if(currentCC3000State() & CC3000_ASSOC)
//    {
//        sendString("Connected to AP: ");
//        sendString((char *)inf->uaSSID);
//        sendString("\r\n");
//    }
//    if(currentCC3000State() & CC3000_IP_ALLOC)
//    {
//        sendString("CC3000 MAC Address: ");
//        printMACAddr((char *)inf->uaMacAddr);
//        sendString("\r\n");
//        sendString("CC3000 IP Address: ");
//        printIpAddr((char *)inf->aucIP);
//        sendString("\r\n");
//    }
//    if(currentCC3000State() & CC3000_SERVER_INIT)
//    {
//        sendString("Server Initialized\r\n");
//    }
//    if( currentCC3000State() & CC3000_CLIENT_CONNECTED)
//    {
//        sendString("Client Connected\r\n");
//    }
//}
