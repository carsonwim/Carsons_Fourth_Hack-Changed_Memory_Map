/*
 * ServerAdditionalFeatures.c
 *
 *  Created on: 27 Sep 2012
 *      Author: Carson McAfee
 */

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
#include "uart.h"
#include "sensors.h"
#include "board.h"
#include "terminal.h"
#include "strlib.h"
#include "server.h"

#include "ServerAdditionalFeatures.h"

void printName (void){
	terminalPrint("\n\r New Class Works \n\n\n");
}



//void DATA_Send_FN (char * buffer_word){
//
//	if(strncmp(buffer_word,"DATA",strlen("DATA")) == 0)
//	                {
//	                    SetupAccel();
//	                    SetupThermistor();
//	                    while(currentCC3000State() & CC3000_CLIENT_CONNECTED)
//	                    {
//	                        // Start Sending Data to server
//	                        // Send data to CC3000
//	                        hci_unsolicited_event_handler();
//	                        unsolicicted_events_timer_disable();
//	                        toggleLed(CC3000_SENDING_DATA_IND);
//
//
//
//	                        SetupAccelXXX();
//	                        TakeADCMeas(X_AXIS_MEAS);      // Take 1 ADC Sample for X-AXIS
//
//
//	                        SetupAccelYYY();
//	                        TakeADCMeas(Y_AXIS_MEAS);      // Take 1 ADC Sample for Y-AXIS
//
//
//	                        SetupAccelZZZ();
//	                        TakeADCMeas(Z_AXIS_MEAS);      // Take 1 ADC Sample for Y-AXIS
//
//
//	                        SetupThermistorADC();  // One time setup and calibration
//	                        TakeADCMeas(TEMP_MEAS);         // Take 1 ADC Sample FOR TEMP
//
//	                        SetupVcc();
//	                        TakeADCMeas(VCC_MEAS);
//
//	                        __no_operation();
//
//	                        bytesSent = send(clientDescriptor, (unsigned char *)dataPacket, sizeof(dataPacket), 0);
//	                        if (bytesSent != sizeof(dataPacket))
//	                        {
//	                            // Check if socket is still available
//	                            curSocket =  getsockopt(clientDescriptor, SOL_SOCKET, SOCK_DGRAM , &optval, (socklen_t*)&optlen);
//	                            if (curSocket != 0)
//	                            {
//	                                closesocket(clientDescriptor);
//	                                terminalPrint("Client Disconnected\r\n");
//																	clientDescriptor = -1;
//	                                unsetCC3000MachineState(CC3000_CLIENT_CONNECTED);
//	                            }
//	                        }
//	                        unsolicicted_events_timer_init();
//	                        __delay_cycles(10);			//this should wait a second
//	                    }
//	                }
//}
