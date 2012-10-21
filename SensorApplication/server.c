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


//Old Variable Decleration
long serverSocket;
sockaddr serverSocketAddr;
sockaddr clientaddr;
socklen_t addrlen;
int clientDescriptor = -1;
volatile int bytesRecvd = 0;
int curSocket = 0;
int bytesSent = 0;
int optval, optlen;
//*****************************************************************************************************************************
//*****************************************************************************************************************************
//*****************************************************************************************************************************
//New Variable Declaration

char generalConfirmationPacket[] = { Setup_Complete, 'A','A', 'A','A','A','A'};		//Generalised format for a send message.

char incomingPacketData1; 					//Data from incoming config message. Will be used to setup the adc.
char incomingPacketData2;					//Data from incoming config message. Will be used to setup the adc.
unsigned long config_data = 0;

char deviceconfigured = FALSE;				//By default the device is not configured. Checked by the incoming message.
char DMA0_State = NOT_DONE;					// State of DMA register process
char DMA1_State = NOT_DONE;
char ADC_STATE = HALTED;
char DMA_STATE = HALTED;

unsigned long ammount_of_samples_in_packet = (unsigned long)AMMOUNT_OF_SAMPLES_IN_PACKET;;	//Should be put in a config header for the device.
unsigned long ammount_packets_to_be_sent = (unsigned long)AMMOUNT_PACKETS_TO_BE_SENT;		//Should be put in a config header for the device.
unsigned char temp_word_holder[2];
unsigned char *temp_ptr;


int i;										//Used in "for" loop.
unsigned long missed_samples =0;

char *fake_buffer_ptr;
char fake_value;

unsigned char *buffer0_ptr;
unsigned char *buffer1_ptr;
unsigned char *sendingBuffer_ptr;
char packet_header_size = 5; 				//<Command 1 Byte><Size of data 2 bytes><Missed Data 2 bytes>
unsigned char *confirmationPacket_ptr;   	// Packet sent to confirm a recieved message.


// Packet Structure: <Command 1 Byte><Size of data 2 bytes><Missed Data 2 bytes><Data of whatever size>
unsigned char dataPacket[] = { 0x01,   0x05,0x00,    0x00,0x00,    0x01,0x02,0x03,0x04,0x05};
char serverErrorCode = 0;
//*****************************************************************************************************************************
//*****************************************************************************************************************************
//*****************************************************************************************************************************
void waitForConnection(void)
{
	setup_data_packet_header();

	//RESET ADC and DMA:
	refresh_ADC();
	ADC_STATE = HALTED;
	//RESET DMA STATE.
	refresh_DMA();
	DMA_STATE = HALTED;

    if(currentCC3000State() & CC3000_SERVER_INIT) 	// Check whether the server functionality is successfully initialized
    {
        while(1) 									// Begin waiting for client and handle the connection
        {
            hci_unsolicited_event_handler();
            addrlen = sizeof(clientaddr);
            
            // accept blocks until we receive a connection
			while ( (clientDescriptor == -1) || (clientDescriptor == -2) )
			{
				clientDescriptor = accept(serverSocket, (sockaddr *) &clientaddr, &addrlen);
			}
            hci_unsolicited_event_handler();

            if(clientDescriptor >= 0)				// Connection Accepted, Wait for data exchange
            {
                setCC3000MachineState(CC3000_CLIENT_CONNECTED);
                unsolicicted_events_timer_disable();
                //**********************
                // Important Function. Manages incoming Message aswell as initiating outgoing message.
                incomingPacketManager();
                //**********************


			}
            else if(clientDescriptor == SOCKET_INACTIVE_ERR)
            {
				clientDescriptor = -1;
                // Reinitialize the server
                shutdownServer();
                initServer();
            }
            if(bytesRecvd < 0){check_socket_connection();} //If the recieve function goes inactive, shuts down. This checks for that case
            hci_unsolicited_event_handler();
        }
    }
}

//*****************************************************************************
//	incomingPacketManager
//This function waits for an incoming message, and then extracts the
//value from the message. If the message is an config one, then it will assign
//the config info to a global variable. The function will return the next action
//to be taken.
// Packet Structure: <Command 1 Byte><Size of data 2 bytes><Missed Data 2 bytes><Data 2 bytes>
//*****************************************************************************
void incomingPacketManager(void){

	char requestBuffer[SERVER_RECV_BUF_SIZE] = {NULL};
	bytesRecvd = recv(clientDescriptor, requestBuffer, sizeof(requestBuffer), 0);

	switch(requestBuffer[0])
	{
	case Config:
		//RESET ADC and DMA:
		refresh_ADC();
		ADC_STATE = HALTED;
		//RESET DMA STATE.
		refresh_DMA();
		DMA_STATE = HALTED;

		//Extracts the required Data from config.
		confirmationPacket_ptr = &config_data;
		*confirmationPacket_ptr = requestBuffer[6];
		confirmationPacket_ptr++;
		*confirmationPacket_ptr = requestBuffer[5];

		generalConfirmationPacket[1] = 0;					//First Byte of Data Size
		generalConfirmationPacket[2] = 2;					//Second Byte of Data Size
		generalConfirmationPacket[3] = 0;					//First Byte of Data lost
		generalConfirmationPacket[4] = 0;					//Second Byte of Data Size
		confirmationPacket_ptr =  &ammount_packets_to_be_sent;
		generalConfirmationPacket[6] = *confirmationPacket_ptr;
		confirmationPacket_ptr++;
		generalConfirmationPacket[5] = *confirmationPacket_ptr;

		bytesSent = send(clientDescriptor, (unsigned char *)generalConfirmationPacket, sizeof(generalConfirmationPacket), 0);
		turnLedOff(CC3000_SENDING_DATA_IND);
		toggleLed(CC3000_SENDING_DATA_IND);
		__delay_cycles(100000);
		toggleLed(CC3000_SENDING_DATA_IND);
		if (bytesSent != sizeof(generalConfirmationPacket)){check_socket_connection();}
		deviceconfigured = FALSE;

		break;

	case Start:
		if(deviceconfigured == FALSE){
			configure_channel(&config_data);//This is where I configure the device to operate differently.
			deviceconfigured = TRUE;
		}

		if(DMA_STATE == HALTED){							//VIMP: The stop state must stop the DMA
			DMA0CTL += DMAEN;
			DMA_STATE = RUNNING;
			__delay_cycles(1000);
		}

		if(ADC_STATE == HALTED){
			ADC10CTL0 |= ADC10ENC + ADC10SC;
			ADC_STATE = RUNNING;							//VIMP: The stop state must stop the ADC
			__delay_cycles(100000);
		}


		for(i=0; i<ammount_packets_to_be_sent; i++){

			while(DMA0_State != DONE && DMA1_State != DONE); //Waits while buffers are populated

			//This checks which buffer is full, and toggles the operation.
			if(sendingBuffer_ptr == buffer0_ptr){		//Buffer0 is fill or done
				DMA0_State = NOT_DONE;					// Reset for next usage
				DMA1CTL += DMAEN;						// Start or enable DMA1 buffer filling process
			}
			else{										//Buffer1 is fill or done
				DMA1_State = NOT_DONE;					// Reset for next usage
				DMA0CTL += DMAEN;						// Start or enable DMA0 buffer filling process
			}
			//*********************************************************************************************
			//This is a pointer manipulation method to include the missed ADC cycles into the packet header.
			sendingBuffer_ptr +=4;
			temp_ptr = &missed_samples;
			*sendingBuffer_ptr = *temp_ptr;
			temp_ptr++;
			sendingBuffer_ptr--;
			*sendingBuffer_ptr = *temp_ptr;
			sendingBuffer_ptr -=3;
			missed_samples = 0;
			//*********************************************************************************************

			if(currentCC3000State() & CC3000_CLIENT_CONNECTED){
				bytesSent = send(clientDescriptor, sendingBuffer_ptr, ammount_of_samples_in_packet+packet_header_size, 0);
				toggleLed(CC3000_SENDING_DATA_IND);
				if (bytesSent != ammount_of_samples_in_packet+packet_header_size){
					check_socket_connection();

				}
			}
			if (clientDescriptor == -1){break;}

		}
		break;

	case Stop:
		//RESET ADC:
		refresh_ADC();
		ADC_STATE = HALTED;
		//RESET DMA.
		refresh_DMA();
		DMA_STATE = HALTED;

		// The confirmation packet being sent back contains no relevant information.
		generalConfirmationPacket[1] = 0;					//First Byte of Data Size
		generalConfirmationPacket[2] = 2;					//Second Byte of Data Size
		generalConfirmationPacket[3] = 0;					//First Byte of Data lost
		generalConfirmationPacket[4] = 0;					//Second Byte of Data Size
		generalConfirmationPacket[5] = 0;
		generalConfirmationPacket[6] = 0;

		bytesSent = send(clientDescriptor, (unsigned char *)generalConfirmationPacket, sizeof(generalConfirmationPacket), 0);
		turnLedOff(CC3000_SENDING_DATA_IND);
		toggleLed(CC3000_SENDING_DATA_IND);
		__delay_cycles(100000);
		toggleLed(CC3000_SENDING_DATA_IND);
		if (bytesSent != sizeof(generalConfirmationPacket)){check_socket_connection();}

		break;
	default:
		break;

	}

}

void do_nothing(void){}


//****************************************************************************************************************************************
// This does the basic configuration of the Packet Headers.
//****************************************************************************************************************************************
void setup_data_packet_header (void){

	buffer0_ptr 		= (unsigned char *)BUFFER0_STR_ADD;
	buffer1_ptr 		= (unsigned char *)BUFFER1_STR_ADD;
	sendingBuffer_ptr 	= (unsigned char *)BUFFER0_STR_ADD;

	*buffer0_ptr = Meas_Data;
	buffer0_ptr +=2;
	temp_ptr = &ammount_of_samples_in_packet;
	*buffer0_ptr = *temp_ptr;
	temp_ptr++;
	buffer0_ptr--;
	*buffer0_ptr = *temp_ptr;
	buffer0_ptr -=1;

	*buffer1_ptr = Meas_Data;
	buffer1_ptr +=2;
	temp_ptr = &ammount_of_samples_in_packet;
	*buffer1_ptr = *temp_ptr;
	temp_ptr++;
	buffer1_ptr--;
	*buffer1_ptr = *temp_ptr;
	buffer1_ptr -=1;


	buffer0_ptr = (unsigned char *)BUFFER0_STR_ADD;
	buffer1_ptr = (unsigned char *)BUFFER1_STR_ADD;
}


//****************************************************************************************************************************************
//Check Socket Connection
//
// This function is used throughout the main loop of the code (WaitforConnection). It checks the socket connection, and then changes
// the state of the clientDescriptor. The clientDescriptor state is responsible for initiating a socket connection.
//****************************************************************************************************************************************
void check_socket_connection(void){
	curSocket =  getsockopt(clientDescriptor, SOL_SOCKET, SOCK_DGRAM , &optval, (socklen_t*)&optlen);
	if (curSocket != 0)
	{
		closesocket(clientDescriptor);
		clientDescriptor = -1;
		unsetCC3000MachineState(CC3000_CLIENT_CONNECTED);

		//RESET ADC and DMA:
		refresh_ADC();
		ADC_STATE = HALTED;
		//RESET DMA STATE.
		refresh_DMA();
		DMA_STATE = HALTED;
	}
}


//*****************************************************************************
//
//! Initialize Connection Server
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Waits for a connection where we will
//
//*****************************************************************************
void initServer(void)
{
    char portStr[12];
    memset(portStr,0,sizeof(portStr));

    // Open Server Socket
    serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (serverSocket == -1)
    {
        serverError(SERV_ERR_SOCKET);
        wlan_stop();
        while(1)
        {
            __no_operation();
        }
    }

    // Open port
    int port = SERVER_PORT;
    serverSocketAddr.sa_family = AF_INET;

    // Set the Port Number
    serverSocketAddr.sa_data[0] = (port & 0xFF00)>> 8;
    serverSocketAddr.sa_data[1] = (port & 0x00FF);

    memset (&serverSocketAddr.sa_data[2], 0, 4);

    if (bind(serverSocket, &serverSocketAddr, sizeof(sockaddr)) != 0)
    {
        serverError(SERV_ERR_BIND);
        return;
    }


    // Start Listening
    if (listen (serverSocket, 1) != 0)
    {
        serverError(SERV_ERR_LISTEN);
        return;
    }

    setCC3000MachineState(CC3000_SERVER_INIT);

    itoa(port, portStr, DECIMAL_BASE);
}

//*****************************************************************************
//
//! Shut down server sockets
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Waits for a connection where we will 
//
//*****************************************************************************
void shutdownServer()
{
     // Close the Server's Socket
    closesocket(serverSocket);
    serverSocket = 0xFFFFFFFF;        
}

//*****************************************************************************
//
//! \brief  Waits for a connection where we will 
//!
//! \param  none
//!
//! \return none
//!
//
//*****************************************************************************
void serverError(char err)
{     
     switch(err)
     {
        case SERV_ERR_SOCKET:
            serverErrorCode = SERV_ERR_SOCKET;
         break;
        case SERV_ERR_BIND:
            serverErrorCode = SERV_ERR_BIND;
         break;
        case SERV_ERR_LISTEN:
            serverErrorCode = SERV_ERR_LISTEN;
            break;
     }
     while(1)
     {
         __no_operation();
     }
}

//****************************************************************************************************************************************
//DMA Interrupt Handler
// This needs to deal with both the interrupts. When a DMA channel is done, it changes its state, and assigns itself as the next
// buffer to be sent.
//****************************************************************************************************************************************
#pragma vector=DMA_VECTOR
__interrupt void DMA0_ISR (void)
{
  switch(__even_in_range(DMAIV,16))
  {
    case  0: break;                          	// No interrupt
    case  2:
    	DMA0_State = DONE;
    	sendingBuffer_ptr = (unsigned char *)BUFFER0_STR_ADD;

      break;                                 	// DMA0IFG
    case  4:
    	DMA1_State = DONE;
    	sendingBuffer_ptr = (unsigned char *)BUFFER1_STR_ADD;

    	break;                          		// DMA1IFG
    case  6: break;                          	// DMA2IFG
    default: break;
  }
}
//*****************************************************************************************************************************
// ADC Interrupt handler. I am basically only going to use this interrupt to count the number of times that the ADC cycle
// is missed.
//*****************************************************************************************************************************
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  switch(__even_in_range(ADC10IV,12))
  {
    case  0: break;                          // No interrupt
    case  2:
    	missed_samples++;
    	break;                          // conversion result overflow
    case  4: break;                          // conversion time overflow
    case  6: break;                          // ADC10HI
    case  8: break;                          // ADC10LO
    case 10: break;                          // ADC10IN
    case 12: break;                          // Clear CPUOFF bit from 0(SR)
    default: break;
  }
}
