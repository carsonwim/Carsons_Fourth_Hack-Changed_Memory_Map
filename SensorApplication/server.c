

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
#include "strlib.h"
#include "server.h"
#include "carsons_file.h"


long serverSocket;
sockaddr serverSocketAddr;

/** \brief Definition of data packet to be sent by server */
// Packet Structure: <Command 1 Byte><Size of data 2 bytes><Missed Data 2 bytes><Data of whatever size>
unsigned char dataPacket[] = { 0x01,0x00 ,0x05,0x00,0x00,0x01,0x02,0x03,0x04,0x05};

//unsigned char dataPacket[60] = "AAAAAAAAA1AAAAAAAAA2AAAAAAAAA3AAAAAAAAA4AAAAAAAAA5AAAAAAAAA6";

char serverErrorCode = 0;

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
//! \brief  Waits and handle a client connection
//!
//! \param  none
//!
//! \return none
//!
//
//*****************************************************************************
void waitForConnection(void)
{
	unsigned int quantity = 1460;
	unsigned char *buffer1_ptr;

	buffer1_ptr = (unsigned char *)0xF200;
	int i;
	for(i=0 ; i < quantity ; i++){
		*buffer1_ptr = 'A';
		buffer1_ptr +=1;
	}
	buffer1_ptr = (unsigned char *)0xF200;

    sockaddr clientaddr;  
    socklen_t addrlen;
    int clientDescriptor = -1;
    volatile int bytesRecvd = 0;
    int curSocket = 0;
    int bytesSent = 0;
    int optval, optlen;


    // Check whether the server functionality is successfully initialized
    if(currentCC3000State() & CC3000_SERVER_INIT)
    {
        // Begin waiting for client and handle the connection
        while(1)
        {
            hci_unsolicited_event_handler();
            addrlen = sizeof(clientaddr);
            
            // accept blocks until we receive a connection
			while ( (clientDescriptor == -1) || (clientDescriptor == -2) )
			{
				clientDescriptor = accept(serverSocket, (sockaddr *) &clientaddr, &addrlen);
			}

            hci_unsolicited_event_handler();
            
            // Call user specified Clietn Accepted Event Handler
            if(clientDescriptor >= 0)
            {
                setCC3000MachineState(CC3000_CLIENT_CONNECTED);
                // Connection Accepted, Wait for data exchange
               
                char requestBuffer[SERVER_RECV_BUF_SIZE] = {NULL};
                bytesRecvd = recv(clientDescriptor, requestBuffer, sizeof(requestBuffer), 0);

                if(currentCC3000State() & CC3000_CLIENT_CONNECTED){
					bytesSent = send(clientDescriptor, (unsigned char *)dataPacket, sizeof(dataPacket), 0);
//					bytesSent = send(clientDescriptor, buffer1_ptr, quantity, 0);
					toggleLed(CC3000_SENDING_DATA_IND);
					if (bytesSent != sizeof(dataPacket))
//					if (bytesSent != quantity)
					{	// Check if socket is still available
						curSocket =  getsockopt(clientDescriptor, SOL_SOCKET, SOCK_DGRAM , &optval, (socklen_t*)&optlen);
						if (curSocket != 0)
						{
							closesocket(clientDescriptor);
							clientDescriptor = -1;
							unsetCC3000MachineState(CC3000_CLIENT_CONNECTED);
						}
					}
                }

                // Check whether we received the DATA command from the client
                // telling us to begin sending data to it.
//                if(strncmp(requestBuffer,"DATA",strlen("DATA")) == 0)
//                {
//                    SetupAccel();
//                    SetupThermistor();
//                    while(currentCC3000State() & CC3000_CLIENT_CONNECTED)
//                    {
//                        // Start Sending Data to server
//                        // Send data to CC3000
//                        hci_unsolicited_event_handler();
//                        unsolicicted_events_timer_disable();
//                        toggleLed(CC3000_SENDING_DATA_IND);
//
////                        demoPacketCreator();
//                       bytesSent = send(clientDescriptor, buffer1_ptr, quantity, 0);
//
////                        bytesSent = send(clientDescriptor, (unsigned char *)dataPacket, sizeof(dataPacket), 0);
//                        if (bytesSent != quantity)
//                        {
//                            // Check if socket is still available
//                            curSocket =  getsockopt(clientDescriptor, SOL_SOCKET, SOCK_DGRAM , &optval, (socklen_t*)&optlen);
//                            if (curSocket != 0)
//                            {
//                                closesocket(clientDescriptor);
////                                terminalPrint("Client Disconnected\r\n");
//																clientDescriptor = -1;
//                                unsetCC3000MachineState(CC3000_CLIENT_CONNECTED);
//                            }
//                        }
//                        unsolicicted_events_timer_init();
//                        __delay_cycles(10);			//this should wait a second
//                    }
//                }
//                __delay_cycles(1000);
            }
            else if(clientDescriptor == SOCKET_INACTIVE_ERR)
            {
//                terminalPrint("Socket Server Timeout. Restarting Server\r\n");
								clientDescriptor = -1;
                // Reinitialize the server
                shutdownServer();
                initServer();
            }
            hci_unsolicited_event_handler();
        }
    }
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
