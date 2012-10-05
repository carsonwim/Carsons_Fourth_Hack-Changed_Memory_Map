

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

//New Variable Declaration
enum Msg_Type
{
    Config     		= 0x41,
    Start     		= 0x42,
    Stop      		= 0x43,
    Continue    	= 0x44,
    Meas_Data 		= 0x45,
    Setup_Complete	= 0x46
};

//char *packetPtr;
char incomingPacketData1;
char incomingPacketData2;
char generalConfirmationPacket[] = { Setup_Complete, 'A','A', 'A','A'};
char deviceconfigured = FALSE;
char DMA0_State = NOT_DONE;
unsigned int ammount_of_samples_in_packet = 1400;
unsigned int ammount_packets_to_be_sent = 1500;
unsigned char *buffer0_ptr;
#define BUFFER0_STR_ADD (0xF300);
unsigned char* buffer0_ptr = (unsigned char*)BUFFER0_STR_ADD;
int i;

sockaddr clientaddr;
socklen_t addrlen;
int clientDescriptor = -1;
volatile int bytesRecvd = 0;
int curSocket = 0;
int bytesSent = 0;
int optval, optlen;



/** \brief Definition of data packet to be sent by server */
// Packet Structure: <Command 1 Byte><Size of data 2 bytes><Missed Data 2 bytes><Data of whatever size>
unsigned char dataPacket[] = { 0x01,   0x05,0x00,    0x00,0x00,    0x01,0x02,0x03,0x04,0x05};

//unsigned char dataPacket[60] = "AAAAAAAAA1AAAAAAAAA2AAAAAAAAA3AAAAAAAAA4AAAAAAAAA5AAAAAAAAA6";

char serverErrorCode = 0;

void waitForConnection(void)
{
	setup_parrelel_sampling();
//	unsigned int quantity = 1460;
//	unsigned char *buffer1_ptr;
//
//	buffer1_ptr = (unsigned char *)0xF200;
//	int i;
//	for(i=0 ; i < quantity ; i++){
//		*buffer1_ptr = 'A';
//		buffer1_ptr +=1;
//	}
//	buffer1_ptr = (unsigned char *)0xF200;

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
            


            if(clientDescriptor >= 0)// Connection Accepted, Wait for data exchange
            {
                setCC3000MachineState(CC3000_CLIENT_CONNECTED);


                unsolicicted_events_timer_disable();

                if (incomingPacketManager() == Start){    // Here is where I must check if I can send again.

                	if(deviceconfigured == FALSE){
                		deviceconfigured = TRUE; //This is where I would configure the device to operate differently
                	}


                	for(i=0; i<ammount_packets_to_be_sent; i++){
                		startDMAandADC();


						while(DMA0_State &= NOT_DONE);
						DMA0_State = NOT_DONE;

						if(currentCC3000State() & CC3000_CLIENT_CONNECTED){
							bytesSent = send(clientDescriptor, buffer0_ptr, ammount_of_samples_in_packet, 0);
							toggleLed(CC3000_SENDING_DATA_IND);
							if (bytesSent != ammount_of_samples_in_packet)
							{	// Check if socket is still available
								check_socket_connection();
							}
						}
                	}
                }


            }
            else if(clientDescriptor == SOCKET_INACTIVE_ERR)
            {
//                terminalPrint("Socket Server Timeout. Restarting Server\r\n");
				clientDescriptor = -1;
                // Reinitialize the server
                shutdownServer();
                initServer();
            }

            if(bytesRecvd < 0){check_socket_connection();}
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
char incomingPacketManager(void){

	char requestBuffer[SERVER_RECV_BUF_SIZE] = {NULL};
	bytesRecvd = recv(clientDescriptor, requestBuffer, sizeof(requestBuffer), 0);

	switch(requestBuffer[0])
	{
	case Config:
		incomingPacketData1 = requestBuffer[5];
		incomingPacketData2 = requestBuffer[6];
		bytesSent = send(clientDescriptor, (unsigned char *)generalConfirmationPacket, sizeof(generalConfirmationPacket), 0);
		turnLedOff(CC3000_SENDING_DATA_IND);
		toggleLed(CC3000_SENDING_DATA_IND);
		__delay_cycles(100000);
		toggleLed(CC3000_SENDING_DATA_IND);
		if (bytesSent != sizeof(generalConfirmationPacket))
		{	// Check if socket is still available
			check_socket_connection();
		}
		deviceconfigured = FALSE;
		break;
	case Start:
		break;
	case Stop:
		break;
	case Continue:
		break;

	}

//	if (bytesRecvd)

	return requestBuffer[0];

}
//****************************************************************************************************************************************
// This sets up the ports for use.  Must still start the ADC  and DMA's. Remember to turn these off when you are done! DMA is not enabled.
//****************************************************************************************************************************************
void setup_parrelel_sampling (void){
	//****************************************************************************************************************************************
	//Setup  accelerometer
    // ~20KHz sampling
    //Configure GPIO
    // P3.0,P3.1 and P3.2 are accelerometer inputs
    P3OUT &= ~(BIT0);
    P3DIR &= ~(BIT0);
    P3REN |= BIT0;

    ACC_PORT_SEL0 |= ACC_X_PIN ;    //Enable A/D channel inputs
    ACC_PORT_SEL1 |= ACC_X_PIN ;
    ACC_PORT_DIR &= ~(ACC_X_PIN );
    ACC_PWR_PORT_DIR |= ACC_PWR_PIN;              //Enable ACC_POWER
    ACC_PWR_PORT_OUT |= ACC_PWR_PIN;

    // Allow the accelerometer to settle before sampling any data
    __delay_cycles(200000);
	  //****************************************************************************************************************************************
	  //Config the ADC
	  // Configure ADC10;
	  ADC10CTL0 = ADC10SHT_3 + ADC10MSC + ADC10ON;                    					//32 Clock Cycles in sample, Continous Saample, Adc On
	  ADC10CTL1 = ADC10SHP + ADC10CONSEQ_2 + ADC10SSEL_2 + ADC10DIV_0;               	//Repeat Single Channel, Sm Clock, Divide clock by 2
	                                            // Sampling timer, rpt single ch
	//  ADC10CTL2 = ADC10RES;                     // 10-bit resolution
	  ADC10CTL2 &= ~ADC10RES;
	  ADC10MCTL0 = ADC10INCH_12 + ADC10SREF_1;  // Vref+, A12

	 //****************************************************************************************************************************************
	 // Configure internal reference
	  while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
	  REFCTL0 |= REFVSEL_3+REFON;               // Select internal ref = 2.5V
	                                            // Internal Reference ON
	  __delay_cycles(75);                       // Delay (~75us) for Ref to settle
	  //****************************************************************************************************************************************
	  // Configure DMA
	   DMACTL0 = DMA0TSEL__ADC10IFG;            // ADC10IFG trigger
	  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC10MEM0);
	                                            // Source single address
	  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) 0xF200);
	                                            // Destination array address
	  DMA0SZ = ammount_of_samples_in_packet;                            // 32 conversions
	  DMA0CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL; // + DMAEN
	                                            // Rpt, inc dest, word access,
	                                            // enable int after 32 conversions


}

//****************************************************************************************************************************************
//Start DMA and ADC
//****************************************************************************************************************************************

void startDMAandADC (void){
	DMA0CTL += DMAEN;
	ADC10CTL0 |= ADC10ENC + ADC10SC;        // Start sampling

}

//****************************************************************************************************************************************
//Check Socket Connection
//****************************************************************************************************************************************
void check_socket_connection(void){
	curSocket =  getsockopt(clientDescriptor, SOL_SOCKET, SOCK_DGRAM , &optval, (socklen_t*)&optlen);
	if (curSocket != 0)
	{
		closesocket(clientDescriptor);
		clientDescriptor = -1;
		unsetCC3000MachineState(CC3000_CLIENT_CONNECTED);
	}
}
//****************************************************************************************************************************************
//DMA Interupt Handler
//****************************************************************************************************************************************

#pragma vector=DMA_VECTOR
__interrupt void DMA0_ISR (void)
{
  switch(__even_in_range(DMAIV,16))
  {
    case  0: break;                          // No interrupt
    case  2:
    	ADC10CTL0 &= ~ADC10ENC;
    	DMA0_State = DONE;
      break;                                 // DMA0IFG
    case  4: break;                          // DMA1IFG
    case  6: break;                          // DMA2IFG
    case  8: break;                          // Reserved
    case 10: break;                          // Reserved
    case 12: break;                          // Reserved
    case 14: break;                          // Reserved
    case 16: break;                          // Reserved
    default: break;
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
