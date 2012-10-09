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
#include "demo_config.h"
#include "sensors.h"
#include "board.h"
#include "strlib.h"
#include "server.h"
#include "carsons_file.h"

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
enum Msg_Type
{
    Config     		= 0x46, //F = conFigeration
    Start     		= 0x53,	//S = Start
    Stop      		= 0x50,	//P stoP
    Continue    	= 0x43, //C Continue
    Meas_Data 		= 0x4D, //Measured
    Setup_Complete	= 0x44  //setup Done
};
char generalConfirmationPacket[] = { Setup_Complete, 'A','A', 'A','A','A','A'};		//Generalised format for a send message.

char incomingPacketData1; 					//Data from incoming config message. Will be used to setup the adc.
char incomingPacketData2;					//Data from incoming config message. Will be used to setup the adc.

char deviceconfigured = FALSE;				//By default the device is not configured. Checked by the incoming message.
char DMA0_State = NOT_DONE;					// State of DMA register process
char DMA1_State = NOT_DONE;
char ADC_STATE = HALTED;
char DMA_STATE = HALTED;

unsigned long ammount_of_samples_in_packet = 1455;	//Should be put in a config header for the device.
unsigned long ammount_packets_to_be_sent = 1500;		//Should be put in a config header for the device.
unsigned char temp_word_holder[2];
unsigned char *temp_ptr;
#define BUFFER0_STR_ADD 0xF382;				//Allocated in the linker command file.
#define BUFFER1_STR_ADD 0xF982;				//Allocated in the linker command file.

int i;										//Used in "for" loop.
unsigned long missed_samples =0;

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
	setup_parrelel_sampling();
	setup_data_packet_header();

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
//                terminalPrint("Socket Server Timeout. Restarting Server\r\n");
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
char incomingPacketManager(void){

	char requestBuffer[SERVER_RECV_BUF_SIZE] = {NULL};
	bytesRecvd = recv(clientDescriptor, requestBuffer, sizeof(requestBuffer), 0);

	switch(requestBuffer[0])
	{
	case Config:
		incomingPacketData1 = requestBuffer[5];
		incomingPacketData2 = requestBuffer[6];
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
			deviceconfigured = TRUE; //This is where I would configure the device to operate differently.
			// Consider setting up the ADC here, and creating a function that does this.
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

			if(DMA0_State == DONE || DMA1_State == DONE){	// VIMP: This may be a redundant check.
				// One of them will be done, because the buffers are waiting to be sent.
				//Safer to keep this here.


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
					if (bytesSent != ammount_of_samples_in_packet+packet_header_size){	check_socket_connection();}
		    	}
			}
		}
		break;
	case Stop:
		break;
	case Continue:
		break;

	}


	return requestBuffer[0];

}
void do_nothing(void){}


//****************************************************************************************************************************************
// This does the basic configeration of the Packet Headers.
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
// This sets up the ports for use.  Must still start the ADC  and DMA's. Remember to turn these off when you are done! DMA is not enabled.
//****************************************************************************************************************************************
void setup_parrelel_sampling (void){
//****************************************************************************************************************************************
	//Setup the Accelerometer:
    P3OUT &= ~(BIT0); 													// P3.0,P3.1 and P3.2 are accelerometer inputs
    P3DIR &= ~(BIT0);
    P3REN |= BIT0;

    ACC_PORT_SEL0 		|= ACC_X_PIN ;    								//Enable A/D channel inputs
    ACC_PORT_SEL1 		|= ACC_X_PIN ;
    ACC_PORT_DIR 		&= ~(ACC_X_PIN );
    ACC_PWR_PORT_DIR 	|= ACC_PWR_PIN;   								//Enable ACC_POWER
    ACC_PWR_PORT_OUT 	|= ACC_PWR_PIN;


    __delay_cycles(200000); 											// Allow the accelerometer to settle before sampling any data
//****************************************************************************************************************************************
    //Config the ADC
    ADC10CTL0 = ADC10SHT_3 + ADC10MSC + ADC10ON;               			//32 Clock Cycles in sample, Continous Saample, Adc On
    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_2 + ADC10SSEL_2 + ADC10DIV_0;  	//Repeat Single Channel, Sm Clock, Divide clock by 1
    ADC10CTL2 &= ~ADC10RES;												//8 bit resolution
    ADC10MCTL0 = ADC10INCH_12 + ADC10SREF_1;  							// Vref+, Channel A12
    //VIMP!!! I am going to turn on an interrupt for when a value is not collected!
	ADC10IE |= ADC10OVIE;					// Turn ADC interrupt on. This will catch the missed samples.


//****************************************************************************************************************************************
    // Configure internal reference
	while(REFCTL0 & REFGENBUSY);              							// If ref generator busy, WAIT
	REFCTL0 |= REFVSEL_3+REFON;               							// Select internal ref = 2.5V
	                                            						// Internal Reference ON
	__delay_cycles(75);                       							// Delay (~75us) for Ref to settle
//****************************************************************************************************************************************
	  // Configure DMA0
	   DMACTL0 = DMA0TSEL__ADC10IFG + DMA1TSEL__ADC10IFG;            	// ADC10IFG trigger - Set for both DMA Channels
	  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC10MEM0); 		// Source single address
	  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) (buffer0_ptr + packet_header_size));	// Destination array address
	  DMA0SZ = ammount_of_samples_in_packet;                            // Sets the ammount of transfers to be completed - Set above.
	  DMA0CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL + DMASRCBYTE + DMADSTBYTE;
	  // Single transfer process with DM0SZ itterations, inc dest, byte access in both registers,

//****************************************************************************************************************************************
	  // Configure DMA1
	  __data16_write_addr((unsigned short) &DMA1SA,(unsigned long) &ADC10MEM0);			// Source single address
	  __data16_write_addr((unsigned short) &DMA1DA,(unsigned long) (buffer1_ptr + packet_header_size));	// Destination array address
	  DMA1SZ = ammount_of_samples_in_packet;                            // Sets the ammount of transfers to be completed - Set above.
	  DMA1CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL + DMASRCBYTE + DMADSTBYTE;
	  // Single transfer process with DM1SZ itterations, inc dest, byte access in both registers,
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
