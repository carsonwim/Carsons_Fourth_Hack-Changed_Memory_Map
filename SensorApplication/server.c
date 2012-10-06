

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
sockaddr clientaddr;
socklen_t addrlen;
int clientDescriptor = -1;
volatile int bytesRecvd = 0;
int curSocket = 0;
int bytesSent = 0;
int optval, optlen;


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

char incomingPacketData1; 						   							//Data from incoming config message. Will be used to setup the adc.
char incomingPacketData2;													//Data from incoming config message. Will be used to setup the adc.
char generalConfirmationPacket[] = { Setup_Complete, 'A','A', 'A','A'};		//Generalised format for a send message.
char deviceconfigured = FALSE;												//By default the device is not configured. Checked by the incoming message.
char DMA0_State = NOT_DONE;													// State of DMA register process
char DMA1_State = NOT_DONE;
char ADC_STATE = HALTED;
char DMA_STATE = HALTED;

unsigned int ammount_of_samples_in_packet = 1400;							//Should be put in a config header for the device.
unsigned int ammount_packets_to_be_sent = 1500;								//Should be put in a config header for the device.
#define SENDING_PACKETS 0x5DC
#define BUFFER0_STR_ADD (0xF400);											//Allocated in the linker command file.
#define BUFFER1_STR_ADD (0xFA00);											//Allocated in the linker command file.

//sendingBuffer_ptr = buffer0_ptr;											//Just the start Default.
int i;																		//Used in "for" loop.
unsigned int count;

void really_long_time (void);

//#ifdef __CCS__
//#pragma DATA_SECTION(buffer0_Array, ".fram_adc_buf0")
//char buffer0_Array[SENDING_PACKETS];
//
//#pragma DATA_SECTION(buffer1_Array, ".fram_adc_buf1")
//char buffer1_Array[SENDING_PACKETS];
//
//#endif
unsigned char *buffer0_ptr;
unsigned char *buffer1_ptr;
unsigned char *sendingBuffer_ptr;


/** \brief Definition of data packet to be sent by server */
// Packet Structure: <Command 1 Byte><Size of data 2 bytes><Missed Data 2 bytes><Data of whatever size>
unsigned char dataPacket[] = { 0x01,   0x05,0x00,    0x00,0x00,    0x01,0x02,0x03,0x04,0x05};
char serverErrorCode = 0;

void waitForConnection(void)
{
	setup_parrelel_sampling();
	buffer0_ptr = (unsigned char *)BUFFER0_STR_ADD;
	buffer1_ptr = (unsigned char *)BUFFER1_STR_ADD;
	sendingBuffer_ptr = buffer0_ptr;											//Just the start Default.


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
                incomingPacketManager();

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
		if(deviceconfigured == FALSE){
			deviceconfigured = TRUE; //This is where I would configure the device to operate differently
		}

		if(DMA_STATE == HALTED){							//VIMP: The stop state must stop the DMA
			DMA0CTL += DMAEN;
			DMA_STATE = RUNNING;
			__delay_cycles(100000);
		}

		if(ADC_STATE == HALTED){
			ADC10CTL0 |= ADC10ENC + ADC10SC;
			ADC_STATE = RUNNING;							//VIMP: The stop state must stop the ADC
			__delay_cycles(100000);
		}

		for(i=0; i<ammount_packets_to_be_sent; i++){

			while(DMA0_State == NOT_DONE && DMA1_State == NOT_DONE);
			if(DMA0_State == DONE || DMA1_State == DONE){
				if(sendingBuffer_ptr == buffer0_ptr){		//Buffer0 is fill or done
					DMA0_State = NOT_DONE;					// Reset for next usage
					DMA1CTL += DMAEN;						// Start or enable DMA1 buffer filling process
				}
				else{
					DMA1_State = NOT_DONE;					// Reset for next usage
					DMA0CTL += DMAEN;						// Start or enable DMA0 buffer filling process
				}
				if(currentCC3000State() & CC3000_CLIENT_CONNECTED){
					bytesSent = send(clientDescriptor, sendingBuffer_ptr, ammount_of_samples_in_packet, 0);
					toggleLed(CC3000_SENDING_DATA_IND);
					if (bytesSent != ammount_of_samples_in_packet){	check_socket_connection();}
		    	}
			}
		}
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
	  // Configure DMA0
	   DMACTL0 = DMA0TSEL__ADC10IFG + DMA1TSEL__ADC10IFG;            // ADC10IFG trigger
	  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC10MEM0);
	                                            // Source single address
	  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) 0xF400);
	                                            // Destination array address
	  DMA0SZ = ammount_of_samples_in_packet;                            // 32 conversions
	  DMA0CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL + DMASRCBYTE + DMADSTBYTE; // + DMAEN
	                                            // Rpt, inc dest, word access,
	                                            // enable int after 32 conversions
//****************************************************************************************************************************************
	  // Configure DMA1
//	   DMACTL0 += DMA1TSEL__ADC10IFG;            // ADC10IFG trigger: Note that the register is the same. Sets both Interrupts
	  __data16_write_addr((unsigned short) &DMA1SA,(unsigned long) &ADC10MEM0);
	                                            // Source single address
	  __data16_write_addr((unsigned short) &DMA1DA,(unsigned long) 0xFA00);
	                                            // Destination array address
	  DMA1SZ = ammount_of_samples_in_packet;                            // 32 conversions
	  DMA1CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL + DMASRCBYTE + DMADSTBYTE; // + DMAEN
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
    case  0: break;                          	// No interrupt
    case  2:
    	DMA0_State = DONE;
    	sendingBuffer_ptr = buffer0_ptr;
      break;                                 	// DMA0IFG
    case  4:
    	DMA1_State = DONE;
    	sendingBuffer_ptr = buffer1_ptr;
    	break;                          		// DMA1IFG
    case  6: break;                          	// DMA2IFG
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



void really_long_time (void){
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);                   // Delay between sequence convs
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);
    __delay_cycles(25000);

}
//                    P3OUT &= ~(BIT4 + BIT5 + BIT6 + BIT7);
//
//                    P3DIR |= (BIT4 + BIT5 + BIT6 + BIT7);
//                	P3OUT ^= 0xF0;
//                	DMA0CTL += DMAEN;
//                //	P3OUT ^= 0x40;
//                	really_long_time ();
//                	really_long_time ();
//                	really_long_time ();
//                	really_long_time ();
//                	really_long_time ();
//                	really_long_time ();
//                	really_long_time ();
//                	really_long_time ();
//                	really_long_time ();
//                	P3OUT ^= 0xF0;
//
//                	ADC10CTL0 |= ADC10ENC + ADC10SC;        // Start sampling
////                	__bis_SR_register( GIE);        // LPM0, ADC10_ISR will force exit
//                  while(1)
//                  {
//                    while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
//                //    ADC10CTL0 |= ADC10ENC + ADC10SC;        // Start sampling
//
//                    __no_operation();
//                    // << SET BREAKPOINT HERE
//                	P3OUT ^= 0x40;

//	  //****************************************************************************************************************************************
//	  // Configure DMA (ADC10IFG trigger)
//
//	   DMACTL0 = DMA0TSEL__ADC10IFG;            // ADC10IFG trigger
//	  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC10MEM0);
//	                                            // Source single address
//	  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &buffer0_Array);
//	                                            // Destination array address
//	  DMA0SZ = 0x578;                            // Number of conversions is 1400
//	  DMA0CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL;
//	                                            // Rpt, inc dest, word access,
//	                                            // enable int after 32 conversions
//
//	  // Configure DMA (ADC10IFG trigger)
//
//	   DMACTL0 += DMA1TSEL__ADC10IFG;            // ADC10IFG trigger
//	  __data16_write_addr((unsigned short) &DMA1SA,(unsigned long) &ADC10MEM0);
//	                                            // Source single address
//	  __data16_write_addr((unsigned short) &DMA1DA,(unsigned long) &buffer1_Array);
////	  DMA1DA = (__SFR_FARPTR) (unsigned long) 0xF500;
//	                                            // Destination array address
//	  DMA1SZ = 0x578;                            // Number of conversions is 1400
//	  DMA1CTL = DMADT_0 + DMADSTINCR_3 + DMAIE + DMALEVEL;

