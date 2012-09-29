

//*****************************************************************************
//
//! \addtogroup link_buff_api
//! @{
//
//*****************************************************************************
#include "hci.h"
#include "spi.h"
#include "os.h"
#include "evnt_handler.h"
#include <msp430.h>


#define READ                    3
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define ASSERT_CS()          (P1OUT &= ~BIT3)

#define DEASSERT_CS()        (P1OUT |= BIT3)

#define HEADERS_SIZE_EVNT       (SPI_HEADER_SIZE + 5)

#define SPI_HEADER_SIZE			(5)

#define 	eSPI_STATE_POWERUP 				 (0)
#define 	eSPI_STATE_INITIALIZED  		 (1)
#define 	eSPI_STATE_IDLE					 (2)
#define 	eSPI_STATE_WRITE_IRQ	   		 (3)
#define 	eSPI_STATE_WRITE_FIRST_PORTION   (4)
#define 	eSPI_STATE_WRITE_EOT			 (5)
#define 	eSPI_STATE_READ_IRQ				 (6)
#define 	eSPI_STATE_READ_FIRST_PORTION	 (7)
#define 	eSPI_STATE_READ_EOT				 (8)







typedef struct
{
	gcSpiHandleRx  SPIRxHandler;

	unsigned short usTxPacketLength;
	unsigned short usRxPacketLength;
	unsigned long  ulSpiState;
	unsigned char *pTxPacket;
	unsigned char *pRxPacket;

}tSpiInformation;


tSpiInformation sSpiInformation;

//
// Static buffer for 5 bytes of SPI HEADER
//
unsigned char tSpiReadHeader[] = {READ, 0, 0, 0, 0};


void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
void SpiWriteAsync(const unsigned char *data, unsigned short size);
void SpiPauseSpi(void);
void SpiResumeSpi(void);
void SSIContReadOperation(void);

// The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
// for the purpose of detection of the overrun. The location of the memory where the magic number 
// resides shall never be written. In case it is written - the overrun occured and either recevie function
// or send function will stuck forever.
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//#pragma is used for determine the memory location for a specific variable.                            ///        ///
//__no_init is used to prevent the buffer initialization in order to prevent hardware WDT expiration    ///
// before entering to 'main()'.                                                                         ///
//for every IDE, different syntax exists :          1.   __CCS__ for CCS v5                    ///
//                                                  2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench  ///
// *CCS does not initialize variables - therefore, __no_init is not needed.                             ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef __CCS__
#pragma DATA_SECTION(spi_buffer, ".FRAM_DATA")
char spi_buffer[CC3000_RX_BUFFER_SIZE];

#elif __IAR_SYSTEMS_ICC__
#pragma location = "FRAM_DATA"
__no_init char spi_buffer[CC3000_RX_BUFFER_SIZE];
#endif



#ifdef __CCS__
#pragma DATA_SECTION(wlan_tx_buffer, ".FRAM_DATA")
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

#elif __IAR_SYSTEMS_ICC__
#pragma location = "FRAM_DATA"
__no_init unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];
#endif
//*****************************************************************************
// 
//!  This function get the reason for the GPIO interrupt and clear cooresponding
//!  interrupt flag
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  This function This function get the reason for the GPIO interrupt
//!          and clear cooresponding interrupt flag
// 
//*****************************************************************************
void
SpiCleanGPIOISR(void)
{
	SPI_IFG_PORT &= ~SPI_IRQ_PIN;
}
 


//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void
SpiClose(void)
{
	if (sSpiInformation.pRxPacket)
	{
		sSpiInformation.pRxPacket = 0;
	}

	//
	//	Disable Interrupt in GPIOA module...
	//
    tSLInformation.WlanInterruptDisable();
}


//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void 
SpiOpen(gcSpiHandleRx pfRxHandler)
{
	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

	sSpiInformation.SPIRxHandler = pfRxHandler;
	sSpiInformation.usTxPacketLength = 0;
	sSpiInformation.pTxPacket = NULL;
	sSpiInformation.pRxPacket = (unsigned char *)spi_buffer;
	sSpiInformation.usRxPacketLength = 0;
	spi_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
	wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;



	//
	// Enable interrupt on the GPIOA pin of WLAN IRQ
	//
	tSLInformation.WlanInterruptEnable();
}

//*****************************************************************************
//
//! This function: init_spi
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  initializes an SPI interface
//
//*****************************************************************************

int init_spi(void)
{
    // Select the SPI lines: MISO/MOSI on P1.6,7 
    MOSI_MISO_PORT_SEL |= (SPI_MISO_PIN + SPI_MOSI_PIN);
    MOSI_MISO_PORT_SEL2 &= (~(SPI_MISO_PIN + SPI_MOSI_PIN));
    //CLK on P2.2
    SPI_CLK_PORT_SEL |= (SPI_CLK_PIN);
    SPI_CLK_PORT_SEL2 &= ~SPI_CLK_PIN;
    
    UCB0CTLW0 |= UCSWRST;                     // **Put state machine in reset**
    UCB0CTLW0 |= (UCMST+UCSYNC+UCMSB);   	// 3-pin, 8-bit SPI master
    // Clock polarity high, MSB
    UCB0CTLW0 |= UCSSEL_2;                    // ACLK
    UCB0BR0 = 0x02;                           // /2 change to /1
    UCB0BR1 = 0;                              //
    
    UCB0CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**
    
    return(ESUCCESS);
}



//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
long
SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
    //
    // workaround for first transaction
    //
    ASSERT_CS();
	
    // Assuming we are running on 24 MHz ~50 micro delay is 1200 cycles;
    __delay_cycles(1200);
	
    // SPI writes first 4 bytes of data
    SpiWriteDataSynchronous(ucBuf, 4);

    __delay_cycles(1200);
	
    SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

    // From this point on - operate in a regular way
    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    
    DEASSERT_CS();

    return(0);
}



//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
long
SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
    unsigned char ucPad = 0;

	//
	// Figure out the total length of the packet in order to figure out if there is padding or not
	//
    if(!(usLength & 0x0001))
    {
        ucPad++;
    }


    pUserBuffer[0] = WRITE;
    pUserBuffer[1] = HI(usLength + ucPad);
    pUserBuffer[2] = LO(usLength + ucPad);
    pUserBuffer[3] = 0;
    pUserBuffer[4] = 0;

    usLength += (SPI_HEADER_SIZE + ucPad);
        
        // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
        // for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun 
        // occurred - and we will stuck here forever!
	if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}

	if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
	{
		while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED)
			;
	}
	
	if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
	{
		//
		// This is time for first TX/RX transactions over SPI: the IRQ is down - so need to send read buffer size command
		//
		SpiFirstWrite(pUserBuffer, usLength);
	}
	else 
	{
		//
		// We need to prevent here race that can occur in case 2 back to back packets are sent to the 
		// device, so the state will move to IDLE and once again to not IDLE due to IRQ
		//
		tSLInformation.WlanInterruptDisable();

		while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE)
		{
			;
		}

		
		sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
		sSpiInformation.pTxPacket = pUserBuffer;
		sSpiInformation.usTxPacketLength = usLength;
		
		//
		// Assert the CS line and wait till SSI IRQ line is active and then initialize write operation
		//
		ASSERT_CS();

		//
		// Re-enable IRQ - if it was not disabled - this is not a problem...
		//
		tSLInformation.WlanInterruptEnable();
	}


	//
	// Due to the fact that we are currently implementing a blocking situation
	// here we will wait till end of transaction
	//

	while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState)
		;
	
    return(0);
}

 


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void
SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{
	while (size)
    {   	
                    while (!(TXBufferIsEmpty()));
		UCB0TXBUF = *data;
		while (!(RXBufferIsEmpty()));
		UCB0RXBUF;
		size --;
        data++;
    }
}

//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void
SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{
	long i = 0;
    unsigned char *data_to_send = tSpiReadHeader;
	
	for (i = 0; i < size; i ++)
    {
    	while (!(TXBufferIsEmpty()));
		UCB0TXBUF = data_to_send[i];
		while (!(RXBufferIsEmpty()));
		data[i] = UCB0RXBUF;
    }
}



//*****************************************************************************
//
//! This function enter point for read flow: first we read minimal 5 SPI header bytes and 5 Event
//!	Data bytes
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void
SpiReadHeader(void)
{
	SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);
}


//*****************************************************************************
//
//! This function processes received SPI Header and in accordance with it - continues reading 
//!	the packet
//!
//!  \param  None
//!
//!  \return None
//!
//!  \brief  ...
//
//*****************************************************************************
long
SpiReadDataCont(void)
{
    long data_to_recv;
	unsigned char *evnt_buff, type;

	
    //
    //determine what type of packet we have
    //
    evnt_buff =  sSpiInformation.pRxPacket;
    data_to_recv = 0;
	STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);
	
    switch(type)
    {
        case HCI_TYPE_DATA:
        {
			//
			// We need to read the rest of data..
			//
			STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);
			if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
			{	
    	        data_to_recv++;
			}

			if (data_to_recv)
			{
            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}
            break;
        }
        case HCI_TYPE_EVNT:
        {
			// 
			// Calculate the rest length of the data
			//
            STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
			data_to_recv -= 1;
			
			// 
			// Add padding byte if needed
			//
			if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
			{
				
	            data_to_recv++;
			}
			
			if (data_to_recv)
			{
            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}

			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
            break;
        }
    }
	
    return (0);
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiPauseSpi
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void 
SpiPauseSpi(void)
{
	SPI_IRQ_PORT &= ~SPI_IRQ_PIN;
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiResumeSpi
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void 
SpiResumeSpi(void)
{
	SPI_IRQ_PORT |= SPI_IRQ_PIN;
}



//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiTriggerRxProcessing
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************
void 
SpiTriggerRxProcessing(void)
{
	//
	// Trigger Rx processing
	//
	SpiPauseSpi();
	DEASSERT_CS();
        
        // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
        // for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun 
        // occurred - and we will stuck here forever!
	if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
	{
		while (1)
			;
	}
	
	sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}

//*****************************************************************************
// 
//!  The IntSpiGPIOHandler interrupt handler
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  GPIO A interrupt handler. When the external SSI WLAN device is
//!          ready to interact with Host CPU it generates an interrupt signal.
//!          After that Host CPU has registrated this interrupt request
//!          it set the corresponding /CS in active state.
// 
//*****************************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void IntSpiGPIOHandler(void)
{
	//__bic_SR_register(GIE);
	switch(__even_in_range(P2IV,P2IV_P2IFG3))
    {
	  case P2IV_P2IFG3:
		if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
		{
			/* This means IRQ line was low call a callback of HCI Layer to inform on event */
	 		sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
		}
		else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
		{
			sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
			
			/* IRQ line goes down - we are start reception */
			ASSERT_CS();

			//
			// Wait for TX/RX Compete which will come as DMA interrupt
			// 
	        SpiReadHeader();

			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
			
			//
			//
			//
			SSIContReadOperation();
		}
		else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
		{
			SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

			sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

			DEASSERT_CS();
		}
		break;
	default:
		break;
	}

	//__bis_SR_register(GIE);
}

//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SSIContReadOperation
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void
SSIContReadOperation(void)
{
	//
	// The header was read - continue with  the payload read
	//
	if (!SpiReadDataCont())
	{
		
		
		//
		// All the data was read - finalize handling by switching to teh task
		//	and calling from task Event Handler
		//
		SpiTriggerRxProcessing();
	}
}


//*****************************************************************************
//
//! Indication if TX SPI buffer is empty
//!
//!  \param  
//!
//!  \return true or false
//!
//!  \brief  The function returns 1 if buffer is empty, 0 otherwise 
//
//*****************************************************************************

long TXBufferIsEmpty(void)
{
 return (UCB0IFG&UCTXIFG);

}

//*****************************************************************************
//
//! Indication if RX SPI buffer is empty
//!
//!  \param  
//!
//!  \return true or false
//!
//!  \brief  The function returns 1 if buffer is empty, 0 otherwise 
//
//*****************************************************************************

long RXBufferIsEmpty(void)
{
 return (UCB0IFG&UCRXIFG);
   

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
