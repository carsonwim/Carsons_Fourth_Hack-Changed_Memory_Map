/*****************************************************************************

*****************************************************************************/
//*****************************************************************************
//
//! \addtogroup common_api
//! @{
//
//*****************************************************************************

#include "cc3000_common.h"
#include "socket.h"
#include "wlan.h"
#include "evnt_handler.h"

////*****************************************************************************
////
////! void __error__(char *pcFilename, unsigned long ulLine)
////!
////!  \param  pcFilename - file name, where error occurred
////!  \param  ulLine     - line number, where error occurred
////!
////!  \return none
////!
////!  \brief stub function for ASSERT macro
////
////*****************************************************************************
//void
//__error__(char *pcFilename, unsigned long ulLine)
//{
//    //TODO full up function
//}


void 
SimpleLinkWaitEvent(unsigned short usOpcode, void *pRetParams)
{
	//
	// In the blocking implementation the control to caller will be returned only after the 
	// end of current transaction
	//
	tSLInformation.usRxEventOpcode = usOpcode;
	hci_event_handler(pRetParams, 0, 0);
}


void 
SimpleLinkWaitData(unsigned char *pBuf, unsigned char *from, unsigned char *fromlen)
{
	//
	// In the blocking implementation the control to caller will be returned only after the 
	// end of current transaction, i.e. only after data will be received
	//
	tSLInformation.usRxDataPending = 1;
	hci_event_handler(pBuf, from, fromlen);
}


//This function is used for copying 32 bit to stream while converting to little endian format.
unsigned char* UINT32_TO_STREAM_f (unsigned char *p, unsigned long u32)
{
	*(p)++ = (unsigned char)(u32);
	*(p)++ = (unsigned char)((u32) >> 8);
	*(p)++ = (unsigned char)((u32) >> 16);
	*(p)++ = (unsigned char)((u32) >> 24);
	return p;
}
//This function is used for copying 16 bit to stream while converting to little endian format.
unsigned char* UINT16_TO_STREAM_f (unsigned char *p, unsigned short u16)
{
	*(p)++ = (unsigned char)(u16);
	*(p)++ = (unsigned char)((u16) >> 8);
	return p;
}


//This function is used for copying received stream to 16 bit in little endian format.
unsigned short STREAM_TO_UINT16_f(char* p, unsigned short offset)
{
        return (unsigned short)((unsigned short)((unsigned short)(*(p + offset + 1)) << 8) + (unsigned short)(*(p + offset)));
}


//This function is used for copying received stream to 32 bit in little endian format.
unsigned long STREAM_TO_UINT32_f(char* p, unsigned short offset)
{
        return (unsigned long)((unsigned long)((unsigned long)(*(p + offset + 3)) << 24) + (unsigned long)((unsigned long)(*(p + offset + 2)) << 16) + (unsigned long)((unsigned long)(*(p + offset + 1)) << 8) + (unsigned long)(*(p + offset)));
}



//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
