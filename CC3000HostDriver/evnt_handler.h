
#ifndef __EVENT_HANDLER_H__
#define __EVENT_HANDLER_H__
#include "hci.h"
#include "socket.h"

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef  __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern unsigned char *hci_event_handler(void *pRetParams, unsigned char *from, unsigned char *fromlen);
extern long hci_unsol_event_handler(char *event_hdr);
extern long hci_unsolicited_event_handler(void);

#define M_BSD_RESP_PARAMS_OFFSET(hci_event_hdr)((char *)(hci_event_hdr) + HCI_EVENT_HEADER_SIZE)

#define SOCKET_STATUS_ACTIVE       0
#define SOCKET_STATUS_INACTIVE     1
/* Init socket_active_status = 'all ones': init all sockets with SOCKET_STATUS_INACTIVE.
   Will be changed by 'set_socket_active_status' upon 'connect' and 'accept' calls */
#define SOCKET_STATUS_INIT_VAL  0xFFFF
#define M_IS_VALID_SD(sd) ((0 <= (sd)) && ((sd) <= 7))
#define M_IS_VALID_STATUS(status) (((status) == SOCKET_STATUS_ACTIVE)||((status) == SOCKET_STATUS_INACTIVE))

extern unsigned long socket_active_status;

extern void set_socket_active_status(long Sd, long Status);
extern long get_socket_active_status(long Sd);

typedef struct _bsd_accept_return_t
{
    long             iSocketDescriptor;
    long             iStatus;
    sockaddr   		tSocketAddress;
    
} tBsdReturnParams;


typedef struct _bsd_read_return_t
{
    long             iSocketDescriptor;
    long             iNumberOfBytes;
    unsigned long	 uiFlags;
} tBsdReadReturnParams;

#define BSD_RECV_FROM_FROMLEN_OFFSET	(4)
#define BSD_RECV_FROM_FROM_OFFSET		(16)


typedef struct _bsd_select_return_t
{
    long					iStatus;
	unsigned long 			uiRdfd;
	unsigned long 			uiWrfd;
	unsigned long 			uiExfd;
} tBsdSelectRecvParams;


typedef struct _bsd_getsockopt_return_t
{
	unsigned char			ucOptValue[4];
	char						iStatus;
} tBsdGetSockOptReturnParams;

typedef struct _bsd_gethostbyname_return_t
{
    long             retVal;
    long             outputAddress;
} tBsdGethostbynameParams;

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __EVENT_HANDLER_H__
