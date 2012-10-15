
#ifndef SERVER_H
#define SERVER_H

#define SERVER_RECV_BUF_SIZE 7
//*********************************************************************************************************************
//*********************************************************************************************************************
//*********************************************************************************************************************
enum Msg_Type
{
    Config     		= 0x46, //F = conFigeration
    Start     		= 0x53,	//S = Start
    Stop      		= 0x50,	//P = stoP
    Continue    	= 0x43, //C = Continue
    Meas_Data 		= 0x4D, //M = Measured
    Setup_Complete	= 0x44  //D = setup Done
};



//*********************************************************************************************************************
//*********************************************************************************************************************
//*********************************************************************************************************************

// SERVER ERROR
enum serverError
{
    SERV_ERR_SOCKET = 1,
    SERV_ERR_BIND = 2,
    SERV_ERR_LISTEN = 3    
};

void initServer(void);
void waitForConnection(void);
char incomingPacketManager(void);
void setup_data_packet_header (void);
//unsigned char swap_Word_Bytes(unsigned char memory_address);
void setup_parrelel_sampling (void);
void check_socket_connection(void);
void shutdownServer();
void serverError(char err);

void do_nothing(void);

#endif
