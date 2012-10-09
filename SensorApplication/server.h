
#ifndef SERVER_H
#define SERVER_H

#define SERVER_RECV_BUF_SIZE 7
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
