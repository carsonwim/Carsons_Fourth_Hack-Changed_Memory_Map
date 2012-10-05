
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
void setup_parrelel_sampling (void);
void startDMAandADC (void);
void check_socket_connection(void);
void shutdownServer();
void serverError(char err);

#endif
