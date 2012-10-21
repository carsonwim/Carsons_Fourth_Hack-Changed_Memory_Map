/*****************************************************************************
*
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*SOCKOPT_RECV_TIMEOUT
*****************************************************************************/
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
void incomingPacketManager(void);
void setup_data_packet_header (void);
void check_socket_connection(void);
void shutdownServer();
void serverError(char err);

void do_nothing(void);

#endif
