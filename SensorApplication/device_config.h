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
#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include "common.h"

//************************************************************
// Modify the following settings as necessary to run the Device
//
//This is how the device is configured. It shows all the relevant
//settings that may need consideration.
//************************************************************

#define IP_ALLOC_METHOD USE_STATIC_IP						//Or define IP_ALLOC_METHOD = USE_DHCP.


// Default SSID Settings
#define DEFAULT_OUT_OF_BOX_SSID       		"FramRouter"	// Name of the Wifi SSid
#define DEFAULT_AP_SECURITY          		3				// 3 for WPA2
#define DFAULT_AP_SECURITY_KEY             	"aaaaaaaa"		// Password For Router

#if IP_ALLOC_METHOD == USE_STATIC_IP						// Specify the Static IP
#define STATIC_IP_OCT1 192
#define STATIC_IP_OCT2 168
#define STATIC_IP_OCT3 1
#define STATIC_IP_OCT4 235
#endif

#define SERVER_PORT 1204									// Incoming Port. This is a standard!!!!

#define BUFFER0_STR_ADD 0xF382;				//Allocated in the linker command file.
#define BUFFER1_STR_ADD 0xF982;				//Allocated in the linker command file.

#define AMMOUNT_OF_SAMPLES_IN_PACKET  1455	//Should be put in a config header for the device.
#define AMMOUNT_PACKETS_TO_BE_SENT 1500		//Should be put in a config header for the device.

//ADC
#define ADC_CLOCK_CYCLES ADC10SHT_5
#define ADC_CLOCK_SOURCE ADC10SSEL_0  // Auxillary Clock
#define ADC_CLOCK_DIVIDE ADC10DIV_0
#define ADC_REFERENCE_VOLTAGE ADC10SREF_0

#endif

