#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include "config_defs.h"

//************************************************************
// Modify the following settings as necessary to run the demo
//************************************************************

#define IP_ALLOC_METHOD USE_STATIC_IP						//Or define IP_ALLOC_METHOD = USE_DHCP.


// Default SSID Settings
#define DEFAULT_OUT_OF_BOX_SSID       		"FramRouter"	// Name of the Wifi SSid
#define DEFAULT_AP_SECURITY          		3				// 3 for WPA2
#define DFAULT_AP_SECURITY_KEY             	"password"		// Password For Router

#if IP_ALLOC_METHOD == USE_STATIC_IP						// Specify the Static IP
#define STATIC_IP_OCT1 192
#define STATIC_IP_OCT2 168
#define STATIC_IP_OCT3 1
#define STATIC_IP_OCT4 234
#endif

#define SERVER_PORT 1204									// Incoming Port. This is a standard!!!!

#define BUFFER0_STR_ADD 0xF382;				//Allocated in the linker command file.
#define BUFFER1_STR_ADD 0xF982;				//Allocated in the linker command file.

#define AMMOUNT_OF_SAMPLES_IN_PACKET  1455	//Should be put in a config header for the device.
#define AMMOUNT_PACKETS_TO_BE_SENT 1500		//Should be put in a config header for the device.

//ADC
#define ADC_CLOCK_CYCLES ADC10SHT_0
#define ADC_CLOCK_SOURCE ADC10SSEL_0
#define ADC_CLOCK_DIVIDE ADC10DIV_0
#define ADC_REFERENCE_VOLTAGE ADC10SREF_0






#endif
								//		//RESET ADC and DMA:
								//		refresh_ADC();
								//		ADC_STATE = HALTED;
								//		//RESET DMA STATE.
								//		refresh_DMA();
								//		DMA_STATE = HALTED;
