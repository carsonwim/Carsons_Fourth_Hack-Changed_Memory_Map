

#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include "config_defs.h"

//
// Modify the following settings as necessary to run the demo
//

#define IP_ALLOC_METHOD USE_STATIC_IP
//#define IP_ALLOC_METHOD USE_DHCP

// Default SSID Settings
#define DEFAULT_OUT_OF_BOX_SSID       		"FramRouter"
#define DEFAULT_AP_SECURITY          		3
#define DFAULT_AP_SECURITY_KEY             	"password"

#if IP_ALLOC_METHOD == USE_STATIC_IP
#define STATIC_IP_OCT1 192
#define STATIC_IP_OCT2 168
#define STATIC_IP_OCT3 1
#define STATIC_IP_OCT4 234
#endif

#define SERVER_PORT 1204
#endif
