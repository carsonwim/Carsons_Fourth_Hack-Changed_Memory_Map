#ifndef SERVER_SETUP
#define SERVER_SETUP

enum channel
{
	channel1		= 0x3031,
	channel2		= 0x3032,
	channel3		= 0x3033,
	channel4		= 0x3034,
	channel5		= 0x3035,
	channel6		= 0x3036,
	channel7		= 0x3037,
	channel8		= 0x3038,
	channel9		= 0x3039,
	channel123		= 0x303A
};

void clear_buffers(void);
void refresh_ADC(void);
void refresh_DMA(void);
void configure_channel(unsigned long*ptr);

#endif
