#ifndef SERVER_SETUP
#define SERVER_SETUP

enum channel
{
	channel1		= 0x3031,
	channel2		= 0x3032,
	channel3		= 0x3033,
	channel4		= 0x0008,
	channel5		= 0x0010,
	channel6		= 0x0020,
	channel7		= 0x0040,
	channel8		= 0x0080,
	channel9		= 0x0100,
	channel123		= 0x0007,
};

void clear_buffers(void);
void refresh_ADC(void);
void refresh_DMA(void);
void configure_channel(unsigned long*ptr);

#endif
