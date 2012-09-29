
// Pin Definitions
#define ACC_PWR_PIN       BIT7
#define ACC_PWR_PORT_DIR  P2DIR
#define ACC_PWR_PORT_OUT  P2OUT
#define ACC_PORT_DIR      P3DIR
#define ACC_PORT_OUT      P3OUT
#define ACC_PORT_SEL0     P3SEL0
#define ACC_PORT_SEL1     P3SEL1
#define ACC_X_PIN         BIT0
#define ACC_Y_PIN         BIT1
#define ACC_Z_PIN         BIT2

// Accelerometer Input Channel Definitions
#define ACC_X_CHANNEL     ADC10INCH_12
#define ACC_Y_CHANNEL     ADC10INCH_13
#define ACC_Z_CHANNEL     ADC10INCH_14


#define X_AXIS_MEAS 2
#define Y_AXIS_MEAS 3
#define Z_AXIS_MEAS 4
#define TEMP_MEAS   5
#define VCC_MEAS    6

void SetupAccel(void);
void SetupThermistor(void);
void SetupAccelXXX(void);
void TakeADCMeas(char mtype);
void SetupAccelYYY(void);
void SetupVcc(void);
void SetupAccelZZZ(void);
void SetupThermistorADC(void);
