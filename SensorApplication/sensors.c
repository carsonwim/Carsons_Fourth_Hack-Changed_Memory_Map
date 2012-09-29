
#include "msp430fr5739.h"
#include "sensors.h"
#include "carsons_file.h"


volatile unsigned int XValue = 0;
volatile unsigned int YValue = 0;
volatile unsigned int ZValue = 0;
volatile unsigned int VccValue = 0;
volatile  int TempValue = 0;
volatile  int ActualTemp = 0;
volatile  int ActualTempFa = 0;

//int dataCount = 0;
unsigned char SampleAdcDone = 0;

// NTC Calibration tables
signed int  ntc_table_adc[25] = {888, 842, 788, 726, 658, 586, 512, 442, 375, 316, 263, 218, 179, 148, 121, 100, 82, 67, 56, 46, 38, 32, 27, 22, 19};
signed int  ntc_table_temp[25] = {-35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85};

extern unsigned char dataPacket[];

//*****************************************************************************
//
//! Setup Accelerometer
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  Sets up ADC for sampling the Accelerometer
//
//*****************************************************************************
void SetupAccel(void)
{
    //Setup  accelerometer
    // ~20KHz sampling
    //Configure GPIO
    // P3.0,P3.1 and P3.2 are accelerometer inputs
    P3OUT &= ~(BIT0 + BIT1 + BIT2);   
    P3DIR &= ~(BIT0 + BIT1 + BIT2); 
    P3REN |= BIT0 + BIT1 + BIT2;
    
    ACC_PORT_SEL0 |= ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN;    //Enable A/D channel inputs
    ACC_PORT_SEL1 |= ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN;
    ACC_PORT_DIR &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
    ACC_PWR_PORT_DIR |= ACC_PWR_PIN;              //Enable ACC_POWER
    ACC_PWR_PORT_OUT |= ACC_PWR_PIN;
    
    // Allow the accelerometer to settle before sampling any data 
    __delay_cycles(200000);    
}

//*****************************************************************************
//
//! Setup Thermistor
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  Sets up ADC for sampling the Thermistor
//
//*****************************************************************************
void SetupThermistor(void)
{
    // ~16KHz sampling
    //Turn on Power
    P2DIR |= BIT7;
    P2OUT |= BIT7;
    
    // Configure ADC
    P1SEL1 |= BIT4;  
    P1SEL0 |= BIT4; 
    
    // Allow for settling delay 
    __delay_cycles(50000);
    
}

//*****************************************************************************
//
//! Setup Accelerometer X Axis
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  Sets up ADC for sampling the Accelerometer's X Axis
//
//*****************************************************************************
void SetupAccelXXX(void)
{  
    //Setup  accelerometer
    // ~20KHz sampling
    //Configure GPIO
    // P3.0,P3.1 and P3.2 are accelerometer inputs
    ADC10CTL0 &= ~ADC10ENC;                        // Ensure ENC is clear
    ADC10CTL0 = ADC10ON + ADC10SHT_5;// + ADC10MSC;  
    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_0;  
    ADC10CTL2 = ADC10RES;    
    
    ADC10MCTL0 = ADC10INCH_12;
    ADC10IV = 0x00;                          // Clear all ADC12 channel int flags  
    //ADC10IE |= ADC10IE0;
    
    
}

//*****************************************************************************
//
//! Setup Accelerometer Y Axis
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  Sets up ADC for sampling the Accelerometer's Y Axis
//
//*****************************************************************************
void SetupAccelYYY(void)
{  
    ADC10CTL0 &= ~ADC10ENC;                        // Ensure ENC is clear
    ADC10CTL0 = ADC10ON + ADC10SHT_5;// + ADC10MSC;  
    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_0;  
    ADC10CTL2 = ADC10RES;    
    
    ADC10MCTL0 = ADC10INCH_13;
    ADC10IV = 0x00;                          // Clear all ADC12 channel int flags  
}

//*****************************************************************************
//
//! Setup Accelerometer Z Axis
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  Sets up ADC for sampling the Accelerometer's Z Axis
//
//*****************************************************************************
void SetupAccelZZZ(void)
{  
    //Setup  accelerometer
    // ~20KHz sampling
    //Configure GPIO
    ADC10CTL0 &= ~ADC10ENC;                  // Ensure ENC is clear
    ADC10CTL0 = ADC10ON + ADC10SHT_5;        // + ADC10MSC;  
    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_0;  
    ADC10CTL2 = ADC10RES;    
    
    ADC10MCTL0 = ADC10INCH_14;
    ADC10IV = 0x00;                          // Clear all ADC12 channel int flags      
}

//*****************************************************************************
//
//! Takes ADC Measurement
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief  Obtains and stores the relevant ADC measurement in data buffer
//
//*****************************************************************************
void TakeADCMeas(char mtype)
{  
    SampleAdcDone = 0; 

    ADC10IFG = 0;
    ADC10CTL0 &= ~ADC10ENC;
    ADC10CTL0 &= ~ADC10SC;
    ADC10CTL0 |= ADC10ENC + ADC10SC ;       // Start conversion 
    
    while (ADC10CTL1 & BUSY);
    
    if (mtype == X_AXIS_MEAS)
    {
        XValue = ADC10MEM0>>3; //Reduced to 7-bit to Xfer to PC GUI, previously 8bits        
        dataPacket[X_AXIS_MEAS] = XValue;
    }
    
    if (mtype == Y_AXIS_MEAS)
    {
        YValue = ADC10MEM0>>3; //Reduced to 7-bit to Xfer to PC GUI, previously 8bits
        dataPacket[Y_AXIS_MEAS] = YValue;
    }
    
    if (mtype == Z_AXIS_MEAS)
    {
        ZValue = ADC10MEM0>>3; //Reduced to 7-bit to Xfer to PC GUI, previously 8bits
        dataPacket[Z_AXIS_MEAS] = ZValue;
    }
    
    if (mtype == TEMP_MEAS)
    {        
        TempValue = ADC10MEM0;        
        
        int i;
        for (i =0; i<40; i++)
            if (TempValue <= ntc_table_adc[i])
                if (TempValue >= ntc_table_adc[i+1])
                {
                    ActualTemp = ntc_table_temp[i+1] + (int)
                      ( 
                  ((float) (ntc_table_temp[i] - ntc_table_temp[i+1]) )/
                  ((float) ( ntc_table_adc[i] - ntc_table_adc[i+1]))*
                  ((float) ( ADC10MEM0 - ntc_table_adc[i+1])));
                    break;
                }
        
        ActualTempFa = (int)((float)ActualTemp*(float)1.8) + 32;
        dataPacket[TEMP_MEAS] = ActualTempFa;         
    }
    
    if (mtype == VCC_MEAS)
    {
        VccValue = (10*ADC10MEM0)>>8;      // This formula holds because ADC measures Vcc/2, the reference voltage is 2 volts, and finally,
        // we set it up so that the Vcc would show up as a number between 0 and 40 (not 0 - 4).
        
        dataPacket[VCC_MEAS] = VccValue;
    }  
    __no_operation();                       // For debug only
}



void SetupVcc(void)
{  
    
    // Configure ADC10 - Pulse sample mode; ADC10SC trigger
    ADC10CTL0 &= ~ADC10ENC;                        // Ensure ENC is clear
    ADC10CTL0 = ADC10SHT_5 + ADC10ON;         // 16 ADC10CLKs; ADC ON
    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_0;     // s/w trig, single ch/conv
    ADC10CTL2 = ADC10RES;                     // 10-bit conversion results
    ADC10MCTL0 = ADC10SREF_1 + ADC10INCH_11;  // AVcc/2    
    
    // Configure internal reference
    while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
    REFCTL0 |= REFVSEL_1+REFON;               // Select internal ref = 1.5V 
    // Internal Reference ON   
    __delay_cycles(10000);                      // Delay for Ref to settle
    
}



void SetupThermistorADC(void)
{    
    // Configure ADC
    ADC10CTL0 &= ~ADC10ENC; 
    ADC10CTL0 = ADC10SHT_7 + ADC10ON;        // ADC10ON, S&H=192 ADC clks
    // ADCCLK = MODOSC = 5MHz
    ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10SSEL_0; 
    ADC10CTL2 = ADC10RES;                    // 10-bit conversion results
    ADC10MCTL0 = ADC10INCH_4;                // A4 ADC input select; Vref=AVCC
}
