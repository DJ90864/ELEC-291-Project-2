
#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz) see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK

#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)


// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 22050L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

#define PWM_FREQ    200000L
#define DUTY_CYCLE  50

#define SET_CS LATBbits.LATB0=1
#define CLR_CS LATBbits.LATB0=0

#define Buzz_ON   LATBbits.LATB12=1
#define Buzz_OFF  LATBbits.LATB12=0

#define Small_Ferr  1
#define Large_Ferr  2
#define Small_NFerr 3
#define Large_NFerr 4

#define TRUE 1
#define FALSE 0


/* Pinout for DIP28 PIC32MX130:

                                   MCLR (1   28) AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 (2   27) AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 (3   26) AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 (4   25) CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 (5   24) AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 (6   23) AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 (7   22) PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS (8   21) PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 (9   20) VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 (10  19) VSS
                         SOSCI/RPB4/RB4 (11  18) TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 (12  17) TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD (13  16) TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 (14  15) PGEC3/RPB6/PMD6/RB6
*/

// Flash memory commands
#define WRITE_ENABLE     0x06  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define WRITE_DISABLE    0x04  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define READ_STATUS      0x05  // Address:0 Dummy:0 Num:1 to infinite fMax: 32MHz
#define READ_BYTES       0x03  // Address:3 Dummy:0 Num:1 to infinite fMax: 20MHz
#define READ_SILICON_ID  0xab  // Address:0 Dummy:3 Num:1 to infinite fMax: 32MHz
#define FAST_READ        0x0b  // Address:3 Dummy:1 Num:1 to infinite fMax: 40MHz
#define WRITE_STATUS     0x01  // Address:0 Dummy:0 Num:1 fMax: 25MHz
#define WRITE_BYTES      0x02  // Address:3 Dummy:0 Num:1 to 256 fMax: 25MHz
#define ERASE_ALL        0xc7  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define ERASE_BLOCK      0xd8  // Address:3 Dummy:0 Num:0 fMax: 25MHz
#define READ_DEVICE_ID   0x9f  // Address:0 Dummy:2 Num:1 to infinite fMax: 25MHz


// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

// Uses Timer4 to delay <us> microseconds
void Timer4us(unsigned char t) 
{
     T4CON = 0x8000; // enable Timer4, source PBCLK, 1:1 prescaler
    // delay 100us per loop until less than 100us remain
    while( t >= 100)
    {
        t-=100;
        TMR4 = 0;
        while( TMR4 < SYSCLK/10000);
    }
    // delay 10us per loop until less than 10us remain
    while( t >= 10)
    {
        t-=10;
        TMR4 = 0;
        while( TMR4 < SYSCLK/100000);
    }
    // delay 1us per loop until finished
    while( t > 0)
    {
        t--;
        TMR4 = 0;
        while( TMR4 < SYSCLK/1000000);
    }
    // turn off Timer4 so function is self-contained
    T4CONCLR = 0x8000;
}


// Initially from here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/pwm
void Init_pwm (void)
{
    // OC1 can be assigned to PA0, PB3, PB4, PB15, and PB7(in use).
    // Check TABLE 11-2: OUTPUT PIN SELECTION in datasheet.
    // Set OC1 to pin PA0 (pin 2 of DIP 28) with peripheral pin select
    RPA0Rbits.RPA0R = 0x0005;
 
    // Configure standard PWM mode for output compare module 1
    OC1CON = 0x0006; 
 
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] – 1
    PR2 = (SYSCLK / (PWM_FREQ*1)) - 1;
 
    // A write to OCxRS configures the duty cycle
    // : OCxRS / PRy = duty cycle
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);

 	T2CON = 0x0;
    T2CONSET = 0x8000;      // Enable Timer2, prescaler 1:1
	T2CONbits.TCKPS=0x0; // Set pre-scaler to 1
    OC1CONSET = 0x8000;     // Enable Output Compare Module 1
}

void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

// SPI Flash Memory connections:
// RA1  (pin 3)   (MISO) -> Pin 5 of 25Q32
// RB1  (pin 5)   (MOSI) -> Pin 2 of 25Q32
// RB14 (pin 25)  (SCLK) -> Pin 6 of 25Q32
// RB0  (pin 4)   (CSn)  -> Pin 1 of 25Q32
// 3.3V: connected to pins 3, 7, and 8
// GND:  connected to pin 4

void config_SPI(void)
{
	int rData;

	// SDI1 can be assigned to any of these pins (table TABLE 11-1: INPUT PIN SELECTION):
	//0000 = RPA1; 0001 = RPB5; 0010 = RPB1; 0011 = RPB11; 0100 = RPB8
	SDI1Rbits.SDI1R=0b0010; //SET SDI1 to RB1, pin 5 of DIP28
    ANSELB &= ~(1<<1); // Set RB1 as a digital I/O
    TRISB |= (1<<1);   // configure pin RB1 as output
	
	// SDO1 can be configured to any of these pins by writting 0b0011 to the corresponding register.
	// Check TABLE 11-2: OUTPUT PIN SELECTION (assuming the pin exists in the dip28 package): 
	// RPA1, RPB5, RPB1, RPB11, RPB8, RPA8, RPC8, RPA9, RPA2, RPB6, RPA4
	// RPB13, RPB2, RPC6, RPC1, RPC3
	RPA1Rbits.RPA1R=0b0011; // config RA1 (pin 3) for SD01
	
	// SCK1 is assigned to pin 25 and can not be changed, but it MUST be configured as digital I/O
	// because it is configured as analog input by default.
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB |= (1<<14);   // configure RB14 as output
    
    // CSn is assigned to RB0, pin 4.  Also onfigure as digital output pin.
    ANSELB &= ~(1<<0); // Set RB0 as a digital I/O
	TRISBbits.TRISB0 = 0;
	LATBbits.LATB0 = 1;	

	SPI1CON = 0; // Stops and resets the SPI1.
	rData=SPI1BUF; // clears the receive buffer
	SPI1STATCLR=0x40; // clear the Overflow
	SPI1CON=0x10008120; // SPI ON, 8 bits transfer, SMP=1, Master,  SPI mode unknown (looks like 0,0)
	SPI1BRG=8; // About 2.4MHz clock frequency
}

unsigned char SPIWrite(unsigned char a)
{
	SPI1BUF = a; // write to buffer for TX
	while(SPI1STATbits.SPIRBF==0); // wait for transfer complete
	return SPI1BUF; // read the received value
}

void Set_pwm (unsigned char val)
{
	OC1RS = (PR2 + 1) * ((float)val / 256.0);
}

#define PIN_PERIOD (PORTB&(1<<5))

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod(int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;

	_CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD != 0) // Wait for square wave to be 0
	{
		if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
	}

	_CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD == 0) // Wait for square wave to be 1
	{
		if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
	}

	_CP0_SET_COUNT(0); // resets the core timer count
	for (i = 0; i < n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD != 0) // Wait for square wave to be 0
		{
			if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
		}
		while (PIN_PERIOD == 0) // Wait for square wave to be 1
		{
			if (_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}


float Calc_Freq(void)
{	float freq;
	float count;
	float T;
	

	count = GetPeriod(500);
	T = (count * 2.0) / ((float)SYSCLK * 1000.0);
	freq = 1.0/T;
	
	return freq;
}


void play_from(unsigned long int address, unsigned long int playcnt)
{
	unsigned char c;
	
	CLR_CS; // Enable 25Q32 SPI flash memory.
    SPIWrite(READ_BYTES);
    SPIWrite((unsigned char)((address>>16)&0xff));
    SPIWrite((unsigned char)((address>>8)&0xff));
    SPIWrite((unsigned char)(address&0xff));
     
  while(playcnt>=1){
    
    c=SPIWrite(0x00);
	Set_pwm(c); // Output value to PWM (used as DAC)
	Timer4us(29); //this dealy is required to maintain the 22050 Hz frequency for the sound.
	playcnt--;
  }
  	SET_CS;
}


void play(unsigned int i)
{	
	int addr;
	int nbytes;
	unsigned long int wav_index[]={
    0x00002d, // "You have found a "
    0x00502a, // "Small Ferrous Metal "
    0x00d9a3, // "Large Ferrous Metal "
    0x015e18, // "Small Non-Ferrous Metal "
    0x01f9b5, // "Large Non-Ferrous Metal "
    0x028fcd
};
	addr = wav_index[0];
	nbytes = wav_index[1]-wav_index[0];
	play_from(addr, nbytes);
		
	addr = wav_index[i];
	nbytes = wav_index[i+1]-wav_index[i];
	play_from(addr, nbytes);
}


void play_buzzer( unsigned int frequency)
{
	unsigned int half_period = 1000/frequency;
	
	
		Buzz_ON;
		waitms(half_period);

		Buzz_OFF;
		waitms(half_period);
		
		Buzz_ON;
		waitms(half_period);

		Buzz_OFF;
		waitms(half_period);
		
}


int Freq_stable(void)
{
	float Base_freq;
	float r;
	int i = 0;
	
	
	Base_freq = Calc_Freq();
	r = Base_freq;
	
	waitms(500);
	
	Base_freq = Calc_Freq();
	
	while(r >= Base_freq-10 && r <= Base_freq+10){
		
		i++;
		r = Base_freq;
		waitms(500);
		Base_freq = Calc_Freq();
		
		if(i >= 3)exit(1);
	}
	
	if(i>=3){
		return 1;
	}else{
		return 0;
	}	

}


void Initialize_SYS(void)
{
	DDPCON = 0;
	CFGCON = 0;

	TRISBbits.TRISB6 = 0;
	LATBbits.LATB6 = 0;	
	INTCONbits.MVEC = 1;
	
	Init_pwm(); // pwm output used to implement DAC

    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    config_SPI(); // Configure hardware SPI module
    
    TRISBbits.TRISB12=0; //initialise the buzzer pin.
 
	SET_CS; // Disable 25Q32 SPI flash memory

	ANSELB &= ~(1 << 5); // Set RB5 as a digital I/O for freq measurement
	TRISB |= (1 << 5);   // configure pin RB5 as input for frequency measurement
	CNPUB |= (1 << 5);   // Enable pull-up resistor for RB5 for freq measurement


	Buzz_OFF;

}

void Run_Metal_Detector(float Base_freq)
{
   
    float curr_freq;
    float diff;
	float Buzz_freq;
	
	curr_freq = Calc_Freq();
	diff = curr_freq - Base_freq;
	
	if(Freq_stable()){
		
		if(diff>=25 && diff<=100){
			play(Small_NFerr);
		}
		else if(diff>=100){
			play(Large_NFerr);
		}
		else if(diff<=-25 && diff>=-100){
			play(Small_Ferr);
		}
		else if(diff<=-100){
			play(Large_Ferr);
		}
		
	}else{
		
		Buzz_freq = 1 + abs(diff)/Base_freq; 	
		play_buzzer(Buzz_freq);
		
	}	
	
}

int main(void)
{
    
    float Base_freq;
    float curr_freq;
    float diff;
    float r;
    

	Initialize_SYS();

	Base_freq = Calc_Freq();	

	
	while(1){
		Run_Metal_Detector(Base_freq);
	}
	
    return 1;	
}