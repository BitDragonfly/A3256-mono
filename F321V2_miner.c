//-----------------------------------------------------------------------------
// F32x_UART0_Interrupts.c
//-----------------------------------------------------------------------------
// Copyright 2006 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This program demonstrates how to configure the C8051F320 to write to and read
// from the UART interface. The program reads a word using the UART interrupts
// and outputs that word to the screen, with all characters in uppercase
//
//
// How To Test:
//
// 1) Ensure that jumpers are placed on J3 of the C8051F320 target board
//    that connect the P0.4 pin to the TX signal, and P0.5 to the RX signal.
// 2) Ensure that the serial cable is connected to the RS232 connector
//    on the target board.
// 3) Specify the target baudrate in the constant <BAUDRATE>.
// 4) Open Hyperterminal, or a similar program, and connect to the target
//    board's serial port.
// 6) Type up to 64 characters into the Terminal and press Enter.  The MCU 
//    will then print back the characters that were typed
//
//
// Target:         C8051F32x
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
// Release 1.0
//    -Initial Revision (SM)
//    -4 JUN 2007
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <c8051f320.h>                 // SFR declarations
#include <stdio.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
sfr16 TMR2RL   = 0xca;                 // Timer2 reload value 
sfr16 TMR2     = 0xcc;                 // Timer2 counter
sfr16 ADC0     = 0xbd;                 // ADC0 result

sbit SW1 = P2^0;
sbit SW2 = P2^1;
sbit LED1 = P2^2;
sbit LED2 = P2^3;
sbit probe = P2^0;


sbit MCUTXD = P0^4;
sbit MCURXD = P0^5;

sbit NounceLED = P1^2;//O
sbit hashCLK   = P1^0;//O,12MHz,CEX0
sbit hashRSTn  = P1^3;//O
sbit ConfigP   = P1^4;//O
sbit ConfigN   = P1^5;//O
sbit ReportP   = P1^6;//I P0.2,MOSI
sbit ReportN   = P1^7;//I
sbit ReportCLK = P0^0;//I





#define SW1PRESSED (SW1==0) 
#define BAUDRATE      115200           // Baud rate of UART in bps
#define SYSTEMCLOCK 24000000           // SYSCLK frequency in Hz

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

#define UART_BUFFERSIZE 64
unsigned char xdata UART_RxBuffer[UART_BUFFERSIZE];
unsigned char UART_RxCounter = 0;
unsigned char UART_RxTimer = 0;
unsigned char TX_Ready =1;
static unsigned char Byte;

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void SYSCLK_Init (void);
void PCA0_Init(void);
void CLKMUL_Init (void);
void SPI0_Init(void);
void UART0_Init (void);
void ADC0_Init (void);
void Timer2_Init(void);
void PORT_Init (void);
void Timer2_Init (void);

void delayus(unsigned char us)
{
	unsigned char i,j;
	for (j=0;j<us;j++)
	{
		for (i=0;i<10;i++)
		{
			;
		}
	}
}
void delayms(unsigned int ms)
{
	unsigned int i,j;
	for (j=0;j<ms;j++)
	{
		//for (i=0;i<1280;i++)
		for (i=0;i<2380;i++)//24MHz
		{
			;
		}
	}
}

///-----------------------------------------------------------------------------
// Routine
//-----------------------------------------------------------------------------


unsigned char *UART_TxBufferPtr;
unsigned char  UART_TxBuffer_Size=0;
unsigned char  UART_TxCounter=0;

static unsigned long accumulator = 0;     // accumulator for averaging
#define AVERAGERSAMPLE 16
static unsigned int measurements = AVERAGERSAMPLE;  // measurement counter
unsigned int ADCmV;

union {unsigned char c[4]; unsigned int i[2]; unsigned long l;} nounce;
unsigned char nounceindex=3;
void SendPBUF(unsigned char *txbuf,unsigned char txsize)
{
	  UART_TxBufferPtr = txbuf;
	  UART_TxBuffer_Size = txsize;
	  TX_Ready = 0;                                     // Set the flag to zero
      TI0 = 1;
	  while (TX_Ready == 0);delayus(40);
}
/*
void SendSBUF(unsigned char buf)
{
	UART_TxBufferPtr = (unsigned char *)&buf;
	UART_TxBuffer_Size = 1;
	TX_Ready = 0;                                     // Set the flag to zero
    TI0 = 1;
	while (TX_Ready == 0);
	delayus(40);
}
*/
#define VBTVCC 0x0F
#define VBTV5R 0x0D
#define VBTV18 0x0C	 //P2.4
#define VBTVOM 0x09

#define ADCBP   0x0C	 //P2.4
#define ADCOSC  0x0E	 //P2.6

bit ADCReady;
/*
unsigned int GetADC(unsigned char ch)
{
//P2.3=0B,P2.4=0C,P2.5=0D,P2.7=0F
	AMX0P = ch;
	ADCmV = 0;
	accumulator = 0;
	measurements = AVERAGERSAMPLE;
	ADCReady = 0;
	while(!ADCReady);
	return ADCmV;
}
*/

void DetectAsics(void);
void DoRxTask(unsigned char xdata *rxbuf);
extern unsigned long xdata ClockCfg[2];
void CalcNounce(void)
{
	unsigned char i,j,byte;
	for (j=0;j<4;j++)
	{
	byte = 0;
	for (i=0;i<8;i++)
	{
		byte = byte << 1;
		if (nounce.c[j]&0x01) byte = byte | 0x01;
		nounce.c[j] = nounce.c[j] >> 1;
	}
	nounce.c[j] = byte;
	}
	nounce.l = nounce.l - 0x00000180;//AV2
}
unsigned int waitnouncetimer;
bit WaittingNounce(unsigned int timeout)
{
	while (	nounceindex !=0xFF)
	{
		delayms(1);
		waitnouncetimer++;
		if (waitnouncetimer>timeout | ((UART_RxCounter!=0)))
		{
			return 0;
		}
	}
	return 1;
}
unsigned int ledtimer;
bit gotnounce;
void main (void)
{

   unsigned int i;
   /*
	unsigned long mainN,mainR;
	mainN = (ClockCfg[0]&0x3FFFFFFF) >> 18;
	mainR = (ClockCfg[0]) >> 29;
	mainR = ((ClockCfg[1] & 0x00000007) << 3) | mainR;
	i = (mainN/mainR);
	i = 12/2 * i;//336MHz
	*/	
  // unsigned char c;
   PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer
                                       // enable)
   PORT_Init();                        // Initialize Port I/O
   SYSCLK_Init ();                     // Initialize Oscillator
   CLKMUL_Init ();
   UART0_Init();
   //ADC0_Init();
   //Timer2_Init();
   SPI0_Init();   
   PCA0_Init();
   EA = 1;

hashRSTn= 0;
ConfigP = 1;
ConfigN = 1;
   delayms(10);
hashRSTn = 1;
i=0x41;
   //SendSBUF(i);
   SendPBUF("USB Miner", sizeof("USB Miner"));
   delayms(100);
   
   UART_RxCounter = 0;
   UART_RxTimer = 0;

	NounceLED = 0;gotnounce=0;  	
	EA = 1;

	DetectAsics();
	if (WaittingNounce(100))
	{
		gotnounce = 1;
		CalcNounce();
		SendPBUF((unsigned char *)&nounce.l,4);
	}
	SPIEN = 0;IE = IE & 0xBF;//disable spi interrupt
	delayms(200);
	NounceLED=1;
	ledtimer = 0;

   while(1)
   {
	if (UART_RxCounter!=0)
	{
		delayms(1);
		UART_RxTimer ++ ;
		if (UART_RxCounter>63)
		{
			NounceLED = 0;gotnounce = 0;
			UART_RxCounter = 0;
			UART_RxTimer = 0;
			// got midstate & data
			DoRxTask(&UART_RxBuffer);
			if (WaittingNounce(60000))
			{	gotnounce = 1;ledtimer=0;
				CalcNounce();
				SendPBUF((unsigned char *)&nounce.l,4);
			}
			SPIEN = 0;IE = IE & 0xBF;//disable spi interrupt
			delayms(10);
		}
		else
		{
			if (UART_RxTimer==20)//RxTimerOut
			{
				UART_RxCounter = 0;
				UART_RxTimer = 0;
			}
		}
	}
	else
	{//idle
		ledtimer = ledtimer + 1;
		delayms(1);
		if (gotnounce)
		{
			if (ledtimer==250)
			{
				NounceLED = 1;
			}
			else if (ledtimer==500)
			{
				NounceLED = 0;
			}
			else if (ledtimer==750)
			{
				NounceLED = 1;
			}
			else if (ledtimer==1000)
			{
				NounceLED = 0;
				ledtimer = 0;
				gotnounce =0;
			}
		}
		else
		{
			if (ledtimer==1000)
			{
				ledtimer = 0;
				NounceLED = 0;
			}
			else if (ledtimer==200)
			{
				NounceLED = 1;
			}
		}

	}

/*
	if (SW2==0)
	{
	while (SW2==0);
	hashRSTn= 0;
	delayms(100);
	hashRSTn= 1;
	}
	//GetADC(0);
	if (SW1==0)
	{EA=0;
	while (SW1==0);
	SPIEN = 1;SPIF=0;probe = 0;
	DetectAsics();
	CalcNounce();
	SPIEN = 0;
	EA=1;
	SendPBUF((unsigned char *)&nounce.l,4);
	delayms(1000);
	}
*/
	}
	


}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure the Crossbar and GPIO ports.
// P0.4 - UART TX (push-pull)
// P0.5 - UART RX 


//-----------------------------------------------------------------------------

void PORT_Init (void)
{
   XBR0    |= 0x01;                    // Enable UART0
   XBR1    |= 0x40;                    // Enable crossbar and weak pull-ups
   P0MDOUT = 0x10;                    // Set TX pin to push-pull
   P1MDOUT = 0x3D;                    // enable push-pull output
   P2MDOUT = 0x01;

}

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This routine initializes the system clock to use the internal oscillator
// at its maximum frequency.
// Also enables the Missing Clock Detector.
//-----------------------------------------------------------------------------
//#define EXTOSC
void SYSCLK_Init (void)
{
   OSCICN = 0x83;                     // Configure internal oscillator for
                                       // its maximum frequency
   CLKSEL  = 0x10;
   //
   #ifdef EXTOSC
   OSCXCN = (0x60 | 0x07);//26MHz
   CLKSEL = 0x01;
   OSCICN = 0x00;
   #endif
   //XBR0   |= 0x08;                     // Route /SYSCLK to first available pin
   XBR1   |= 0x01;                    // Route CEX0 to P1.0
   //
   RSTSRC  = 0x04;                     // Enable missing clock detector
   P0SKIP  = 0x8C;//OSC,SYSCLK@P1.0,only SYSCLK, then to other IO
   //XBR0   |= 0xF0;//OSC,SYSCLK@P1.4
}
void PCA0_Init (void)
{
   // Configure PCA time base; overflow interrupt disabled
   PCA0CN = 0x00;                      // Stop counter; clear all flags
   PCA0MD = 0x08;                      // Use SYSCLK as time base,{CPS2,CPS1,CPS0,ECF}

   PCA0CPM0 = 0x46;                    // Module 0 = Frequency Output mode

   // Configure frequency for CEX0
   PCA0CPH0 = 1; //sysclk/2
   // Start PCA counter
   CR = 1;
}
//-----------------------------------------------------------------------------
// SPI0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configures SPI0 to use 4-wire Slave mode. The SPI timing is
// configured for Mode 0,0 (data centered on first edge of clock phase and
// SCK line low in idle state).
//
//-----------------------------------------------------------------------------

void SPI0_Init()
{
   SPI0CKR = 0x00;
   SPI0CFG = 0x20;                     // Operate in Slave mode
                                       // CKPHA = '1', CKPOL = '0'
   SPI0CN = 0x00;                      // 3-wire Slave mode, SPI disabled
   XBR0 |= 0x02;					// Enable the SPI on the XBAR
   //ESPI0 = 1; //IE.6                // Enable SPI interrupts
   //IE = IE | 0x40;
}

//-----------------------------------------------------------------------------
// CLKMUL_Init ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This routine initializes the Clock Multiplier using the Internal Oscillator
// as the source.
//
//-----------------------------------------------------------------------------

void CLKMUL_Init (void)
{
   char i;

   CLKMUL = 0x00;                      // Reset the Multiplier by writing 0x00
                                       // to register CLKMUL.

   CLKMUL |= 0x00;                     // Select the Multiplier input source
                                       // via the MULSEL bits.
                                       // '00'b = Internal Oscillator

   CLKMUL |= 0x80;                     // Enable the Multiplier with the MULEN
                                       // bit (CLKMUL | = 0x80).

   for (i = 35; i > 0; i--);           // Delay for >5 µs.
                                       // At 12 MHz SYSCLK ~ 80 ns, so 5 us is
                                       // 61 SYSCLK counts. DJNZ = 2 clocks.

   CLKMUL |= 0xC0;                     // Initialize the Multiplier with the
                                       // MULINIT bit (CLKMUL | = 0xC0).

   while ((CLKMUL & 0x20) != 0x20);    // Poll for MULRDY => ‘1’.
   CLKSEL = 0x12; // Switch SYSCLK to the CLKMUL output
}

//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure the UART0 using Timer1, for <BAUDRATE> and 8-N-1.
//-----------------------------------------------------------------------------


void UART0_Init (void)
{
   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //        clear RI0 and TI0 bits
   if (SYSTEMCLOCK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2);
	  TH1=TH1-1;//SLOW down
      CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
      CKCON |=  0x08;
   } else if (SYSTEMCLOCK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSTEMCLOCK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   }

   TL1 = TH1;                          // init Timer1
   TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TR1 = 1;                            // START Timer1
   TX_Ready = 1;                       // Flag showing that UART can transmit
   IP |= 0x10;                         // Make UART high priority
   ES0 = 1;                            // Enable UART0 interrupts

   //TB80 = 1;
   //S0MODE = 1; //9-Bit
   //add 10nF to pin for glitch
}
//-----------------------------------------------------------------------------
// ADC0_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configures ADC0 to make single-ended analog measurements on pin P2.4
//  
//-----------------------------------------------------------------------------

void ADC0_Init (void)
{
   ADC0CN = 0x02;                      // ADC0 disabled, normal tracking, 
                                       // conversion triggered on TMR2 overflow

   REF0CN = 0x03;                      // Enable on-chip VREF and buffer

   AMX0P = 0x0C;                       // ADC0 positive input = P2.4
   AMX0N = 0x1F;                       // ADC0 negative input = GND
                                       // i.e., single ended mode

   ADC0CF = ((SYSTEMCLOCK/3000000)-1)<<3;   // set SAR clock to 3MHz

   ADC0CF |= 0x00;                     // right-justify results 

   //EIE1 |= 0x08;                       // enable ADC0 conversion complete int.

   //AD0EN = 1;                          // enable ADC0
}
//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
// Return Value:  None
// Parameters:    None
//
// Configure Timer2 to 16-bit auto-reload and generate an interrupt at 100uS 
// intervals.  Timer 2 overflow automatically triggers ADC0 conversion.
// 
//-----------------------------------------------------------------------------

void Timer2_Init (void)
{
   TMR2CN  = 0x00;                     // Stop Timer2; Clear TF2;
                                       // use SYSCLK as timebase, 16-bit 
                                       // auto-reload
   CKCON  |= 0x10;                     // select SYSCLK for timer 2 source
   TMR2RL  = 65535 - (SYSTEMCLOCK / 10000); // init reload value for 10uS
   TMR2    = 0xffff;                   // set to reload immediately
   TR2     = 1;                        // start Timer2
}
//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// UART0_Interrupt
//-----------------------------------------------------------------------------
//
// This routine is invoked whenever a character is entered or displayed on the
// Hyperterminal.
//
//-----------------------------------------------------------------------------

void UART0_Interrupt (void) interrupt 4
{
   if (RI0 == 1)
   {
      RI0 = 0;                           // Clear interrupt flag

      Byte = SBUF0;                      // Read a character from UART

      if (UART_RxCounter < UART_BUFFERSIZE)
      {
         UART_RxBuffer[UART_RxCounter] = Byte; // Store in array
         UART_RxCounter++;             // Update counter
      }
   }

   if (TI0 == 1)                   // Check if transmit flag is set
   {
      TI0 = 0;                           // Clear interrupt flag

      if (UART_TxBuffer_Size != 0)         // If buffer not empty
      {
         Byte = UART_TxBufferPtr[UART_TxCounter];

		 //delayus(1);
         SBUF0 = Byte;                   // Transmit to Hyperterminal

         UART_TxCounter++;            // Update counter

         UART_TxBuffer_Size--;             // Decrease array size
      }
      if (UART_TxBuffer_Size == 0)
      {
         UART_TxBuffer_Size = 0;          // Set the array size to 0
		 UART_TxCounter = 0; //reset pointer counter
         TX_Ready = 1;                    // Indicate transmission complete
      }
   }
}
void SPI_ISR (void) interrupt 6
{
	if (SPIF)
	{
		nounce.c[nounceindex] = SPI0DAT;
		nounceindex--;
	}
    SPI0CN = SPI0CN&0x01;
	probe = !probe;
}
//-----------------------------------------------------------------------------
// ADC0_ISR
//-----------------------------------------------------------------------------
// 
// This ISR averages 2048 samples then prints the result to the terminal.  The 
// ISR is called after each ADC conversion which is triggered by Timer2.
//
//-----------------------------------------------------------------------------
void ADC0_ISR (void) interrupt 10
{

   unsigned long result=0;
   unsigned long mV;                         // measured voltage in mV

   AD0INT = 0;                               // clear ADC0 conv. complete flag

   accumulator += ADC0;
   measurements--;

   if(measurements == 0)
   {  
      measurements = AVERAGERSAMPLE;
      result = accumulator / AVERAGERSAMPLE;
      accumulator=0;

      // The 10-bit ADC value is averaged across 2048 measurements.  
      // The measured voltage applied to P1.4 is then:
      //
      //                           Vref (mV)
      //   measurement (mV) =   --------------- * result (bits) 
      //                       (2^10)-1 (bits)

      mV =  result * 2440 / 1023;   
	  ADCmV = mV;
	  ADCReady = 1;
   }
}



//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
#define BYTE unsigned char
#define WORD unsigned int
#define DWORD unsigned long
typedef struct _id {
    BYTE version;
    DWORD serial;
    char product[8];
} IDENTITY;
///////klondike.h/////////
#define MASTER_ADDRESS      0x00
#define MAX_WORK_COUNT      4   // must be binary multiple
#define MAX_RESULT_COUNT    2   // must be binary multiple
#define WORKMASK            MAX_WORK_COUNT-1


// for controlling how bits are adjusted per bank
#define DATA_ZERO   0x48
#define DATA_ONE    0x90
#define DATA_SPLIT  0x50
// set values for ASIC PLL, we use R=32 N=Freq*2 in MHz
#define WORK_TICKS          16777
#define TICK_FACTOR         24000
#define CLOCK_R_VALUE       32  //R
#define DEFAULT_HASHCLOCK   1638	//N 24MHz,512=192MHz, 819=(2.4M*128)/192*512, 896,1280,1408,1536
#define CLOCK_LOW_CFG       0x00000007
#define CLOCK_HIGH_CFG      0x00000000
#define PLL_CLOCK_R      1
#define PLL_CLOCK_F      110
#define PLL_CLOCK_OD     0

/////asic.c///////
void SendBit(bit bitdata)
{
	unsigned char PN;
	P1 = (P1 & 0xCF);//P,N==0;
	delayus(1);
	if(bitdata)
	{
		PN = 0x10;
	}
	else
	{
		PN = 0x20;
	}
	P1 = P1 | PN;
}
void Send32(DWORD xdata *wdatapt, unsigned char len)
{
	union {unsigned char c[4]; unsigned int i[2]; unsigned long l;} wdata;
	unsigned char i;
	do
	{
	wdata.l = *wdatapt;
	for (i=0;i<32;i++)
	{
		SendBit(wdata.c[3]&0x01);//lsbit first,lsbyte first
		wdata.l = wdata.l >> 1;
	}
	wdatapt++;
	len--;
	}while (len>0);
}
//R=1;
//F= 100
//D= 1
//CLK = 12MHz * F/R / D
BYTE  xdata BankSize;
DWORD xdata ClockCfg[2]= { ((DWORD)PLL_CLOCK_OD << 28) |((DWORD)PLL_CLOCK_F << 21) |((DWORD)PLL_CLOCK_R << 16) | CLOCK_LOW_CFG, CLOCK_HIGH_CFG };
DWORD xdata PrecalcHashes[6];
DWORD xdata NonceRanges[10];
typedef struct _worktask {
    BYTE WorkID;
    DWORD MidState[8];
    DWORD Merkle[3];
} WORKTASK;
void SendAsicData(WORKTASK *work)
{
	P1 = (P1 & 0xCF);//START ,ConfigN = 0;ConfigP = 0;
    Send32(&ClockCfg,2);
    Send32(&work->Merkle,3);
    Send32(&PrecalcHashes[1],5);
    Send32(&work->MidState,8);
    Send32(&PrecalcHashes,1);
    Send32(&NonceRanges,BankSize);
	delayus(1);

	nounceindex = 3;IE = IE | 0x40;//ESPI0=1;//IE.6
	waitnouncetimer=0;SPIEN = 1;SPIF=0;probe = 0;

	P1 = P1 | 0x30;//END,ConfigN = 1;ConfigP = 1; 
	//begin to check data;
	/*
			while(SPIF==0);SPIF=0;
			nounce.c[3] = SPI0DAT;
			probe = !probe;
			while(SPIF==0);SPIF=0;
			nounce.c[2] = SPI0DAT;
			probe = !probe;
			while(SPIF==0);SPIF=0;
			nounce.c[1] = SPI0DAT;
			probe = !probe;
			while(SPIF==0);SPIF=0;
			nounce.c[0] = SPI0DAT;
			probe = !probe;
	*/
	//
}
#define r(x) ((x-n)&7)
DWORD rotate(DWORD x, BYTE y)
{
    return ((x<<y) | (x>>(32-y)));
}

void AsicPreCalc(WORKTASK *work)
{ 
	code DWORD K[3] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf };
    DWORD xdata x, y, z;
    DWORD xdata m[8];
    BYTE xdata n;
    
    for(n = 0; n < 8; n++)
        m[n] = work->MidState[n];

    for(n = 0; n < 3; n++) {
				   
        x = m[5-n] ^ m[6-n];
        x = x & m[4-n];		
        x = m[6-n] ^ x;
        x += K[n]; 
        x += work->Merkle[n];
        x += m[7-n];
        y = rotate(m[4-n], 26);
        z = rotate(m[4-n], 21);
        z = y^z;
        y = rotate(m[4-n], 7);
        z = y^z;		 
        m[7-n] = z+x;
        m[3-n] = m[3-n] + m[7-n];
        x = rotate(m[r(0)], 30);
        y = rotate(m[r(0)], 19);
        y = y^x;
        x = rotate(m[r(0)], 10);
        y = x^y;
        x = m[r(0)] | m[r(1)];
        x = m[r(2)] & x;
        z = m[r(0)] & m[r(1)];
        x = x | z;
        m[7-n] += y + x;

        PrecalcHashes[2-n] = m[7-n];
        PrecalcHashes[5-n] = m[3-n];
    } 
}
/////asic.c///////
#define GOOD_MIDSTATE {0xe48f544a,0x9a3afa71,0x45147113,0x4df6c356,0x82b40025,0x4bfe0860,0xc99876bf,0x4679ba4e}
#define GOOD_DATA {0x2fa722ce,0x1426674f,0x87320b1a}
#define GOOD_NONCE 0x000187a2
WORKTASK xdata TestWork = { 0xFF, GOOD_MIDSTATE, GOOD_DATA };
WORKTASK xdata WorkQue[MAX_WORK_COUNT];

void DetectAsics(void)
{
	BYTE x;

	BankSize = 1;
    NonceRanges[0] = 0;
    //for(x = 1; x < BankSize; x++)
    //    NonceRanges[x] = NonceRanges[x-1] + 0x19999999;


    AsicPreCalc(&TestWork);
    //WorkQue[MAX_WORK_COUNT-1] = TestWork;
    //SendAsicData(&WorkQue[MAX_WORK_COUNT-1]);
	SendAsicData(&TestWork);
}
void DoRxTask(unsigned char xdata *rxbuf)
{
	unsigned char i;
	unsigned long tempdata;
	TestWork.WorkID = 0x01;
	for (i=0;i<32;i++)
	{
		*((unsigned char *)&TestWork.MidState+i) = *(rxbuf+i);
	}
	for (i=0;i<12;i++)
	{
		*((unsigned char *)&TestWork.Merkle+i) = *(rxbuf+52+i);
	}
	for (i=0;i<4;i++)
	{
		tempdata = TestWork.MidState[i];
		TestWork.MidState[i]=TestWork.MidState[7-i];
		TestWork.MidState[7-i] = tempdata;
	}
	tempdata = TestWork.Merkle[0];
	TestWork.Merkle[0] = TestWork.Merkle[2];
	TestWork.Merkle[2] = tempdata;

	BankSize = 1;
    NonceRanges[0] = 0;
	AsicPreCalc(&TestWork);
	SendAsicData(&TestWork);
	//TestWork.MidState = rxbuf;
	//TestWork.Merkle =  rxbuf;
}
