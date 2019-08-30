//-----------------------------------------------------------------------------
// Program Name: C8051F MCU BOARD (S/N:C8051F001, 2012.09.27, Rev.0)
// Copyright (C) 2015 AD Semiconductor Co.,LTD.
// Author: Park, Byoung Bae (yni2yni@hanmail.net)
// Date :12 Januray 2015
// MCU F/W Program Version: C8051F327 MCU BOARD Test Program
// File Name: N/A
// Target: Silabs C8051F327 5x5 28Pin / OSC= 24.0MHz (Int. OSC),
// 		   Intel MCS-51 Compatible MCU
// Tool chain: KEIL C51 C Compiler  Version 8.12 ~ 9.51
// IDE: Silicon Labs Simplicity Studio V2 Higher
// H/W board: C8051F MCU B/D (New Mini board)
// HOST Program Version: N/A
// HOST S/W Compiler IDE Tool: X (Option:MS Visual Studio 2008)
// HEX File:
// Note: Text Code UTF-8
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
//#include <compiler_defs.h>
//#include <SI_C8051F326_Register_Enums.h>                // SFR declarations
//-----------------------------------------------------------------------------
#include <compiler_defs.h>
#include"c8051f326.h"                 // SFR declarations Sliabs C8051F326_7
#include <stdio.h>
//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
#define SYSCLK         2400000       // SYSCLK frequency in Hz(24MHz)
#define BAUDRATE1      115200     	 // Baud rate of UART in bps
//IC I2C Address ------------------------------------------------------------
#define AGS04_Addr		0xD4  	// I2C Chip Address,

#define Timer1_High_Value	00
#define Timer1_Low_Value	20
//-------------------------------------------------------------------------
#define DISABLE	1
#define ENABLE	0
//--------------------------------------------------------------------------
//MCU  GPIO Port Map. Data -------------------------------------------------
sbit I2C_CLK	= P0^3;			//I2C Clock
sbit I2C_DATA	= P0^1;			//I2C Data
sbit I2C_EN		= P0^6;			//I2C ENABLE
sbit RESET		= P0^7;			//IC RESER

// Port0 bit 1 -------------------------------------------------------------
//sbit INT0		= P0^0;		//Touch INT Signal
sbit SW1		= P0^0;     //SW1 Input & External Photocoupler input
sbit SW2		= P0^2;     //SW1 Input
// Tact S/W Port ------------------------------------------------------------
//sbit SW2		= P0^2;     //SW2 Input Port
// LED
sbit LED1		= P2^0;		//GPIO Out port2
sbit LED2		= P2^1;		//GPIO Out port2
sbit LED3		= P2^2;		//GPIO Out port2
sbit LED4		= P2^3;		//GPIO Out port2
//sbit PC_Out		=P0^7;		// Photocoupler Output Control
//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F326
//-----------------------------------------------------------------------------
sfr16 TMR1    = 0x8A;                 // Timer1 counter
sfr16 SBRL1   = 0x93;
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void);
void PORT_Init (void);
void Voltage_Regulator_Init();
void Reset_Sources_Init();

// 1st IIC Function
void i2c_start(void); // I2C Start Condition
void i2c_stop(void);  // I2C Stop Condition
bit  bit_wr(unsigned char temp); //Bit Write
void I2C_Read(unsigned char Chip_Addr, unsigned char Reg_Addr,unsigned char* Data, unsigned char Data_Size);
void I2C_Write(unsigned char Chip_Addr, unsigned char Reg_Addr,unsigned char* Data, unsigned char Data_Size);
void Delay1(unsigned char del_t);

// Delay Time
void Delay(unsigned int);
//void _nop_ (void);

//UART0
void UART0_Init (void);
void UART0_Interrupt (void);
void BD_Init(void);

// Touch IC Control
void Initialize_AGS04(void);  //  TS20x IC, ADD=GND
void READ_AGS04(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char serial_buffer;	//UART Tx Data
unsigned char reg_w_data[15];
unsigned char reg_r_data[15];
unsigned char Key_data[10];
unsigned char serial_buffer;
unsigned char NbData;
unsigned char i=0;
unsigned char ack;
unsigned int Timer_Count;

//-------------- UART ------------------------
#define UART_BUFFERSIZE 3
unsigned char UART_Buffer[UART_BUFFERSIZE]={0x00,0x00,0x00};
unsigned char UART_Buffer_Size = 0;
unsigned char UART_Input_First = 0;
unsigned char UART_Output_First = 0;
unsigned char TX_Ready =1;
unsigned char UART_Byte_Rx;
unsigned char UART_Byte_Tx;
unsigned char UART_rdata[3]; 	//UART Receive Data Temp.

void serial_send(void)
{
	//SBUF0=serial_buffer;
	//UART_Byte_Tx=serial_buffer;
	SBUF0=UART_Byte_Tx;
	//while(TI0 == 0);
	//TI0 = 0;
	Delay(100);
}

void Delay(unsigned int j)
{
	unsigned int i,k;
	for(i=0; i<= j; i++)		// 24Mhz= 41.6 nsec
	{k=1;k=0;k=1;k=0;k=0;}
	// 10 -> 2.083usec, 100 -> 20.8usec, 1000 = 208usec, 5000 = 0.2msec
}

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------
void main (void)
{
 	OSCILLATOR_Init ();      	// Initialize system clock
	Voltage_Regulator_Init();	// Voltage Regulator
   	Reset_Sources_Init(); 		// Reset Source Setting
   	PORT_Init ();             	// Initialize crossbar and GPIO
   	UART0_Init();				// Initialize UART0
	BD_Init(); 					// Initialize Main Board  & Circuit

   	Delay(50000);Delay(50000);	Delay(50000);Delay(50000);
	Delay(50000);Delay(50000);	Delay(50000);Delay(50000);
	Delay(5000);
   	//printf(" %c" ,0x0C); // Hex Code : FF (New Page)
	//---------------------------------------------------------------------
	// Delay 500[msec], Power-on -> Delay(about 100msec -> Touch IC Control
	//---------------------------------------------------------------------
	Delay(50000);  	// 20msec
    Delay(50000);	// 20mese

    Initialize_AGS04(); // Initialize Touch Sensor IC
	Delay(50000);

   	while (1)
   	{
		//READ_ANSG08(); // Read Touch Sesnor Output
		READ_AGS04();
		Delay(10);
  	}

} //end main

// Enable VDD Monitor as a Reset Source, Missing Clock Detector
void Reset_Sources_Init()
{
    int i = 0;
    VDM0CN    = 0x80;
    for (i = 0; i < 350; i++);  // Wait 100us for initialization
    RSTSRC    = 0x06;
}
//-----------------------------------------------------------------------------
// Regulator Enable / Disable broutines
//-----------------------------------------------------------------------------
void Voltage_Regulator_Init()
{
	REG0CN    = 0x80; //Disable Voltage Regulator
}
//-----------------------------------------------------------------------------
// OSCILLATOR_Init
//-----------------------------------------------------------------------------
// This routine initializes the system clock to use the internal 12MHz /1
// Oscillator as its clock source.
// SYSCLK =>  Int. 12MHz /1 x  (Int. Clock X 4 /2) = 24MHz (Max)
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
    int i = 0;
    OSCLCN    = 0x80;
	CLKMUL    = 0x80;
    for (i = 0; i < 20; i++);    // Wait 5us for initialization
    CLKMUL    |= 0xC0;
    while ((CLKMUL & 0x20) == 0);
    CLKSEL    = 0x32;		// USB Clock Disable , USB Clock On: "CLKSEL = 0x02"
    OSCICN    = 0x83;
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// This function configures the GPIO ports.
// ------------------Pin Map. ---------------------------------------------------
// P0.0: SW1, Input , Photocoupler  Input
// P0.1: SDA1, in/out ,O.D , IIC Data Line
// P0.2: SW2, Input,
// P0.3: SCL1, Output, O.D , IIC Clock Line
// P0.4: Tx, out, P.P , UART Tx Port (Uart Default)
// P0.5: Rx, in, O.D , UART Rx Port (Uart Default)
// P0.6: Output, P.P
// P0.7: Output, Photocoupler Output
//
// P2.0: Output, P.P
// P2.1: Output, P.P
// P2.2: Output, P.P
// P2.3: Output, P.P
// P2.4: Output, P.P
// P2.5: Output, P.P
//
// P3.0: C2D, in/out, O.D // C2 Debug Port
//-----------------------------------------------------------------------------
void PORT_Init (void)
{
   	//P2MDOUT |= 0x04;                    // Enable LED as a push-pull output
	//P0MDOUT   = 0xF0;
	P0MDOUT   = 0xDD;
    P2MDOUT   = 0x3F; 			//P2 port all Push-pull
	//P2MDOUT   = 0x3B; 			//P2.2, P2.3 open-drain
 	// P0.4   digital   push-pull    UART TX
 	// P0.5   digital   open-drain   UART RX (With Weak Pullups)
 	//P0MDOUT   |= 0x10;
 	//GPIOCN	= 0xC0;		// UART (
 	GPIOCN = 0x40;                      // Enable input path
}
//-----------------------------------------------------------------------------
// Configure the UART0 using Baudrate generator, for <BAUDRATE1> and 8-N-1.
// 115200 - 8 - N - 1
//-----------------------------------------------------------------------------
void UART0_Init (void)
{

	SCON0	= 0x10;
	SBRLL0	= 0x98;			//115200bps, 8-N-1
	SBRLH0 	= 0xFF;
	SBCON0	= 0x43;

   	TI0 = 1;             	// Indicate TX0 ready
   	IE        = 0x90;		// Enable UART0 interrupts
}

void UART0_Interrupt (void) interrupt 4
{
   if (RI0 == 1)	 //if Receive Data is Full (SBUF0)
   {
      if( UART_Buffer_Size == 0){     // If new word is entered
          UART_Input_First = 0; }

      RI0 = 0;                       // Clear interrupt flag
      UART_Byte_Rx = SBUF0;   		// Read a character from UART

      if (UART_Buffer_Size < UART_BUFFERSIZE)
      {
         UART_Buffer[UART_Input_First] = UART_Byte_Rx;
         // Store in array , Receive Data
         UART_Buffer_Size++;             // Update array's size
         UART_Input_First++;             // Update counter
      }
	else
		{UART_Buffer_Size= 0;}
   }

    else if (TI0 == 1)                   // Check if transmit flag is set
   {
	//SBUF0 = UART_Byte_Tx;
	while(TI0==0){}
		  TI0 = 0;                       // Clear interrupt flag
  }

}
//-----------------------------------------------------------------------------
// IIC Function Start
//-----------------------------------------------------------------------------
void Delay1(unsigned char del_t)
{
  while(del_t--);		// 24Mhz= 41.66 nsec    //10
  // 100: 416nsec, 1000: 4.1usec
}

void i2c_start(void) //Original
{
	I2C_DATA = 1;
	I2C_CLK	 = 1;
	I2C_DATA = 0;I2C_DATA = 0;
	I2C_CLK	 = 0;I2C_CLK = 0;
}

void i2c_stop(void) //Original
{
	I2C_CLK  = 0;
	I2C_DATA = 0;
	I2C_CLK  = 1;I2C_CLK  = 1;
	I2C_DATA = 1;I2C_DATA = 1;
}

bit  bit_wr(unsigned char temp)
{
	bit ack;

	//P0MDOUT =0x0B;		//Change Data Direction for Output
	//P0MDOUT   = 0x98;	//Port Init. Value
	//P0MDOUT =( 0x98&0xFD);// Change Data Diretion For Output, Port0, bit 1 (P^1) = Clesr('0')

	// SDA High  11111 00000 11111 00000 11111   (1,0,1,0,1)
	// SDA Low   00000 11111 00000 11111 00000   (0,1,0,1,0)
	// SCL          11110 11110 11110 11110 11110   (C,C,C,C,C)
	if(temp&0x80){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0;Delay1(50); I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;   //CLK =  High ,duration CLK=1 x 4
	if(temp&0x40){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0; Delay1(50);I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;
	if(temp&0x20){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0; Delay1(50);I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;
	if(temp&0x10){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0; Delay1(50);I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;
	if(temp&0x08){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0; Delay1(50);I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;
	if(temp&0x04){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0; Delay1(50);I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;
	if(temp&0x02){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0; Delay1(50);I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;
	if(temp&0x01){ I2C_DATA = 1;Delay1(50);I2C_DATA = 1;} else {I2C_DATA = 0; Delay(50);I2C_DATA = 0;}
	I2C_CLK = 1;  I2C_CLK = 1;I2C_CLK = 1; Delay1(50); I2C_CLK = 1; I2C_CLK = 0;

	//P0MDOUT =0x09;		      	//Change Data Direction for Input
	//P0MDOUT = ( 0x98|0x02); 		// Change Data Diretion For inputt, Port0, bit 1 (P^1) = Set('1')

	I2C_DATA = 1; Delay1(50);	//Delay1(3);
	I2C_CLK = 1; 	//Delay1(0);

	if(I2C_DATA==1){ ack = 0;Delay1(50);} //fail,if Data Line == High , Ack =0 , Data Line==Low, Ack=1
	else {ack = 1;Delay1(50);} //succeed

 	I2C_CLK = 0; Delay1(50);

	//P0MDOUT =0x0B; 			//Change Data Direction for Output
	I2C_DATA = 0;

	return ack;

}

void I2C_Write(unsigned char Chip_Addr, unsigned char Reg_Addr,unsigned char* Data, unsigned char Data_Size)
{
		unsigned char i =0;
		i2c_start();
		bit_wr(Chip_Addr);
		bit_wr(Reg_Addr);
		for(i=0;i<Data_Size;i++){
			bit_wr( *(Data+i) );
		}
		i2c_stop();
}

void I2C_Read(unsigned char Chip_Addr, unsigned char Reg_Addr,unsigned char* Data, unsigned char Data_Size)
{
	unsigned char i, i2c_read,k;

	i2c_start();
	bit_wr(Chip_Addr);
	bit_wr(Reg_Addr);
	i2c_stop();
	i2c_start();
	bit_wr(Chip_Addr+1);

	for(i=0;i<Data_Size;i++){
		//P0MDOUT =0x09;	//Data Port : Input
		//P0MDOUT |= 0x02; 	// Change Data Diretion For inputt, Port0, bit 1 (P^1) =>  Set('1')

		I2C_DATA = 1;
		i2c_read = 0; //Read data temp value

		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x80; else i2c_read &= 0x7F; I2C_CLK =0;Delay1(50);k=1; while(k--);
		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x40; else i2c_read &= 0xBF; I2C_CLK =0;Delay1(50);k=1; while(k--);
		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x20; else i2c_read &= 0xDF; I2C_CLK =0;Delay1(50);k=1; while(k--);
		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x10; else i2c_read &= 0xEF; I2C_CLK =0;Delay1(50);k=1; while(k--);
		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x08; else i2c_read &= 0xF7; I2C_CLK =0;Delay1(50);k=1; while(k--);
		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x04; else i2c_read &= 0xFB; I2C_CLK =0;Delay1(50);k=1; while(k--);
		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x02; else i2c_read &= 0xFD; I2C_CLK =0;Delay1(50);k=1; while(k--);
		I2C_CLK =1;Delay1(50); if(I2C_DATA==1) i2c_read |= 0x01; else i2c_read &= 0xFE; I2C_CLK =0;Delay1(50);k=1; while(k--);

		//P0MDOUT =0x0B;		//Data Port : Output
		//P0MDOUT &= 0xFD; 		//Change Data Diretion For Output, Port0, bit 1 (P^1) => Clesr('0')

		if(i==(Data_Size-1))
			{I2C_DATA = 1;Delay1(5);}
		else
			{ I2C_DATA = 0;Delay1(10);}
		I2C_CLK = 1; Delay1(10);
		k=1; while(k--);//delay
		I2C_CLK =0; Delay1(10);
		*(Data+i) = i2c_read;
	}
	i2c_stop();

}

//--------------------- IIC Function End ---------------------------

// Config Main Board I/O Port (C8051F327 MCU BOARD)
void BD_Init(void)
{
	LED1	 	= 0;		//LED On
	LED2		= 0;
	LED3 		= 0;
	LED4		= 0;
}

// ------------------------- initializes function ----------------------------

//-----------------------------------------------------------------------------
// Initialize_AGS04 IC
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// This routine initializes the AGS04 IC (Address 0xD4
//-----------------------------------------------------------------------------
void Initialize_AGS04(void)
{
	unsigned char rw_data[15];

	// S/W Reset enable (Reset, Bit 0 = set)
	rw_data[0] = 0x4D;
	I2C_Write(AGS04_Addr, 0x02,rw_data, 1);
	Delay1(10); //Delay time few msec

	// Channel Enable/Disable
	rw_data[0] = 0x0F; //"1"= Enable, "0"=Disable
	I2C_Write(AGS04_Addr, 0x01,rw_data, 1);
	Delay1(10);

	// Global Control Register 1
	rw_data[0] = 0x4D;
	I2C_Write(AGS04_Addr, 0x02,rw_data, 1);
	Delay1(10);

	// Interrupt Mode
	rw_data[0] = 0x20;
	I2C_Write(AGS04_Addr, 0x03,rw_data, 1);
	Delay1(10);
//-------------------- Sensitivity --------------------
	// Value X 0.1 + 0.05%, Ex)0x09 = 9.0 X 0.1 + 0.05 = 0.95%
	// Sensitivity Ch1
	rw_data[0] = 0x09; // (Max: 0x00, Min: 0xFF)
	I2C_Write(AGS04_Addr, 0x04,rw_data, 1);
	Delay1(10);
	// Sensitivity Ch2
	rw_data[0] = 0x09; // (Max: 0x00, Min: 0xFF)
	I2C_Write(AGS04_Addr, 0x05,rw_data, 1);
	Delay1(10);
	// Sensitivity Ch3
	rw_data[0] = 0x09; // (Max: 0x00, Min: 0xFF)
	I2C_Write(AGS04_Addr, 0x06,rw_data, 1);
	Delay1(10);
	// Sensitivity Ch4
	rw_data[0] = 0x09; // (Max: 0x00, Min: 0xFF)
	I2C_Write(AGS04_Addr, 0x07,rw_data, 1);
	Delay1(10);
//--------------------------------------------------------
	// Value X 0.2 + 0.15%, 0x00 = Disable
	// Sensitivity Limit Ch1
	rw_data[0] = 0x00; // (Disable: 0x00)
	I2C_Write(AGS04_Addr, 0x08,rw_data, 1);
	Delay1(10);
	// Sensitivity Limit Ch2
	rw_data[0] = 0x00; // (Disable: 0x00)
	I2C_Write(AGS04_Addr, 0x09,rw_data, 1);
	Delay1(10);
	// Sensitivity Limit Ch3
	rw_data[0] = 0x00; // (Disable: 0x00)
	I2C_Write(AGS04_Addr, 0x0A,rw_data, 1);
	Delay1(10);
	// Sensitivity Limit Ch4
	rw_data[0] = 0x00; // (Disable: 0x00)
	I2C_Write(AGS04_Addr, 0x0B,rw_data, 1);
	Delay1(10);
//--------------------------------------------------------
	// Calibration Speed control of sense channel
	rw_data[0] = 0x33; //Reset value: 0x23,Recommended 0x33(fast)
	I2C_Write(AGS04_Addr, 0x0C,rw_data, 1);
	Delay1(10);

	// Calibration Speed control of R.D.F channel
	rw_data[0] = 0x33; //Reset value: 0x23,Recommended 0x33(fast)
	I2C_Write(AGS04_Addr, 0x0D,rw_data, 1);
	Delay1(10);

	// Out Expiration
	rw_data[0] = 0x00; //0x00:Don't use output expiration
	I2C_Write(AGS04_Addr, 0x0E,rw_data, 1);
	Delay1(10);

	/*
		// Sensing Duty Control (0x10h~0x15h)
		// Please contact to us
		rw_data[0] = 0x44; //0x00
		rw_data[1] = 0x44; //0x00
		rw_data[2] = 0x44; //0x00
		rw_data[3] = 0x44; //0x00
		rw_data[4] = 0x44; //0x00
		rw_data[5] = 0x44; //0x00
		I2C_Write(AGS04_Addr, 0x10,rw_data, 6);
		Delay1(10);
	*/

	// Sensing period control register
	//Value X 3.2 + 11.2[mse]
	//rw_data[0] = 0x09; //0x09: 9 X 3.2 + 11.2 = 40[msec]
	rw_data[0] = 0x03; //0x03: 3 X 3.2 + 11.2 =20.8[msec]
	I2C_Write(AGS04_Addr, 0x16,rw_data, 1);
	Delay1(10);

	// Pattern Sleep control register
	//if use pattern sleep mode -> Please contact to us
	rw_data[0] = 0x00; //0x00: Pattern sleep Mode Disable
	I2C_Write(AGS04_Addr, 0x1A,rw_data, 1);
	Delay1(10);

	// Pattern Sleep inner control register
	//if use pattern sleep mode -> Please contact to us
	rw_data[0] = 0x00; //0x00
	I2C_Write(AGS04_Addr, 0x1B,rw_data, 1);
	Delay1(10);

	// Pattern Sleep expiration counter register
	//if use pattern sleep mode -> Please contact to us
	//rw_data[0] = 0x00; //0x00
	//I2C_Write(AGS04_Addr, 0x1C,rw_data, 1);
	//Delay1(10);

	// Pattern Sleep sequence number
	//if use pattern sleep mode -> Please contact to us
	//rw_data[0] = 0x00; //0x00
	//I2C_Write(AGS04_Addr, 0x1D,rw_data, 1);
	//Delay1(10);

	// Pattern Sleep channel pair1
	//if use pattern sleep mode -> Please contact to us
	rw_data[0] = 0x00; //0x00
	I2C_Write(AGS04_Addr, 0x1E,rw_data, 1);
	Delay1(10);

	// Pattern Sleep channel pair2
	//if use pattern sleep mode -> Please contact to us
	rw_data[0] = 0x00; //0x00
	I2C_Write(AGS04_Addr, 0x1F,rw_data, 1);
	Delay1(10);
/*
	// Pattern Sleep data A (0x20h~0x2Eh)
	//if use pattern sleep mode -> Please contact to us
	rw_data[0] = 0x00; //0x00
	rw_data[1] = 0x00; //0x00
	rw_data[2] = 0x00; //0x00
	rw_data[3] = 0x00; //0x00
	rw_data[4] = 0x00; //0x00
	rw_data[5] = 0x00; //0x00
	rw_data[6] = 0x00; //0x00
	rw_data[7] = 0x00; //0x00
	rw_data[8] = 0x00; //0x00
	rw_data[9] = 0x00; //0x00
	rw_data[10] = 0x00; //0x00
	rw_data[11] = 0x00; //0x00
	rw_data[12] = 0x00; //0x00
	rw_data[13] = 0x00; //0x00
	rw_data[14] = 0x00; //0x00
	rw_data[15] = 0x00; //0x00
	I2C_Write(AGS04_Addr, 0x20,rw_data, 15);
	Delay1(10);
*/
/*
	// Pattern Sleep data B (0x30h~0x3Eh)
	//if use pattern sleep mode ->Please  contact to us
	rw_data[0] = 0x00; //0x00
	rw_data[1] = 0x00; //0x00
	rw_data[2] = 0x00; //0x00
	rw_data[3] = 0x00; //0x00
	rw_data[4] = 0x00; //0x00
	rw_data[5] = 0x00; //0x00
	rw_data[6] = 0x00; //0x00
	rw_data[7] = 0x00; //0x00
	rw_data[8] = 0x00; //0x00
	rw_data[9] = 0x00; //0x00
	rw_data[10] = 0x00; //0x00
	rw_data[11] = 0x00; //0x00
	rw_data[12] = 0x00; //0x00
	rw_data[13] = 0x00; //0x00
	rw_data[14] = 0x00; //0x00
	rw_data[15] = 0x00; //0x00
	I2C_Write(AGS04_Addr, 0x30,rw_data, 15);
	Delay1(10);
*/
	// S/W Reset Disable (Reset Clear, Bit 0 = clear)
	rw_data[0] = 0x4C;
	I2C_Write(AGS04_Addr, 0x02,rw_data, 1);
	Delay1(10); //Delay time few msec

//----- ---------AGS04 initialization --------------------------------
}

// ---------------------------READ Function ---------------------------
// READ OUTPUT DATA AGS04
void READ_AGS04(void)
{
	unsigned char rd_data[10];
	// unsigned char rw_data[2];
	unsigned char LED_CONTROL;
	//unsigned char i;
	//unsigned char j;
	LED_CONTROL = 0;

	// Output Register 0x00h, "1"= Touch On, "0"=Touch Off
	// Register Map => (0000[CS4][CS3][CS2][CS1])
	I2C_Read(AGS04_Addr,0x00,rd_data,1);
	Delay1(10);

	//---- Output data , send UART port
	UART_Byte_Tx = (rd_data[0]); //Debug Code
	serial_send();	//Debug Code
	//hex_conv(rd_data[0]); //Output Register, Data Send to UART
	Delay(50);
	//Key_data[i] = rd_data[0];

	LED_CONTROL = rd_data[0] & 0x01;
	if(LED_CONTROL)
		LED1 =0; //on
	else
		LED1 = 1; //off
	LED_CONTROL = 0;

	LED_CONTROL = rd_data[0] & 0x02;
	if(LED_CONTROL)
		LED2 = 0; //on
	else
		LED2 = 1; //off
	LED_CONTROL = 0;

	LED_CONTROL = rd_data[0] & 0x04;
	if(LED_CONTROL)
		LED3 = 0; //on
	else
		LED3 = 1; //off
	LED_CONTROL = 0;

	LED_CONTROL = rd_data[0] & 0x08;
	if(LED_CONTROL)
		LED4 = 0; //on
	else
		LED4 = 1; //off
	LED_CONTROL = 0;
}
// END of File
