/*
 * File:   newmain.c
 * Author: Jim
 *
 * Created on April 29, 2022, 12:49 AM
 */


// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_XT        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define F_CPU 8000000/64
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)
#define _XTAL_FREQ 8000000

#define RS LATE0                   /* PORTD 0 pin is used for Register Select */
#define EN LATE1                   /* PORTD 1 pin is used for Enable */
#define ldata LATD                 /* PORTB is used for transmitting data to LCD */

#define LCD_Port TRISD              
#define LCD_Control TRISE
#define vref 5.00 // ADC


void ms_delay(int N){
    do
    __delay_ms(1);
    while (--N);
}

void I2C_Master_Init(const unsigned long c)
{
    
//    OSCCON = 0b01111010; // orig code
    OSCCON = 0b01110010; //Primary osc used
//    OSCCON = 0b01100010; // trying because uart random characters
    SSPCON1 = 0x28;
    SSPCON2 = 0x00;
    SSPADD = (_XTAL_FREQ/(4*c))-1; // Set I2C/ SMBus clock speed
    SSPSTAT = 0xc0;
    //SSPSTATbits.CKE = 1; // Use SMBus input signal levels (bit already set in previous step)
    TRISB0 = 1; //Setting as input as given in datasheet
    TRISB1 = 1; //Setting as input as given in datasheet
}

void I2C_Master_Wait()
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
    I2C_Master_Wait();
    SEN = 1;
}

void I2C_Master_RepeatedStart()
{
    I2C_Master_Wait();
    RSEN = 1;
}


void I2C_Master_Stop()
{
    I2C_Master_Wait();
    PEN = 1;
}

signed char I2C_Master_Write(unsigned d)
{
    I2C_Master_Wait();
    SSPBUF = d;
    // optional
    // while(SSPSTATbits.R_W); // wait until write cycle completed
    // if(SSPCON2bits.ACKSTAT)
    // return -2; // if we receive NAK
    return 0;
}

unsigned char I2C_Master_Read(unsigned short a)
{
    unsigned short temp;
    I2C_Master_Wait();
    RCEN = 1;
    I2C_Master_Wait();
    temp = SSPBUF; // Read data from SSPBUF
    I2C_Master_Wait();
    ACKDT = (a)?0:1;
    ACKEN = 1;
    return temp;
}



/////////////////////////////////////   LCD - START   //////////////////////////////////////////////////

void LCD_Command(char cmd)
{
	ldata= cmd;            /* Send data to PORT as a command for LCD */   
	RS = 0;                /* Command Register is selected */
	EN = 1;                /* High-to-Low pulse on Enable pin to latch data */ 
	NOP();
	EN = 0;
	ms_delay(3);	
}

void LCD_Init()
{
    ms_delay(15);           /* 15ms,16x2 LCD Power on delay */
    LCD_Port = 0x00;       /* Set PORTB as output PORT for LCD data(D0-D7) pins */
    LCD_Control = 0x00;    /* Set PORTD as output PORT LCD Control(RS,EN) Pins */
    LCD_Command(0x38);     /* uses 2 line and initialize 5*7 matrix of LCD */
    LCD_Command(0x01);     /* clear display screen */
    LCD_Command(0x0c);     /* display on cursor off */
    LCD_Command(0x06);     /* increment cursor (shift cursor to right) */
}

void LCD_Clear()
{
    	LCD_Command(0x01); /* clear display screen */
}

void LCD_Char(char dat)
{
	ldata= dat;            /* Send data to LCD */  
	RS = 1;                /* Data Register is selected */
	EN=1;                  /* High-to-Low pulse on Enable pin to latch data */   
	NOP();
	EN=0;
	ms_delay(1);
}


void LCD_String(const char *msg)
{
	while((*msg)!=0)
	{		
	  LCD_Char(*msg);
	  msg++;	
    	}
		
}

void LCD_String_xy(char row,char pos,const char *msg)
{
    char location=0;
    if(row<=1)
    {
        location=(0x80) | ((pos) & 0x0f); /*Print message on 1st row and desired location*/
        LCD_Command(location);
    }
    else
    {
        location=(0xC0) | ((pos) & 0x0f); /*Print message on 2nd row and desired location*/
        LCD_Command(location);    
    }  
    LCD_String(msg);

}

/////////////////////////////////      LCD - END     ///////////////////////////////////////////////////

//////////////////////////////// USART - START //////////////////////////////////////////

/*****************************USART Initialization*******************************/
void USART_Init(long baud_rate)
{
    float temp;
    TRISC6=0;                       /*Make Tx pin as output*/
    TRISC7=1;                       /*Make Rx pin as input*/
    temp=Baud_value;     
    SPBRG=(int)temp;                /*baud rate=9600, SPBRG = (F_CPU /(64*9600))-1*/
//    SPBRG = 7;
    TXSTA=0x20;                     /*Transmit Enable(TX) enable*/ 
    RCSTA=0x90;                     /*Receive Enable(RX) enable and serial port enable */
    
}
/******************TRANSMIT FUNCTION*****************************************/ 
void USART_TxChar(char out)
{        
        while(TXIF==0);            /*wait for transmit interrupt flag*/
        TXREG=out;                 /*wait for transmit interrupt flag to set which indicates TXREG is ready
                                    for another transmission*/    
}
/*******************RECEIVE FUNCTION*****************************************/
char USART_RxChar()
{
   
    
    while(RCIF==0);                 /*wait for receive interrupt flag*/
    if(RCSTAbits.OERR)
    {           
        CREN = 0;
        NOP();
        CREN=1;
    }
    return(RCREG);                  /*receive data is stored in RCREG register and return to main program */
}

void USART_SendString(const char *out)
{
   while(*out!='\0')
   {            
        USART_TxChar(*out);
        out++;
   }
   USART_TxChar(0x0D);
   USART_TxChar(0x0A);
}

/////////////////////////////// USART - END ////////////////////////////////////////////

////////////////////////////// MQ3 - START ////////////////////////////////////////////

/*    
    < 120 is sober
    120-400 is drinking ? but within legal limits
    > 400 is drunk
 */

void ADC_Init()
{    
    TRISA = 0xff;		/*Set as input port*/
    ADCON1 = 0x0e;  		/*Ref vtg is VDD & Configure pin as analog pin*/    
    ADCON2 = 0x92;  		/*Right Justified, 4Tad and Fosc/32. */
    ADRESH=0;  			/*Flush ADC output Register*/
    ADRESL=0;   
}

int ADC_Read(int channel)
{
    int digital;
    ADCON0 =(ADCON0 & 0b11000011)|((channel<<2) & 0b00111100);

    /*channel 0 is selected i.e.(CHS3CHS2CHS1CHS0=0000)& ADC is disabled*/
    ADCON0 |= ((1<<ADON)|(1<<GO));/*Enable ADC and start conversion*/

    /*wait for End of conversion i.e. Go/done'=0 conversion completed*/
    while(ADCON0bits.GO_nDONE==1);

    digital = (ADRESH*256) | (ADRESL);/*Combine 8-bit LSB and 2-bit MSB*/
    return(digital);
}

///////////////////////////// MQ3 - END ///////////////////////////////////////////////

// Define register variables read in from RAM (object 1) for sensors 1 and 2
unsigned char high_byte;
unsigned char low_byte;
unsigned char pec_byte;
unsigned char high_byte2,low_byte2,pec_byte2;
// Define register variables read from ambient temp for sensor 1
unsigned char Ta_high_byte;
unsigned char Ta_low_byte;
unsigned char Ta_pec_byte;
// Define variables for calculating
unsigned int tempData = 0x0000; // Holds temp data in hex
float temp = 0; // holds object 1 temperature in kelvin
float Ctemp,Ctemp2 = 20; // Initialize object 1 temp in celsius
float Atemp = 0; // holds ambient temperature
float ACtemp = 40; // Initialize ambient temp

void main()
{
    TRISEbits.RE2 = 0; //set re2 as output for buzzer
// LCD start
    
    LCD_Init();                    /* Initialize 16x2 LCD */
    LCD_String_xy(1,0,"Starting...");    /* Display string at location(row,location). */
                                   /* This function passes string to display */
    LCD_String_xy(2,0,"Please wait.");
    
    
// LCD end    
    char data[10];    // MQ3
    char data2[10];    // MQ32
    int digital;       // MQ3
    float voltage;      // MQ3
    char cTempString[15]; // temp value in string
    char sensorStringValues[50];
    
    float initial_b_reading;      // MQ3
    float final_reading;      // MQ3
    
// Define SMBus Addresses
    char slave_address = 0x04; // Define SMBus Addr for 1st sensor
    char slave_address_read = slave_address*2 + 1;// read command
    char slave_address_write = slave_address*2;
    char slave_address2 = 0x5A; // Define SMBus Addr for 2nd sensor
    char slave_address_read2 = slave_address2*2 + 1; // read command
    char slave_address_write2 = slave_address2*2;
    // Define RAM Access Addresses (same on each device)
    char TaReg = 0x06; // RAM address for ambient temperature
    char To1Reg = 0x07; // RAM address for object 1 temperature
    char To2Reg = 0x08; // RAM address for object 2 temperature
    // start I2C
    I2C_Master_Init(100000); //Initialize I2C Master with 100KHz clock
    
    ////// USART
    USART_Init(9600);
    USART_SendString("This message is from PIC");
    ////// USART
    
    ADC_Init();
    
    ms_delay(5000);
    LCD_Clear();

    while(1){
        // For Sensor 2
        // Read object 1 temp
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(slave_address_write2); // send write to slave address 0x00
        I2C_Master_Write(To1Reg); // send RAM address for Ta
        I2C_Master_RepeatedStart(); // send repeat start
        I2C_Master_Write(slave_address_read2); // send read to slave address 0x00
        // read 3 bytes
        low_byte2 = I2C_Master_Read(0); // Read + Acknowledge
        high_byte2 = I2C_Master_Read(0); // Read + Acknowledge
        pec_byte2 = I2C_Master_Read(1); // Read + NotAcknowledge
        I2C_Master_Stop();
        // Convert bytes to temperature data
        // Calculate object 1 temperature, sensor 1
//        tempData = high_byte*256; // Add high byte temp data
//        tempData = tempData + low_byte; // Add low byte temp data
//        temp = tempData*0.02; // Convert temp to Kelvin
//        Ctemp = temp - 273.15; // Convert temp to celsius
        // Calculate object 1 temperature, sensor 2
        tempData = high_byte2*256; // Add high byte temp data
        tempData = tempData + low_byte2; // Add low byte temp data
        temp = tempData*0.02; // Convert temp to Kelvin
        Ctemp2 = temp - 273.15; // Convert temp to celsius
        // Calculate ambient temperature
//        tempData = Ta_high_byte*256; // Add high byte temp data
//        tempData = tempData + Ta_low_byte; // Add low byte temp data
//        Atemp = tempData*0.02; // Convert temp to Kelvin
//        ACtemp = Atemp - 273.15; // Convert temp to celsius
        
        sprintf(cTempString,"%.2f",Ctemp2); //
        strcat(cTempString, " C");
        
        // MQ3 LOGIC - START
        
        digital=ADC_Read(0);

	/*Convert digital value into analog voltage*/
        voltage= digital*((float)vref/(float)1023);  
        //---------
        initial_b_reading=0.50;        
        final_reading = voltage - initial_b_reading;
        
        if(final_reading<0)
        {
            final_reading = 0.00;
        }else{
            final_reading = voltage;
        }
	/*It is used to convert integer value to ASCII string*/
        //sprintf(data,"%.2f",voltage);
        sprintf(data,"%.2f",final_reading);
        sprintf(data2,"%.2f",voltage);

        strcat(data," %");	/*Concatenate result and unit to print*/
        strcat(data2," R");
        
        
        //if (voltage <= 3.6)
        if (final_reading <=0 && Ctemp2 < 38)  
        {
            
            
            
            USART_SendString(data);
            USART_SendString(data2);
            USART_SendString(cTempString);
            LCD_Clear();
            
            LCD_String_xy(1,1,data);/*Send string data for printing*/
            LCD_String_xy(2,1,cTempString);
            ms_delay(1000); // Delay 200 ms between reads
        }
        else
        {
            LCD_Clear();
            if(final_reading > 0 && Ctemp2 >= 38)
            {
                LCD_String_xy(1,0, "Alcohol Detected");
                LCD_String_xy(1,0, "Fever");
            }
            else if(Ctemp2 >= 38)  
            {
                LCD_String_xy(1,0, "Fever");
                LCD_String_xy(2,0, cTempString);
            }
            else
            {
              LCD_String_xy(1,0, "Alcohol Detected");
              LCD_String_xy(2,0, data);
            }
            PORTEbits.RE2 = 1; //send buzzer
            ms_delay(3000);
            PORTEbits.RE2 = 0; //send buzzer
        }
        
        // MQ3 LOGIC - END
        
        
    }
}