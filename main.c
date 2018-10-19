// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 16000000  

// Pin MACROS
#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0
#define CHAR_ON '1'
#define CHAR_OFF '0'
#define OUTPUT_RELAY1 PORTFbits.RF1
#define OUTPUT_RELAY2 PORTFbits.RF0
#define OUTPUT_RELAY3 PORTAbits.RA3
#define OUTPUT_RELAY4 PORTAbits.RA2

#define OUTPUT_RELAY_DIR_1 TRISFbits.TRISF1
#define OUTPUT_RELAY_DIR_2 TRISFbits.TRISF0
#define OUTPUT_RELAY_DIR_3 TRISAbits.TRISA3
#define OUTPUT_RELAY_DIR_4 TRISAbits.TRISA2

#define INPUTSWITCH1 PORTFbits.RF7
#define INPUTSWITCH2 PORTFbits.RF5
#define INPUTSWITCH3 PORTFbits.RF3
#define INPUTSWITCH4 PORTFbits.RF2


#define INPUT_SWITCH_DIR_1 TRISFbits.TRISF7
#define INPUT_SWITCH_DIR_2 TRISFbits.TRISF5
#define INPUT_SWITCH_DIR_3 TRISFbits.TRISF3
#define INPUT_SWITCH_DIR_4 TRISFbits.TRISF2

#define UART_TX_DIR TRISCbits.TRISC6
#define UART_RX_DIR TRISCbits.TRISC7

//Debugging

//#define DEBUG

char XbeeDataReceiveBuffer[20]="#";
char CopyXbeeDataReceiveBuffer[10]="#";
char ChildLockBuffer[5]="00000";
unsigned XbeeDataReceiveBufferPosition=0;
unsigned XbeeDataReceived=0;


unsigned int M1=0;unsigned int M2=0;unsigned int M3=0;unsigned int M4=0;
void Gpio_Init();
void pinINIT_extra();
void clearAllPorts();
void allPeripharal();
void UartInit();
void CopyReceivedData();
void applicationControl(char switchNumberMSB, char switchNumberLSB, char switchStatus, char SpeedMSB, char SpeedLSB,char ChildLock,char FinalBit);
#include"include.h"
interrupt void isr()
{
    if(RC1IF == 1)
    {
        XbeeDataReceiveBuffer[XbeeDataReceiveBufferPosition]=RC1REG;
#ifdef DEBUG
        TX1REG=XbeeDataReceiveBuffer[XbeeDataReceiveBufferPosition];
#endif
        if(XbeeDataReceiveBuffer[0] == '%')
        {
            XbeeDataReceiveBufferPosition++;
            if(XbeeDataReceiveBufferPosition>15)
            {
              //  TX1REG='O';__delay_ms(1);TX1REG='K';__delay_ms(1);
                XbeeDataReceived=TRUE;
                XbeeDataReceiveBufferPosition=0;
            }
        }
        
    }
}
int main()
{
    Gpio_Init();
   allPeripharal();
    OUTPUT_RELAY1=0;OUTPUT_RELAY2=0;OUTPUT_RELAY3=0;OUTPUT_RELAY4=0;
    M1=OFF;M2=OFF;M3=OFF;M4=OFF;
    while(1)
    {
        if(XbeeDataReceived == TRUE)
        {
            XbeeDataReceived=FALSE;
           // TX1REG='O';__delay_ms(1);TX1REG='K';__delay_ms(1);
            CopyReceivedData();
//            TX1REG=CopyXbeeDataReceiveBuffer[0];__delay_ms(1);
//            TX1REG=CopyXbeeDataReceiveBuffer[1];__delay_ms(1);
//            TX1REG=CopyXbeeDataReceiveBuffer[2];__delay_ms(1);
//            TX1REG=CopyXbeeDataReceiveBuffer[3];__delay_ms(1);
//            TX1REG=CopyXbeeDataReceiveBuffer[4];__delay_ms(1);
//            TX1REG=CopyXbeeDataReceiveBuffer[5];__delay_ms(1);
//            TX1REG=CopyXbeeDataReceiveBuffer[6];__delay_ms(1);
            applicationControl(CopyXbeeDataReceiveBuffer[0],CopyXbeeDataReceiveBuffer[1],
                    CopyXbeeDataReceiveBuffer[2],CopyXbeeDataReceiveBuffer[3],
                    CopyXbeeDataReceiveBuffer[4],CopyXbeeDataReceiveBuffer[5],CopyXbeeDataReceiveBuffer[6]);
            
            
        }
        
        
        /************manual response*/
        //switch1
        if(ChildLockBuffer[1] ==  CHAR_OFF && INPUTSWITCH1 == 1 && M1 == OFF){
            OUTPUT_RELAY1=1;
            TX1REG='R';__delay_ms(1);
            TX1REG='1';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='1';__delay_ms(1);
            M1=ON;

        }
         if(ChildLockBuffer[1] ==  CHAR_OFF && INPUTSWITCH1 == 0 && M1 == ON){
            OUTPUT_RELAY1=0;
            TX1REG='R';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='1';__delay_ms(1);
            M1=OFF;

        }
       //switch2
        if(ChildLockBuffer[2] ==  CHAR_OFF && INPUTSWITCH2 == 1 && M2 == OFF){
            OUTPUT_RELAY2=1;
            TX1REG='R';__delay_ms(1);
            TX1REG='1';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='2';__delay_ms(1);
            M2=ON;

        }
         if(ChildLockBuffer[2] ==  CHAR_OFF && INPUTSWITCH2 == 0 && M2 == ON){
            OUTPUT_RELAY2=0;
            TX1REG='R';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='2';__delay_ms(1);
            M2=OFF;

        }
        //switch3
        if(ChildLockBuffer[3] ==  CHAR_OFF && INPUTSWITCH3 == 1 && M3 == OFF){
            OUTPUT_RELAY3=1;
            TX1REG='R';__delay_ms(1);
            TX1REG='1';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='3';__delay_ms(1);
            M3=ON;

        }
         if(ChildLockBuffer[3] ==  CHAR_OFF && INPUTSWITCH3 == 0 && M3 == ON){
            OUTPUT_RELAY3=0;
            TX1REG='R';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='3';__delay_ms(1);
            M3=OFF;

        }
      //switch4
        if(ChildLockBuffer[4] ==  CHAR_OFF && INPUTSWITCH4 == 1 && M4 == OFF){
            OUTPUT_RELAY4=1;
            TX1REG='R';__delay_ms(1);
            TX1REG='1';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='4';__delay_ms(1);
            M4=ON;

        }
         if(ChildLockBuffer[4] ==  CHAR_OFF && INPUTSWITCH4 == 0 && M4 == ON){
            OUTPUT_RELAY4=0;
            TX1REG='R';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='0';__delay_ms(1);
            TX1REG='4';__delay_ms(1);
            M4=OFF;

        }
    }
    
}
void allPeripharal(){
    UartInit();
}

void Gpio_Init()
{
    pinINIT_extra();
    clearAllPorts();
    OUTPUT_RELAY_DIR_1=0;
    OUTPUT_RELAY_DIR_2=0;
    OUTPUT_RELAY_DIR_3=0;
    OUTPUT_RELAY_DIR_4=0;
    INPUT_SWITCH_DIR_1=1;
    INPUT_SWITCH_DIR_2=1;
    INPUT_SWITCH_DIR_3=1;
    //INPUT_SWITCH_DIR_4=1;
    
    
    UART_TX_DIR=0;
    UART_RX_DIR=1;
    
}

void UartInit()
{
    GIE=1;
    PEIE=1;
    PIE1bits.RC1IE = 0;
    PIE1bits.TX1IE = 0;

    // Set the EUSART module to the options selected in the user interface.

    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE enabled; ABDEN disabled;
    BAUD1CON = 0x0A;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
    RC1STA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave;
    TX1STA = 0x24;

    // Baud Rate = 9600; SP1BRGL 12;
    //SPBRGL = 0x0C;
    //SPBRGL = 0x19;                  // SP1BRGL is 25 (hex value=0x19) for 9600 baud on 16 MHz crystal frequency
    SP1BRGL = 0xA0;                  // SYNC =0 ; BRGH = 1 ; BRG16=1;
    // Baud Rate = 9600; SP1BRGH 1;
    SP1BRGH = 0x01;

    
    PIE1bits.RC1IE = 1;
   // PIE1bits.TX1IE = 1;
    // Transmit Enabled
    TX1STAbits.TXEN = 1;

    // Serial Port Enabled
    RC1STAbits.SPEN = 1;
}
void pinINIT_extra(){
    ANSELG=0x00;    WPUG = 0;
    
    ANSELF=0x00;
    
    ANSELE=0x00;    WPUE=0x00;
    
    ANSELD=0x00;    WPUD=0x00;
    
    ANSELB=0x00;    WPUB=0x00;
    
    ANSELA=0x00;     
} 
void CopyReceivedData(){
    int CopyXbeeDataReceiveBufferPosition=0;
    for(CopyXbeeDataReceiveBufferPosition = 2;CopyXbeeDataReceiveBufferPosition<9;CopyXbeeDataReceiveBufferPosition++)
    {
        CopyXbeeDataReceiveBuffer[CopyXbeeDataReceiveBufferPosition-2]=XbeeDataReceiveBuffer[CopyXbeeDataReceiveBufferPosition];
        XbeeDataReceiveBuffer[CopyXbeeDataReceiveBufferPosition]="#";
       
    }
}
/*
 * always clear all the ports before initialization
 */
void clearAllPorts(){
    OUTPUT_RELAY1=0;
    OUTPUT_RELAY2=0;
    OUTPUT_RELAY3=0;
    OUTPUT_RELAY4=0;
    
}
