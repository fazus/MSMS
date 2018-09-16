
// DSPIC33EP512GP502 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "conf.h"


char RX_BUFF[64];
short moc;
short kupa=0;
short kutas=0;
short cipa=0;
void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void);
int main(void) {


init_p();
    
    TRISAbits.TRISA4 = 0;
LATAbits.LATA4 = 0;
    while (1){
       cipa+=1000; 
    SPI1BUF = cipa;
    for(kutas=0;kutas<100;kutas++);
            if(RX_BUFF[0]==15)kupa=1;
            if(RX_BUFF[0]==16){kupa=0;kutas=0;}
     if(kupa)LATAbits.LATA4 =1;   
     else LATAbits.LATA4 =0;   
        
        
    }
}






void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void) 
{

    short rx_buff = U1RXREG;
    RX_BUFF[moc++]=0xFF&rx_buff;
    if(rx_buff&0x100){moc=0;}

    IFS0bits.U1RXIF = 0;   
}


void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{
    

IFS0bits.SPI1IF = 0;
}
