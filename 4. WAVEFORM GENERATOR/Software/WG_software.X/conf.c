#include "conf.h"
void init_p(void){
PLLFBD=71;                        //                        M=65
CLKDIVbits.PLLPOST=0;             //             N2=2
CLKDIVbits.PLLPRE=0;              //              N1=3


    TRISBbits.TRISB15 = 1; // UART1 RX
    RPINR18bits.U1RXR = 47; 
 

    U1BRG = UBRG(UART1_BAUD); 
    U1STAbits.URXISEL = 0;
    IPC2bits.U1RXIP = 2;
    IFS0bits.U1RXIF = 0;
    U1MODEbits.PDSEL=3;
    
    U1MODEbits.UARTEN = 1;
    
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    
    
    TRISBbits.TRISB0 =0;
    ANSELBbits.ANSB0 = 0;
    
    TRISBbits.TRISB9 =0;
 //   _RP41R=0xA;//SS2 
    
    TRISBbits.TRISB8 =0;
 //   _RP40R=8;//SDO2
    
     TRISBbits.TRISB7 =0;
 //   _RP39R=9;//SCK2
    
    IFS0bits.SPI1IF = 0;              // Clear the Interrupt flag
    IEC0bits.SPI1IE = 0;              // Disable the interrupt
    
    SPI1CON1bits.PPRE=2;
    SPI1CON1bits.SPRE=4;
    SPI1CON2bits.FRMEN = 1;
    SPI1CON2bits.FRMPOL = 0;
    SPI1CON2bits.SPIFSD=0;
    
    SPI1CON1bits.MODE16 = 1;          // Communication is word-wide (16 bits)
    SPI1STATbits.SISEL=6;
    
    SPI1CON1bits.MSTEN = 1;           // Master mode enabled
    SPI1STATbits.SPIEN = 1;           // Enable SPI module

    SPI1BUF = 0x0000;                 // Write data to be transmitted 
    // Interrupt Controller Settings
    
    IPC2bits.SPI1IP = 3;
    IFS0bits.SPI1IF = 0;              // Clear the Interrupt flag
  //  IEC0bits.SPI1IE = 1;              // Enable the interrupt
    
}
