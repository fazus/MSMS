#include "conf.h"


int ExampleLocalFunction(int param1){
    
    return param1;
}

int filtr(int we){
    
    return we;
}

void config(void){
        
    TRISBbits.TRISB15 = 0;
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB6 = 0;
    LATBbits.LATB6 = 0;

 CLKDIVbits.PLLPRE = 0; // N1=2 and N1 = PLLPRE + 2, gives FREF = 7.3728/2 = 3.6864 
 PLLFBD = 40; // M=40 gives FVCO = 40 * 3.6864 = 147.456 MHz.

 CLKDIVbits.PLLPOST = 0; // N2=2 and N2 = 2 * (PLLPOST + 1)
    // This gives FOSC = FVCO/2 = 73.728 MHz, FCY = FOCS/2 = 36.864 MHz, 37 MIPS
 OSCTUN = 0; // No tuning of the oscillator is required
 __builtin_write_OSCCONH(0x01); /* Initiate Clock Switch to FRC with PLL*/
 __builtin_write_OSCCONL(0x01);

 while (OSCCONbits.COSC != 0b01); /* Wait for Clock switch to occur */
 while(!OSCCONbits.LOCK);
 

    ACLKCONbits.SELACLK = 0; // FRC + PLL as clock
    ACLKCONbits.AOSCMD = 0; // Disable ACLK (PLL used instead)
    ACLKCONbits.ASRCSEL = 0; // Use auxiliary oscillator (not used)
    ACLKCONbits.APSTSCLR = 7; // ACLK = FVCO/1 = 147.456 MHz
    

    
    
    
    DAC1STATbits.ROEN = 1;/* Right Channel DAC Output Enabled */
    DAC1STATbits.LOEN = 1;/* Left Channel DAC Output Enabled */
    DAC1STATbits.RITYPE = 0;/* Right Channel Interrupt if FIFO is not Full */
    DAC1STATbits.LITYPE = 0;/* Left Channel Interrupt if FIFO is not Full */
    DAC1CONbits.AMPON = 0;/* Amplifier Disabled During Sleep and Idle Modes */
    DAC1CONbits.DACFDIV = 11;/* Divide PLL FVCO by 12 (zero-based counting) */
    DAC1CONbits.FORM = 1;/* Data Format is signed */
    DAC1DFLT = 0x0;/* Default value set to Midpoint */

    IFS4bits.DAC1RIF = 0;/* Clear Right Channel Interrupt Flag */
    IFS4bits.DAC1LIF = 0;/* Clear Left Channel Interrupt Flag */
    IEC4bits.DAC1RIE = 1;/* Right Channel Interrupt Enabled */
    IEC4bits.DAC1LIE = 1;/* Left Channel Interrupt Enabled */

    DAC1CONbits.DACEN = 1;/* DAC1 Module Enabled */

    AD1CON1bits.FORM   = 1;  // Data Output Format: Signed Fraction (Q15 format)
    AD1CON1bits.SSRC   = 2;  // Sample Clock Source: GP Timer starts conversion
    AD1CON1bits.ASAM   = 1;  // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B  = 1;  // 12-bit ADC operation
    AD1CON1bits.ADDMABM = 1;
    
    AD1CON2bits.VCFG = 0; //1;  // VREF+ external VREF-: VSS
    AD1CON2bits.CSCNA = 1;  // Scan Input Selections for CH0+ during Sample A bit
    AD1CHS0bits.CH0SA    = 0;    // MUXA +ve input selection (AN0) for CH0, IS IGNORED WHEN AD1CON2bits.CSCNA IS SET!!
    AD1CHS0bits.CH0NA    = 0;    // MUXA -ve input selection (Vrefl, instead of AN1) for CH0
    AD1CON3bits.ADRC = 0;  // ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS = 63;  // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
       // ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us
    
    
    AD1CON1bits.ADDMABM = 1;  // DMA buffers are built in scatter/gather mode
    AD1CON2bits.SMPI    = 4; // 6 ADC Channel is scanned
    AD1CON4bits.DMABL   = 0; // Each buffer contains 8 words
    //AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
    AD1CSSL = 0x001F; // 0,1,2,3,4,5 selected for scannng
    AD1PCFGL= 0xFFC0; //0xf0c0; // 0,1,2,3,4,5 as analog Inputs
 
  
    IFS0bits.AD1IF   = 0;  // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE   = 0;//1;  // Do Not Enable A/D interrupt 
    AD1CON1bits.ADON = 1;  // Turn on the A/D converter
        
        TMR3 = 0x0000;
        PR3 = 100;
        IFS0bits.T3IF = 0;
        IEC0bits.T3IE = 0;

        //Start Timer 3
        T3CONbits.TON = 1;

    DMA0CONbits.AMODE = 2;   // Configure DMA for Peripheral indirect mode
    DMA0CONbits.MODE  = 0;   // Continuous, Ping-Pong modes disabled
    DMA0PAD=(int)&ADC1BUF0;
    DMA0CNT = 4; // (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;
    DMA0REQ = 13;     // Select ADC1 as DMA Request source
    
   
            
            
            
// DMA0STB = __builtin_dmaoffset(BufferB);
  IFS0bits.DMA0IF = 0;   //Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1;   //Set the DMA interrupt enable bit
    
   
    
}