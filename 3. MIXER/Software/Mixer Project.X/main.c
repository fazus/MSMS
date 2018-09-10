#include <xc.h>
#include <limits.h>
#include <dsp.h>
#include <math.h>
#include <libq.h>
#include "conf.h"

#pragma config BWRP = WRPROTECT_OFF    // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF              // Watchdog Timer Enable (Watchdog timer always enabled)
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)
 
    int kupa=0;
char kutas=0;

int BufferA[8] __attribute__((space(dma)));
int AnalogInput[8];
    

_Q15 sins_table[1024];
_Q15 saw_table[1024];
_Q15 triangle_table[1024];
_Q16 phase[3] ={0, 0, 0,};
_Q15 sample(_Q15 we, unsigned short with);

char RX_BUFF[8];
char moc=0;

typedef struct{
    
    _Q16 output_q16;
    _Q15 output;
    _Q15 A_diff;
    _Q15 A_limit;
    _Q15 R_diff;
    _Q15 R_limit;
    char state;
}eg_type;

typedef struct{
    _Q16 sample_buff_a;
    _Q15 sample_buff_b;
    _Q15 sample_buff_c;
    _Q15 freq_offset;
    _Q15 *output_p; 
    _Q15 OUT; 
    _Q15 output_gain;
    _Q15 *IN1; 
    _Q15 *IN2;
    char state;
    char *trig;
    eg_type EG;    
    
}fm_op_type;

fm_op_type op_fm[6];
    
_Q15 OP_FM(fm_op_type *op);
_Q15 get_op_output(fm_op_type *op);
int i;
    int cipa;
    

_Q15 pipa,pipa1;
_Q15 zero=0;
_Q16 pipa_q6;
void dds_arrays_prep(void); 
_Q15 wave_assemble(unsigned short with);
char OP_FM_state=0;
int k;
char trig;
int main(void) {
    
    
    
    
    
    config();
    DMA0STA = __builtin_dmaoffset(BufferA); 
    DMA0CONbits.CHEN=1;    // Enable DMA
   
    dds_arrays_prep();
    
    for(k=0;k<6;k++)op_fm[k].EG.state=0;  
    
    for(k=0;k<6;k++)op_fm[k].EG.A_limit=SHRT_MAX/2;

    for(k=0;k<6;k++)op_fm[k].EG.R_diff = 0;
    
    
    for(k=0;k<6;k++)op_fm[k].EG.A_diff = 21000;

    for(k=0;k<6;k++)op_fm[k].EG.R_diff = -21000;
    
    for(k=0;k<6;k++)op_fm[k].state=0;
    
    for(k=0;k<6;k++)op_fm[k].trig=&trig;

    
    op_fm[0].IN1=&pipa;
    op_fm[0].IN2=&pipa1;
    

    

    
    for(k=0;k<6;k++)op_fm[k].output_gain = 0;
    

while(1) {
    pipa1= RX_BUFF[0]*4;
    pipa=2200;
    
    if(i>22000){
    i=0;
   
       trig = 1; 


    }
}
    return 0;
}



void dds_arrays_prep(void)
{
    short ii_pcf=0;
        
    for(ii_pcf=0; ii_pcf<1024;ii_pcf++)
     sins_table[ii_pcf] =  _Q15sinPI(SHRT_MIN+ii_pcf*64)>>1; 
    
}


void __attribute__((interrupt, no_auto_psv))_DAC1RInterrupt(void)
{
    
    i++;
IFS4bits.DAC1RIF = 0;                    /* Clear Right Channel Interrupt Flag */
   // for(k=0;k<6;k++)

OP_FM(&op_fm[0]);
     DAC1RDAT = AnalogInput[0]<<4;
     DAC1LDAT =kupa*get_op_output(&op_fm[0]);// = AnalogInput[2];// sample(10,0,0);
}



void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void)
{
    
    RX_BUFF[moc++]=0xFF&U1RXREG;
    if(U1RXREG&0x100){moc=0;kupa=1;}
    
    IFS0bits.U1RXIF = 0;   
}
_Q15 get_op_output(fm_op_type *op){return (*op).OUT;}


_Q15 OP_FM(fm_op_type *op){
    
    if( (*(*op).trig) > 0 ){
        (*op).EG.state = 1;
        (*op).state = 1;
    }
    
    
    switch((*op).EG.state)
    {
        default:
            
            (*op).EG.output = 0;
             (*op).EG.output_q16 = 0;
            break;
        case 1:
           (*op).EG.output_q16 += (*op).EG.A_diff;
            (*op).EG.output = (*op).EG.output_q16 >> 12;
            
            if((*op).EG.output_q16&0x80000000)(*op).EG.output |= 0x8000;
            else (*op).EG.output &= 0x7FFF;
            
           if( (*op).EG.output >= (*op).EG.A_limit){
            (*op).EG.state = 2;}
            
            
            break;
        case 2:
            
            (*op).EG.output_q16 += (*op).EG.R_diff;
            (*op).EG.output = (*op).EG.output_q16 >> 12;
            if((*op).EG.output_q16&0x80000000)(*op).EG.output |= 0x8000;
            else (*op).EG.output &= 0x7FFF;
            
            if((*op).EG.output < (*op).EG.R_limit ) (*op).EG.state = 0;
            
            break;
    }
    
    if((*op).EG.state==0)(*op).state = 0;
    else (*(*op).trig) =0;
        
        
    (*op).output_gain = (*op).EG.output;
    
    switch((*op).state){
        case 0:
            (*op).OUT = 0;
            break;
        default:
        
            
            (*op).sample_buff_b = _Q15add( (*(*op).IN1), (*(*op).IN2));  
            (*op).sample_buff_b = sample( (*op).sample_buff_b, 0);
            (*op).OUT = _Q16mpy((*op).sample_buff_b,(*op).output_gain);
            
    
            
            break;
    }
    return (*op).OUT;
}



_Q15 sample(_Q15 we,unsigned short with)
{
    _Q15 phs=0;
    phase[with] += (_Q16)0x21AF*we;
    
    phs=phase[with]>>16;
 
  phs>>=6;
  if(phs>0x3FF)phs=0;
  return sins_table[phs&0x3FF];
}










void __attribute__((interrupt, no_auto_psv))_DAC1LInterrupt(void) {
    
    IFS4bits.DAC1LIF = 0; /* Clear Left Channel Interrupt Flag */ 
}

void __attribute__((interrupt, no_auto_psv))_ADC1Interrupt(void) {
    
    
    
    
    IFS0bits.AD1IF = 0;
}

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void)
{
 
  AnalogInput[0] = BufferA[0];
  AnalogInput[1]= BufferA[1];
  
  AnalogInput[2] = BufferA[2];
  AnalogInput[3]= BufferA[3];
  
  AnalogInput[4] = BufferA[4];

IFS0bits.DMA0IF = 0;      //Clear the DMA5 Interrupt Flag
}


_Q15 wave_assemble(unsigned short with){
    _Q15 output;
  
    
    return output;
}