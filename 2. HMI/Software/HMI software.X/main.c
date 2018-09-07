
#include <xc.h>
#include <sys/attribs.h>   
#include <sys/kmem.h>
#include <stdio.h>

// USERID = No Setting
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = OFF             // Peripheral Pin Select Configuration (Allow only one reconfiguration)

// DEVCFG2
#pragma config FPLLIDIV = DIV_1       // PLL Input Divider (12x Divider)
#pragma config FPLLMUL = MUL_20        // PLL Multiplier (24x Multiplier)
#pragma config FPLLODIV = DIV_2     // System PLL Output Clock Divider (PLL Divide by 256)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = OFF                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.




#define LED LATAbits.LATA4

  
//595 OUTPUT PORTS 
#define DATAa LATBbits.LATB8 
#define CLKa LATBbits.LATB10 
#define La LATBbits.LATB9


int k = 0;
    
int i;


//LCD driver
#define RST LATBbits.LATB11
#define CS LATBbits.LATB4     
#define A0 LATBbits.LATB12    

unsigned char command_buffer[24]={0,0x10,0xB0,0,0x10,0xB1,0,0x10,0xB2,0,0x10,0xB3,0,0x10,0xB4,0,0x10,0xB5,0,0x10,0xB6,0,0x10,0xB7};
unsigned char graphic_buffer[2048];
unsigned char dispbuff[40];
char lcd_page=0;
union {
    unsigned int flags;
        unsigned witch_lcd_page  : 1;
        unsigned transp  : 1;
        unsigned flag_3  : 1;
        unsigned flag_4  : 1;
        unsigned flag_5  : 1;
        unsigned flag_6  : 1;
        unsigned flag_7  : 1;
        unsigned flag_8  : 1;
} lcd_flag = {0};
unsigned char x=0;
unsigned y=0;
char s_counter=0;
char lduze[182]={
0, 0x3E, 0x09, 0x09, 0x09, 0x09, 0x3E, //A
0, 0x3F, 0x25, 0x25, 0x25, 0x25, 0x1A,	//B
0,0x1E,0x21,0x21,0x21,0x21,0x12,	//C
0,0x3F,0x21,0x21,0x21,0x12,0x0C,	//D
0,0x3F,0x25,0x25,0x25,0x25,0x21,	//E
0,0x3F,0x05,0x05,0x05,0x05,0x01,	//F
0,0x1E,0x21,0x21,0x29,0x29,0x12,	//G
0,0x3F,0x04,0x04,0x04,0x04,0x3F,	//H
0,0x21,0x21,0x3F,0x21,0x21,0,	//I
0,0x18,0x20,0x20,0x20,0x20,0x1F,	//J
0,0x3F,0x04,0x04,0x0A,0x11,0x20,	//K
0,0x3F,0x20,0x20,0x20,0x20,0x20,	//L
0,0x3F,0x02,0x04,0x04,0x02,0x3F,	//M
0,0x3F,0x02,0x04,0x08,0x08,0x3F,	//N
0,0x1E,0x21,0x21,0x21,0x21,0x1E,	//O
0,0x3F,0x09,0x09,0x09,0x09,0x06,	//P
0,0x1E,0x21,0x29,0x31,0x21,0x1E,	//Q
0,0x3F,0x09,0x09,0x09,0x19,0x26,	//R
0,0x12,0x25,0x25,0x25,0x25,0x18,	//S
0x01,0x01,0x01,0x3F,0x01,0x01,0x01,	//T
0,0x1F,0x20,0x20,0x20,0x20,0x1F, //U
0,0x0F,0x10,0x20,0x20,0x10,0x0F, //V
0,0x1F,0x20,0x10,0x10,0x20,0x1F,	//W
0,0x21,0x12,0x0C,0x0C,0x12,0x21,	//X teraz #
0x01,0x02,0x04,0x38,0x04,0x02,0x01,	//Y
0,0x21,0x31,0x29,0x25,0x23,0x21,	//Z
} ;
char lmale[182]={
0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F,	// a
0x10,0x10,0x16,0x19,0x11,0x11,0x1E,	// b
0x00,0x00,0x0E,0x10,0x10,0x11,0x0E,	// c
0x01,0x01,0x0D,0x13,0x11,0x11,0x0F,	// d
0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E,	// e
0x06,0x09,0x08,0x1C,0x08,0x08,0x08,	// f
0x00,0x0F,0x11,0x11,0x0F,0x01,0x0E, // g
0x10,0x10,0x16,0x19,0x11,0x11,0x11, // h
0x04,0x00,0x04,0x04,0x04,0x04,0x0E, // i
0x02,0x00,0x06,0x02,0x02,0x12,0x0C,	// j
0x10,0x10,0x12,0x14,0x18,0x14,0x12,	// k
0x0C,0x04,0x04,0x04,0x04,0x04,0x0E, // l
0x00,0x00,0x1A,0x15,0x15,0x11,0x11, // m
0x00,0x00,0x16,0x19,0x11,0x11,0x11, // n
0x00,0x00,0x0E,0x11,0x11,0x11,0x0E,	// o
0x00,0x00,0x1E,0x11,0x1E,0x10,0x10,	// p
0x00,0x00,0x0D,0x13,0x0F,0x01,0x01,	// q
0x00,0x00,0x16,0x19,0x10,0x10,0x10,	// r
0x00,0x00,0x0E,0x10,0x0E,0x01,0x1E,	// s
0x08,0x08,0x1C,0x08,0x08,0x09,0x06,	// t
0x00,0x00,0x11,0x11,0x11,0x13,0x0D,	// u
0x00,0x00,0x11,0x11,0x11,0x0A,0x04,	// v
0x00,0x00,0x11,0x15,0x15,0x15,0x0A,	// w
0x00,0x00,0x11,0x0A,0x04,0x0A,0x11,	// x
0x00,0x00,0x11,0x11,0x0F,0x01,0x0E,	// y
0x00,0x00,0x1F,0x02,0x04,0x08,0x1F, // z
};
char cyfry[112]={
0,0x1E,0x31,0x29,0x25,0x23,0x1E,	// 0
0,0x22,0x21,0x3F,0x20,0x20,0x20,	// 1
0,0x32,0x29,0x29,0x29,0x29,0x26,	// 2
0,0x12,0x21,0x21,0x25,0x25,0x1A,	// 3
0,0x18,0x14,0x12,0x3F,0x10,0x10,	// 4
0,0x17,0x25,0x25,0x25,0x25,0x19,	// 5
0,0x1E,0x25,0x25,0x25,0x25,0x18,	// 6
0,0x01,0x01,0x31,0x09,0x05,0x03,	// 7
0,0x1A,0x25,0x25,0x25,0x25,0x1A,	// 8
0,0x06,0x29,0x29,0x29,0x29,0x1E,	// 9
0x00,0,0,0x24,0,0,0,	// :
0x00,0,0x40,0x32,0,0,0,
0x02,0x04,0x08,0x10,0x08,0x04,0x02,	// <
0x00,0x00,0x1F,0x00,0x1F,0x00,0x00,	// =
0x08,0x04,0x02,0x01,0x02,0x04,0x08,	// >
0x0E,0x11,0x01,0x02,0x04,0x00,0x04,	// ?
};


//HMI inputs
#define enc_sw PORTBbits.RB0
#define switch_port PORTBbits.RB1
#define enc_a PORTBbits.RB2
#define enc_b PORTBbits.RB3

unsigned char slide_sw_reg = 0;
unsigned char slide_sw_buff = 0;

unsigned char enc_sw_reg = 0;
unsigned char enc_sw_buff = 0;

unsigned int enc_reg = 0;
unsigned int enc_reg_tmp = 0;
unsigned int enc_buff = 0;





unsigned char slide_switches=0;
unsigned char enc_switches=0;
unsigned short enc_inputs=0;

signed char enc_value_temp[8]={0,0,0,0,0,0,0,0,};
signed char enc_value[8]={0,0,0,0,0,0,0,0,};
char pENC=0;


int chuj =0;
int chuj1 =0;

unsigned int val_tmp[8];
char val[8];



#define CPU_CLOCK          (80000000ul)
#define PBUS_CLOCK         (CPU_CLOCK/2)
#define UBRG(baud) (((PBUS_CLOCK)/4/(baud)-1))
#define UART1_BAUD           500000



void update_LCD(void);



char UART_TX[8];
char UART_RX[64];
char burnox=0;
int kutasss=0;

void send_spi(char data);
void L_pulse(void);
void clock_pulse(void);
void smr(void);
void enc_butt_init(void);
void lcd_init(void);
void draw_pixel(void);
void draw_v_line(unsigned char l);
void draw(void);
void get_raw_input(void);
void force_dma_uart(void);
void inicjalizacja(void);
void draw_h_line(unsigned char l);

void main(void){
    inicjalizacja();
    
    
   
    INTCONbits.MVEC = 1;
    asm volatile("ei");
    
    
    for(k=0;k<2048;k++)graphic_buffer[k]=0xAA;  

    while(1){
        

    chuj++;
            get_raw_input() ;
            if(chuj>100){chuj=0;
            
         for(k=0;k<8;k++) UART_TX[k]=enc_value[k];
        if(slide_sw_reg&0x01 && burnox==0)
        {
        burnox=1;
        force_dma_uart();
        }
            
        if(!slide_sw_reg&0x01)burnox=0;
            
            
            
          draw();
        }

    }

}




void __ISR(_DMA_0_VECTOR,IPL3AUTO)__DMA0Interrupt(void){
    DCH0CONbits.CHEN =0;
    DCH0INTCLR = 0x00FF00FF;
    DCH0INTbits.CHBCIE=1;
    IFS1bits.DMA0IF =0;
    
    while(SPI1STATbits.SPIBUSY);
    A0=0;     
    SPI1BUF = 0;
    SPI1BUF = 0x10;
    SPI1BUF = 0xB0|lcd_page;
    while(SPI1STATbits.SPIBUSY); 
    A0=1;
    DCH0SSA = KVA_TO_PA(&graphic_buffer[(lcd_flag.witch_lcd_page*1023)+(128*lcd_page)]);
    if(lcd_page){
    DCH0CONbits.CHEN = 1;
    DCH0ECONbits.CFORCE =1; 
    }
    if(++lcd_page > 7)lcd_page=0;
    
}

void __ISR(_DMA_1_VECTOR,IPL3AUTO)__DMA1Interrupt(void){

    IFS1CLR = _IFS1_DMA1IF_MASK; 

}

void dma_init(void){
    
    IPC10SET = 3 << _IPC10_DMA0IP_POSITION;        
    IFS1CLR = _IFS1_DMA0IF_MASK;                /* Make sure Interrupt Flag is clear before Enable */
    IEC1SET = _IEC1_DMA0IE_MASK;  
     
    DMACONbits.ON = 1;
    DCH0ECONbits.CHSIRQ = _SPI1_TX_IRQ;
    DCH0ECONbits.SIRQEN = 1;
    DCH0DSA=KVA_TO_PA(&SPI1BUF);
   
    DCH0SSIZ=128;
    DCH0DSIZ=1;
    DCH0CSIZ=1;
    
    DCH0INTCLR = 0x00FF00FF;
    DCH0INTbits.CHBCIE=1;
    
    IPC10SET = 3 << _IPC10_DMA1IP_POSITION;        
    IFS1CLR = _IFS1_DMA1IF_MASK;                /* Make sure Interrupt Flag is clear before Enable */
 //   IEC1SET = _IEC1_DMA1IE_MASK; 
    
  
    DCH1ECONbits.CHSIRQ = _UART1_TX_IRQ;
    DCH1ECONbits.SIRQEN = 1;
    DCH1DSA=KVA_TO_PA(&U1TXREG);
    DCH1SSA = KVA_TO_PA(&UART_TX[0]);
    DCH1SSIZ=8;
    DCH1DSIZ=1;
    DCH1CSIZ=1;
    
    DCH1INTCLR = 0x00FF00FF;
    DCH1INTbits.CHBCIE=1;
    
    DCH1CONbits.CHEN = 1;
    
    IPC10SET = 3 << _IPC10_DMA2IP_POSITION;        
    IFS1CLR = _IFS1_DMA2IF_MASK;                /* Make sure Interrupt Flag is clear before Enable */
//   IEC1SET = _IEC1_DMA2IE_MASK; 
    


}



void inicjalizacja(void){
    asm volatile("di");              
    asm volatile("ehb");              

    enc_butt_init();
    lcd_init();

    TRISAbits.TRISA0 = 0; //UART TX
    TRISAbits.TRISA2 = 1; //UART RX
    RPA0R=1; //RPA0 UART TX 
    U1RXR=0; //RPA2 UART RX

    U1MODE = 0;   
    U1BRG = UBRG(UART1_BAUD); 

    IFS1bits.U1TXIF = 0; 
    IEC1bits.U1TXIE = 0;   
    IPC8bits.U1IP = 2;
    IPC8bits.U1IS = 2;
    U1STAbits.UTXISEL = 0x01; 
    U1STAbits.UTXEN = 1; 
    U1STAbits.URXEN = 1; 
    U1MODEbits.ON = 1; 
    
    
    
    dma_init();   
}

void force_dma_uart(void){
DCH1SSA = KVA_TO_PA(&UART_TX[0]);
DCH1CONbits.CHEN = 1;
DCH1ECONbits.CFORCE =1;    
}
void get_raw_input(void)
{
            if(switch_port);
            else slide_sw_buff= slide_sw_buff | (0x01<<chuj1);
            
            if(enc_sw);
            else enc_sw_buff= enc_sw_buff | (0x01<<chuj1);
            
            if(enc_b)enc_buff= enc_buff | (0x01<<((chuj1*2)+1));
            else ;
            
            if(enc_a)enc_buff= enc_buff | (0x01<<(chuj1*2));
            else ;

            smr();
            chuj1++;
            if(chuj1>7)
    {
               chuj1=0;
               slide_sw_reg=slide_sw_buff;slide_sw_buff=0;
               enc_sw_reg=enc_sw_buff;enc_sw_buff=0;
               enc_reg=enc_buff;enc_buff=0;       
                  
       unsigned char with_enc_counter;        
       for(with_enc_counter=0;with_enc_counter<=7;with_enc_counter++)
        {
       val_tmp[with_enc_counter] = 0x3&(enc_reg>>(with_enc_counter*2));
	   if(val[with_enc_counter] != val_tmp[with_enc_counter])
            { 
		   if(val[with_enc_counter]==2 && val_tmp[with_enc_counter]==3)enc_value[with_enc_counter]++;
		   else if(val[with_enc_counter]==1 && val_tmp[with_enc_counter]==3)enc_value[with_enc_counter]--;
		   val[with_enc_counter] = val_tmp[with_enc_counter]; 
            } 
        }                
    }
            
}            

void write_char(char litera)
{
    unsigned char scx=0;
    unsigned int bufg=0;
    int yy=0;
    unsigned char offsetownikownik;
	if(litera >= '0' && litera < '@')
	{
	offsetownikownik = 7*(litera - '0');
    yy=y>>3;
    for(scx=0;scx<7;scx++){
    bufg=cyfry[offsetownikownik+scx];    
    bufg=bufg<<(y-(yy<<3));  
    graphic_buffer[(!lcd_flag.witch_lcd_page)*1023+scx+x+(128*yy)]|=bufg&0xFF; 
    graphic_buffer[(!lcd_flag.witch_lcd_page)*1023+scx+x+128+(128*yy)]|=bufg>>8;
    }
	}
/////////////////////////////////////////
	if(litera >= '@' && litera < 0x60){
	offsetownikownik = 7*((litera - '@')-1);
    yy=y>>3;
    for(scx=0;scx<7;scx++){
    bufg=lduze[offsetownikownik+scx];    
    bufg=bufg<<(y-(yy<<3));  
    graphic_buffer[(!lcd_flag.witch_lcd_page)*1023+scx+x+(128*yy)]|=bufg&0xFF; 
    graphic_buffer[(!lcd_flag.witch_lcd_page)*1023+scx+x+128+(128*yy)]|=bufg>>8;
    }
	}
////////////////////////////////////////
	if(litera >= 0x60 && litera < 0x7F)
	{
	offsetownikownik = 7*((litera - 0x60)-1);
    yy=y>>3;
    for(scx=0;scx<7;scx++){
    bufg=lmale[offsetownikownik+scx];    
    bufg=bufg<<(y-(yy<<3));  
    graphic_buffer[(!lcd_flag.witch_lcd_page)*1023+scx+x+(128*yy)]|=bufg&0xFF; 
    graphic_buffer[(!lcd_flag.witch_lcd_page)*1023+scx+x+128+(128*yy)]|=bufg>>8;
    }
	}
}


void draw(void){
    for(k=0;k<1024;k++)graphic_buffer[k+(!lcd_flag.witch_lcd_page)*1023]=0; 

    x=1;
    y=5;
    printf("HELLO");
    
    x=1;
    y=12;
    printf("HACK A DAY");
    
    x=1;
    y=19;
    printf("PRIZE 2018");
    
    x=10;
    y=35;
    printf("%d",enc_value[enc_value[1]]);
    
    x=100;
    y=54;
    printf("%d",enc_value[1]);
    
    
    x=100;
    y=35;
    printf("%d",burnox);
    
    
    
    x=1;
    y=30;
    draw_h_line(30);
    
    x=1;
    y=50;
    draw_h_line(30);
            
    
    x=1;
    y=30;
    draw_v_line(20);
    
    x=31;
    y=30;
    draw_v_line(20);
    
    x=62;
    y=32;
    printf("%d",UART_RX[0]);
 

                  
    lcd_flag.witch_lcd_page=!lcd_flag.witch_lcd_page;       
    
    update_LCD();
    
} 

void draw_h_line(unsigned char l){
    unsigned char scx=x;
    while(x!=(scx+l)){   
    draw_pixel();
    x++;
    }  
}
void draw_v_line(unsigned char l){
    unsigned char scy=y; 
    while(y!=(scy+l)){  
    draw_pixel();
    y++;     
    }  
    
}
void draw_pixel(void){
    unsigned char yy;
    yy=y>>3;
    graphic_buffer[(!lcd_flag.witch_lcd_page)*1023+x+(128*yy)]|=0x01<<(y-(yy<<3));
}
void lcd_init(void){
    TRISBbits.TRISB7 = 0; //CS1B
    TRISBbits.TRISB12 = 0; //A0
    TRISBbits.TRISB11 = 0; //RST
    TRISBbits.TRISB13 = 0; //SI
    ANSELBbits.ANSB13 = 0;
    TRISBbits.TRISB14 = 0; //SCL
    RST = 0;
    RPB7R=3;
    RPB13R=3;
    IPC7bits.SPI1IP = 3;
    IPC7bits.SPI1IS = 1;
    
    IEC1bits.SPI1EIE = 0;
    IEC1bits.SPI1TXIE = 0;
    IEC1bits.SPI1RXIE = 0;
    
    IFS1bits.SPI1EIF = 0;
    IFS1bits.SPI1TXIF = 0;
    IFS1bits.SPI1RXIF = 0;
    
    SPI1BRG=0;  
    SPI1STATbits.SPIROV = 0;
    SPI1CON =0; 
    SPI1CONbits.STXISEL = 0; 
    SPI1CONbits.MSTEN = 1; 
    SPI1CONbits.CKP = 1;
    SPI1CONbits.CKE = 0;
    SPI1CONbits.MSSEN = 1;
    SPI1CONbits.FRMEN = 1;
    SPI1CONbits.ENHBUF = 1;
    SPI1CONbits.FRMSYPW = 1;
    SPI1CONbits.FRMPOL = 0;
    SPI1CONbits.SPIFE=1;
    
    for(k=0;k<16;k++)SPI1BUF=0;    
    SPI1CONbits.ON = 1;
    RST = 1;
    
    for(k=0;k<1000;k++);
    A0=0;
    send_spi(0x40);
    send_spi(0xA1);
    send_spi(0xC0);    
    send_spi(0xA6);    
    send_spi(0xA2);   
    send_spi(0x2F);   
    send_spi(0xF8);  
    send_spi(0x00);    
    send_spi(0x27);   
    send_spi(0x81);  
    send_spi(0x15);    
    send_spi(0xAC);  
    send_spi(0x00);  
    send_spi(0xAF); 
    A0=1; 
}
void enc_butt_init(void){
    //ports are output
    TRISBbits.TRISB10 = 0; //latch STCP
    TRISBbits.TRISB8 = 0; //data DS
    TRISBbits.TRISB9 = 0; //clock SHCP
    
    //ports are inputs
    TRISBbits.TRISB0 = 1; //button
    TRISBbits.TRISB1 = 1; //enc_a 
    TRISBbits.TRISB2 = 1; //enc_b
    TRISBbits.TRISB3 = 1; //switch
    //digital input 
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    //internal pull-up on
    CNPUBbits.CNPUB0 =1;
    CNPUBbits.CNPUB1 =1;
    CNPUBbits.CNPUB2 =1;
    CNPUBbits.CNPUB3 =1;
    
    DATAa = 0;
    CLKa = 0;
    La = 0;
    int ax;
    
    
    
    
    DATAa = 1;
    clock_pulse();
    clock_pulse();    
    clock_pulse();
    clock_pulse();
    clock_pulse();
    clock_pulse();
    clock_pulse();
    DATAa = 0;
    clock_pulse();
    L_pulse();
 
    smr();
    smr();
    smr();
    smr();
    smr();
    smr();
    smr();

}
char bgf=0;
void smr(void){
if(bgf==8){DATAa=0;  bgf=0;}
else DATAa=1;
    clock_pulse();
    L_pulse();
    bgf++;
}
void clock_pulse(){
    CLKa=1;
    CLKa=0;
}
void L_pulse(){
    La=1;
    La=0;
}
void _mon_putc(char c)
{
write_char(c);
x+=8;
}


void send_spi(char data){
    SPI1BUF=data;
    while(SPI1STATbits.SPIBUSY); 
}    
void update_LCD(void){
    DCH0CONbits.CHEN = 1;
    DCH0ECONbits.CFORCE =1; 
}


