#ifndef CONF_H

#define	CONF_H



#include <xc.h> // include processor files - each processor file is guarded.  


#define FCY 40000000ULL
#define UBRG(baud)  ((FCY/baud)/16)-1
#define UART1_BAUD           9600 


int ExampleLocalFunction(int param1);
int filtr(int we);
void config(void);


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

    
    
    
    
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

