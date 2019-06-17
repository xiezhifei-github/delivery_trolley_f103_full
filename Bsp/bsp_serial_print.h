#ifndef bsp_serial_print_h
#define bsp_serial_print_h

#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
/*#ifdef __GNUC__
  // With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
  //   set to 'Yes') calls __io_putchar() 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif // __GNUC__ */
int fputc(int c, FILE * f);
int fgetc(FILE * F);

#endif
