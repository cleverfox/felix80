#ifndef HW_H_INCLUDED
#define HW_H_INCLUDED

void init_hw(void);
void usart_setup(void);
void SBR(void);
void ONV(void);
void MOTOR(uint8_t phase);
uint8_t IS_CUTED(void);

#endif

