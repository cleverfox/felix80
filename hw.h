#ifndef HW_H_INCLUDED
#define HW_H_INCLUDED

void init_hw(void);
void usart_setup(void);
void SBR(void);
void ONV(void);
void MOTOR(uint8_t phase);
uint8_t IS_CUTED(void);
void redled_toggle(void);
void redled_off(void);
void redled_on(void);
void grnled_off(void);
void grnled_on(void);
void beep_off(void);
void beep_on(void);
void usart3_setup(void);
void usart2_setup(void);
void usart1_setup(void);


#endif

