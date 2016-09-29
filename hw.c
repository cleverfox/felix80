#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include "hw.h"


void init_hw(void){
    //leds
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4|GPIO5);
    gpio_set(GPIOC, GPIO4|GPIO5);

    //Motor phases
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0|GPIO1);
    gpio_set(GPIOB, GPIO0|GPIO1);

    //SBR
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set(GPIOB, GPIO8);

    //SM
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
    gpio_clear(GPIOB, GPIO9);

    //ZP (beeper)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
    gpio_clear(GPIOB, GPIO5);

    //ST1
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
    gpio_clear(GPIOC, GPIO0);

    //ST2
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
    gpio_clear(GPIOC, GPIO1);
    
    //PRIN
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO14);
    gpio_clear(GPIOB, GPIO14);

    //ONV
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
    gpio_set(GPIOC, GPIO2);

    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO7);
    gpio_clear(GPIOC, GPIO7);

    //KN1
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO12);
    gpio_set(GPIOA, GPIO12);
    exti_set_trigger (EXTI12, EXTI_TRIGGER_FALLING);
    exti_select_source (EXTI12, GPIOA);
    exti_enable_request (EXTI12);
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
}

void usart3_setup(void) {
    nvic_enable_irq(NVIC_USART3_IRQ);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

    gpio_set_mode(GPIO3, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_stopbits(USART3, USART_STOPBITS_1);

    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    USART_CR1(USART3) |= USART_CR1_RXNEIE;
    usart_enable(USART3);
}

void usart2_setup(void) {
    nvic_enable_irq(NVIC_USART2_IRQ);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

    /* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_stopbits(USART2, USART_STOPBITS_1);

    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2, USART_MODE_TX);

    /* Enable USART1 Receive interrupt. */
//    USART_CR1(USART2) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART2);
}

void usart1_setup(void) {
    /* Enable the USART1 interrupt. */
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    /* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);

    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    /* Enable USART1 Receive interrupt. */
    USART_CR1(USART1) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART1);
}

void usart_setup(void) {
    usart1_setup();
    usart2_setup();
    //usart3_setup();
}


void SBR(void) { 
    gpio_clear(GPIOB, GPIO8);
    gpio_set(GPIOB, GPIO8);
};
void ONV(void) { 
    gpio_clear(GPIOC, GPIO2);
    gpio_set(GPIOC, GPIO2);
};
void MOTOR(uint8_t phase) {
    uint8_t ph=3-(phase&3); //countdown for line feed, countup for cut
    switch(ph){
        case 0:
            gpio_set  (GPIOB, GPIO0);
            gpio_set  (GPIOB, GPIO1);
            break;
        case 1:
            gpio_set  (GPIOB, GPIO0);
            gpio_clear(GPIOB, GPIO1);
            break;
        case 2:
            gpio_clear(GPIOB, GPIO0);
            gpio_clear(GPIOB, GPIO1);
            break;
        case 3:
            gpio_clear(GPIOB, GPIO0);
            gpio_set  (GPIOB, GPIO1);
            break;
    };

};
uint8_t IS_CUTED(void) {
    return gpio_port_read(GPIOC) && GPIOC;
};


inline void redled_toggle(void){
    gpio_toggle(GPIOC, GPIO5); //red led toggle
}
inline void redled_off(void){
    gpio_set(GPIOC, GPIO5); //red led off
}
inline void redled_on(void){
    gpio_clear(GPIOC, GPIO5); //red led on
}
inline void grnled_off(void){
    gpio_set(GPIOC, GPIO4); //green led off
}
inline void grnled_on(void){
    gpio_clear(GPIOC, GPIO4); //green led on
}

inline void beep_off(void){
    gpio_clear(GPIOB, GPIO5); //beeper
}
inline void beep_on(void){
    gpio_set(GPIOB, GPIO5); //beeper
}

