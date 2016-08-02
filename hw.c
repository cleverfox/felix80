#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
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

    //ONV
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
    gpio_set(GPIOC, GPIO1);

    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO7);
    gpio_clear(GPIOC, GPIO7);
}

void usart_setup(void) {
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


