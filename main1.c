#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/systick.h>
#include "tools.h"
#include "hw.h"
#include <atom.h>
#include <atomport.h>
#include <atomqueue.h>
#include <atomtimer.h>

void _fault(int, int, const char*);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)

static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_24mhz();
//    rcc_clock_setup_in_hsi_out_24mhz();

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_SPI1);
}

static void spi_setup(void) {
    /* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7 */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4 | GPIO5 | GPIO7 );

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
            GPIO6);

    /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
    spi_reset(SPI1);

    /* Set up SPI in Master mode with:
     * Clock baud rate: 1/64 of peripheral clock frequency
     * Clock polarity: Idle High
     * Clock phase: Data valid on 2nd clock pulse
     * Data frame format: 8-bit
     * Frame format: MSB First
     */
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
            SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    /*
     * Set NSS management to software.
     *
     * Note:
     * Setting nss high is very important, even if we are controlling the GPIO
     * ourselves this bit needs to be at least set to 1, otherwise the spi
     * peripheral will not send any data out.
     */
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    /* Enable SPI1 periph. */
    spi_enable(SPI1);

    //Manual drive A4 
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

    gpio_set(GPIOA, GPIO4);
}

void exti0_isr(void)
{
	exti_reset_request(EXTI0);
        /*
        cout('i');
        int status = nrf24_getStatus();
        if((status&MASK_RX_DR)){
            cout('D'); //data ready
            cout((status>>1 & 0x07) + '0');
        }
        if((status&MASK_TX_DS)){
            cout('S'); //sent ok
        }
        if((status&MASK_MAX_RT)){ 
            cout('E'); //send error
        }
        if(status&1){ //tx full
            cout('F'); //full
        }
        _write(0,"-\r\n",3);
        */
}

/*
static void gpio_setup(void) {
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
    gpio_set(GPIOB, GPIO9);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
    gpio_clear(GPIOB, GPIO1);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
    gpio_set(GPIOB, GPIO0);

    nvic_enable_irq(NVIC_EXTI0_IRQ);
    exti_select_source(EXTI0, GPIOB);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI0);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, GPIO11);
    gpio_clear(GPIOB, GPIO11);

}
*/



#if 0
static void tim_setup(void) {
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral. */
	timer_reset(TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/* Reset prescaler value. */
	timer_set_prescaler(TIM2, 400);

	/* Enable preload. */
	timer_disable_preload(TIM2);

	/* Continous mode. */
	timer_continuous_mode(TIM2);

	/* Period (36kHz). */
	timer_set_period(TIM2, 60000);

	/* Disable outputs. */
	timer_disable_oc_output(TIM2, TIM_OC1);
	timer_disable_oc_output(TIM2, TIM_OC2);
	timer_disable_oc_output(TIM2, TIM_OC3);
	timer_disable_oc_output(TIM2, TIM_OC4);

	/* -- OC1 configuration -- */

	/* Configure global mode of line 1. */
	timer_disable_oc_clear(TIM2, TIM_OC1);
	timer_disable_oc_preload(TIM2, TIM_OC1);
	timer_set_oc_slow_mode(TIM2, TIM_OC1);
	timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);

	/* Set the capture compare value for OC1. */
	timer_set_oc_value(TIM2, TIM_OC1, 1000);

	/* ---- */

	/* ARR reload enable. */
	timer_disable_preload(TIM2);

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable commutation interrupt. */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {

        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM2, TIM_SR_CC1IF);

        /*
           compare_time = timer_get_counter(TIM2);
           frequency = frequency_sequence[frequency_sel++];
           new_time = compare_time + frequency;

           timer_set_oc_value(TIM2, TIM_OC1, new_time);
           if (frequency_sel == 18)
           frequency_sel = 0;
           gpio_toggle(GPIOC, GPIO12);
           */
        //icout(recvd);
//        _write(0,"\4\xFB\0\0\0",5);
        gpio_toggle(GPIOB, GPIO9);
        //recvd=0;
    }
}
#endif

#define STACK_SIZE      1024
#define THREAD_PRIO     42

static uint8_t thread_stacks[4][STACK_SIZE];

static void uart1_thread(uint32_t data);
static ATOM_TCB uart1_thread_tcb;
#define UART_QLEN 64
static uint8_t uart1_rx_storage[UART_QLEN];
static uint8_t uart1_tx_storage[UART_QLEN];
static ATOM_QUEUE uart1_rx;
static ATOM_QUEUE uart1_tx;

static void printer_thread(uint32_t data);
static ATOM_TCB printer_thread_tcb;

static void logic_thread(uint32_t data);
static ATOM_TCB logic_thread_tcb;

void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    gpio_set(GPIOC, GPIO4); //green led off
    while(1){
        int l=20;
        while(l--){
            gpio_toggle(GPIOB, GPIO5); //beeper
            volatile int x=10000;
            while(x--){};
        }
        gpio_toggle(GPIOC, GPIO5); //red led
    }
};

void usart1_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();

    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART1);
        atomQueuePut(&uart1_rx,0, (uint8_t*) &data);
        /*usart_send(USART1, data);
        if(data=='f'){
            fault(0);
        }
        if(data=='z'){
            gpio_toggle(GPIOB, GPIO5);
        }
        */
        /* Enable transmit interrupt so it sends back the data. */
        //        USART_CR1(USART2) |= USART_CR1_TXEIE;
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

        uint8_t status = atomQueueGet(&uart1_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send(USART1, data);
        }else{
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}

#if 0
        if(s.cut){
            SBR();
            Uart1WriteStr("C_");
            int x=0;
            int step=0;
            while(1){
                ONV();
                MOTOR(x);
                x++;
                myDelay(1500);
                if(!IS_CUTED()){
                    step=1;
                }else{
                    if(step>0){
                        step++;
                        if(step>100)
                            break;
                    }
                }
            }
            Uart1WriteStr("C-");
            s.cut=0;
        }
        if(s.linefeed){
            SBR();
            Uart1WriteStr("F_");
            int x=100;
            while(x--){
                ONV();
                MOTOR(x);
                myDelay(1500);
            }
            Uart1WriteStr("F-");
            s.linefeed=0;
        }
        myDelay(800000);
        Uart1WriteStr(".");
#endif

void cdelay(uint32_t);

void cdelay(uint32_t t) {
    volatile uint32_t t1=systick_get_value();
    uint32_t t2=t1-t;
    if(t1 > t){
        t2=0xffffffff-(t-t1);
        while(true){
            volatile uint32_t cc=systick_get_value();
            if(cc<=t2 && cc>t1)
                break;
        }
    }else{
        while(true){
            volatile uint32_t cc=systick_get_value();
            if(cc<t2) break;
        }
    }
}

int main(void) {
    cm_mask_interrupts(true);

    clock_setup();

    systick_set_frequency(SYSTEM_TICKS_PER_SEC, 24000000);
    systick_interrupt_enable();
    systick_counter_enable();

    nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

    //gpio_setup();
    usart_setup();
    //tim_setup();
    spi_setup();
    init_hw();


    int8_t status;
    status = atomOSInit(&thread_stacks[0][0], STACK_SIZE, FALSE);

    if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK) 
        fault(2);
    if (atomQueueCreate (&uart1_tx, uart1_tx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK) 
        fault(3);

    if (status != ATOM_OK) 
        fault(1);

        _write(0,"atomthreads ready\r\n",19);
        atomThreadCreate(&logic_thread_tcb, THREAD_PRIO, logic_thread, 0,
                &thread_stacks[1][0], STACK_SIZE, TRUE);

        atomThreadCreate(&uart1_thread_tcb, THREAD_PRIO, uart1_thread, 0,
                &thread_stacks[2][0], STACK_SIZE, TRUE);

        atomThreadCreate(&printer_thread_tcb, 50, printer_thread, 0,
                &thread_stacks[3][0], STACK_SIZE, TRUE);

        gpio_clear(GPIOC, GPIO4);
        atomOSStart();

    while(1){};
    return 0;
}

static void logic_thread(uint32_t args __maybe_unused) {
    while(1){
        uint8_t data;
        uint8_t status = atomQueueGet(&uart1_rx, SYSTEM_TICKS_PER_SEC, &data);
        if(status == ATOM_OK){
            atomQueuePut(&uart1_tx,0, (uint8_t*) &data);
        }else{
            _write(0,"-=-=-=\r\n",8);
        }
        //atomTimerDelay(SYSTEM_TICKS_PER_SEC*4);
    }
}

static void uart1_thread(uint32_t data __maybe_unused) {
    while(1){
        uint8_t msg;
        uint8_t status = atomQueueGet(&uart1_tx, 0, &msg);
        if(status == ATOM_OK){
            usart_send_blocking(USART1, msg);
        }
    }
}

static void printer_thread(uint32_t data __maybe_unused) {
    while(1){
        gpio_clear(GPIOC, GPIO5);
        cdelay(100);
        gpio_set(GPIOC, GPIO5);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC >> 4);
    }
}

int _write(__unused int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        atomQueuePut(&uart1_tx,0, (uint8_t*) &ptr[i]);
//        usart_send_blocking(USART1, ptr[i]);
    }
    return i;
}


inline uint8_t spi_transfer(uint8_t tx){
        spi_send(SPI1, tx);
        return  spi_read(SPI1);
}


