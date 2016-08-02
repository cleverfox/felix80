#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/systick.h>
#include "tools.h"
#include "hw.h"
#include <atom.h>
//#include <atomport.h>
#include <atomqueue.h>
#include <atomtimer.h>

void _fault(int, int, const char*);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)
void hard_fault_handler() {
    fault(255);
}

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
    rcc_periph_clock_enable(RCC_SPI2);
    rcc_periph_clock_enable(RCC_DMA1);
}

static void spi_setup(void) {
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO12 | GPIO13 | GPIO15 );

    /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
    spi_reset(SPI2);

    SPI2_I2SCFGR = 0; //disable i2s
    /* Set up SPI in Master mode with:
     * Clock baud rate: 1/64 of peripheral clock frequency
     * Clock polarity: Idle High
     * Clock phase: Data valid on 2nd clock pulse
     * Data frame format: 8-bit
     * Frame format: MSB First
     */
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
            SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    /*
     * Set NSS management to software.
     *
     * Note:
     * Setting nss high is very important, even if we are controlling the GPIO
     * ourselves this bit needs to be at least set to 1, otherwise the spi
     * peripheral will not send any data out.
     */
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);

    spi_enable(SPI2);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

    gpio_set(GPIOB, GPIO12);
}

void exti0_isr(void)
{
	exti_reset_request(EXTI0);
}

static void tim2_setup(void) {
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
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);

	/* Reset prescaler value. */
	timer_set_prescaler(TIM2, 24);

	/* Enable preload. */
	timer_disable_preload(TIM2);

	/* Continous mode. */
	timer_continuous_mode(TIM2);

	/* Period (36kHz). */
	timer_set_period(TIM2, 65535);

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
//	timer_set_oc_value(TIM2, TIM_OC1, 1000);

	/* ---- */

	/* ARR reload enable. */
	timer_disable_preload(TIM2);

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable commutation interrupt. */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}


static ATOM_QUEUE uart1_rx;
static ATOM_QUEUE uart1_tx;
void tim2_isr(void) {
//    atomIntEnter();
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
//        char data='T';
//        atomQueuePut(&uart1_tx,0, (uint8_t*) &data);
    }
//    atomIntExit(0);
}

void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    gpio_set(GPIOC, GPIO4); //green led off
    while(1){
        int l=4;
        while(l--){
            gpio_set(GPIOB, GPIO5); //beeper
            volatile int x=300;
            while(x--){};
            gpio_clear(GPIOB, GPIO5); //beeper
            x=50000;
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

#if 1
void cdelay(uint32_t);

void cdelay(uint32_t t) {
    volatile uint32_t t1=timer_get_counter(TIM2);
    volatile uint32_t t2=t1-t;
    if(t1 < t){
        t2=0xffff-(t-t1);
        while(true){
            volatile uint32_t cc=timer_get_counter(TIM2);
            if(cc<=t2 && cc>t1)
                break;
        }
    }else{
        while(true){
            volatile uint32_t cc=timer_get_counter(TIM2);
            if(cc<t2) break;
        }
    }
}
#endif

inline void idelay(uint32_t);
inline void idelay(uint32_t t){
    timer_set_counter(TIM2, t);
    TIM_DIER(TIM2) |= TIM_DIER_CC1IE;
    //timer_enable_irq(TIM2, TIM_DIER_CC1IE);
    __asm("wfi");
}

static uint8_t idle_stack[256];
#define STACK_SIZE      1024
#define THREAD_PRIO     42
static uint8_t thread_stacks[3][STACK_SIZE];

static void uart1_thread(uint32_t data);
static ATOM_TCB uart1_thread_tcb;
#define UART_QLEN 64
static uint8_t uart1_rx_storage[UART_QLEN];
static uint8_t uart1_tx_storage[UART_QLEN];

static void printer_thread(uint32_t data);
static ATOM_TCB printer_thread_tcb;

static void logic_thread(uint32_t data);
static ATOM_TCB logic_thread_tcb;


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
    tim2_setup();

    nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);

    /*
    usart_send_blocking(USART1, 'h');
    usart_send_blocking(USART1, 'i');
    */


    int8_t status;
    status = atomOSInit(idle_stack, 256, FALSE);

    if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK) 
        fault(2);
    if (atomQueueCreate (&uart1_tx, uart1_tx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK) 
        fault(3);

    if (status != ATOM_OK) fault(1);

    _write(0,"atomthreads ready\r\n",19);

    atomThreadCreate(&logic_thread_tcb, THREAD_PRIO, logic_thread, 0,
            thread_stacks[0], STACK_SIZE, TRUE);

    atomThreadCreate(&uart1_thread_tcb, THREAD_PRIO, uart1_thread, 0,
            thread_stacks[1], STACK_SIZE, TRUE);

    atomThreadCreate(&printer_thread_tcb, 50, printer_thread, 0,
            thread_stacks[2], STACK_SIZE, TRUE);

    gpio_clear(GPIOC, GPIO4);
    atomOSStart();

    while(1){};
    return 0;
}

static void logic_thread(uint32_t args __maybe_unused) {
    while(1){
        //uint8_t data;
        /*
        uint8_t status = atomQueueGet(&uart1_rx, SYSTEM_TICKS_PER_SEC, &data);
        if(status == ATOM_OK){
    	    data++;
            atomQueuePut(&uart1_tx,0, (uint8_t*) &data);
        }else{
        }
        */
        _write(0,"-=-=-=\r\n",8);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC*4);
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

void dma1_channel5_isr(void) {
    atomIntEnter();
    if ((DMA1_ISR &DMA_ISR_TCIF5) != 0) {
        DMA1_IFCR |= DMA_IFCR_CTCIF5;
    }
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
    spi_disable_tx_dma(SPI2);
    dma_disable_channel(DMA1, DMA_CHANNEL5);

    char data='D';
    atomQueuePut(&uart1_tx,0, (uint8_t*) &data);
    atomIntExit(0);
}

static void printer_thread(uint32_t arg __maybe_unused) {
    CRITICAL_STORE;
    uint8_t printbuf[72];
    uint8_t data;
    while(1){
        uint8_t status = atomQueueGet(&uart1_rx, SYSTEM_TICKS_PER_SEC, &data);
        if(status == ATOM_OK){
            if(data=='w'){
                _write(0,"w\r\n",3);
                int x=72;
                while(x--){
                    printbuf[x]=0xf0;
                }
                dma_channel_reset(DMA1, DMA_CHANNEL5);

		dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&SPI2_DR);
		dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)printbuf);
		dma_set_number_of_data(DMA1, DMA_CHANNEL5, 72);
		dma_set_read_from_memory(DMA1, DMA_CHANNEL5);
		dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);

                /*
		dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_16BIT);
		dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_16BIT);
                */

		dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
		dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);

		dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_HIGH);

		dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
		dma_enable_channel(DMA1, DMA_CHANNEL5);
                spi_enable_tx_dma(SPI2);
            }
            if(data=='q'){
                _write(0,"q\r\n",3);
                int x=36;
                gpio_clear(GPIOB, GPIO12);
                while(x--){
                    spi_xfer(SPI2, 0x0000);
                }
                gpio_set(GPIOB, GPIO12);
            }
            if(data=='f'){
                _write(0,"feed\r\n",6);
                CRITICAL_START();
                SBR();
                ONV();
                gpio_set(GPIOC, GPIO0);
                gpio_set(GPIOC, GPIO1);
                int x=200;
                while(x--){
                    MOTOR(x);
                    gpio_clear(GPIOC, GPIO5);
                    cdelay(1000);
                    gpio_set(GPIOC, GPIO5);
                }
                CRITICAL_END();
            }
            if(data=='p'){
                _write(0,"prin\r\n",6);
                CRITICAL_START();
                SBR();
                int x=200;
                while(x--){
                    if(x%10==0){
                        ONV();
                    }
                    MOTOR(x);
                    gpio_clear(GPIOC, GPIO0);
                    cdelay(500);
                    gpio_set(GPIOC, GPIO0);
                    gpio_clear(GPIOC, GPIO1);
                    cdelay(500);
                    gpio_set(GPIOC, GPIO1);
                }
                CRITICAL_END();
            }
            atomQueuePut(&uart1_tx,0, (uint8_t*) &data);
        }
        //atomTimerDelay(SYSTEM_TICKS_PER_SEC >> 4);
    }
}

int _write(__unused int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        atomQueuePut(&uart1_tx,0, (uint8_t*) &ptr[i]);
//        usart_send_blocking(USART1, ptr[i]);
    }
    //USART_CR1(USART1) |= USART_CR1_TXEIE;
    return i;
}


