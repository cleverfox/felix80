#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/systick.h>
#include "tools.h"
#include "sleep.h"
#include "iic.h"
#include "hw.h"
#include <atom.h>
#include <atomsem.h>
#include <atomqueue.h>
#include <atomtimer.h>
#include <string.h>
#include "chargen.h"
#include "verdana8.h"
#include "tahoma8.h"
#include "lucida10.h"
#include "Terminus_28pt.h"


void _fault(int, int, const char*);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)
/*
void hard_fault_handler() {
    fault(255);
}
*/

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
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_I2C2);
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
            SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

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

void exti0_isr(void) {
    exti_reset_request(EXTI0);
}

void i2c2_ev_isr(void){
    usart_send_blocking(USART1, 'V');
}
void i2c2_er_isr(void){
    usart_send_blocking(USART1, 'R');
}

static ATOM_QUEUE str2print;
//static ATOM_QUEUE lin2print;

static ATOM_QUEUE uart1_rx;
static ATOM_QUEUE uart1_tx;
void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    grnled_off();
    while(1){
        int l=4;
        while(l--){
            beep_on();
            volatile int x=300;
            while(x--){};
            beep_off();
            x=50000;
            while(x--){};
        }
        redled_toggle();
    }
};

void usart2_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();

    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART2);
        atomQueuePut(&uart1_rx,0, (uint8_t*) &data);
    }

    atomIntExit(0);
}

void usart3_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();

    if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART3);
        atomQueuePut(&uart1_rx,0, (uint8_t*) &data);
    }

    atomIntExit(0);
}


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
            usart_send(USART2, data);
        }else{
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}

static void adc_setup(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO7); //TERM thermistor
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO6); //VPL +24v when ONV
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO5); //VIN
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4); //X5 SNPE

    /* Make sure the ADC doesn't run during config. */
    //adc_power_off(ADC1);

    /* We configure everything for one single conversion. */
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

    adc_power_on(ADC1);

    /* Wait for ADC starting up. */
    int i;
    for (i = 0; i < 800000; i++) /* Wait a bit. */
        __asm__("nop");

    adc_reset_calibration(ADC1);
    adc_calibration(ADC1);
}

static uint8_t idle_stack[256];
#define STACK_SIZE      1024
static uint8_t thread_stacks[3][STACK_SIZE];

#define UART_QLEN 64
static uint8_t uart1_rx_storage[UART_QLEN];
static uint8_t uart1_tx_storage[UART_QLEN];

#define LINQLEN 2
#define STRQLEN 2
struct print_str {
    uint8_t repeat_lines;
    uint8_t action;
    char str[96];
};
static struct print_str str2pr_storage[LINQLEN];

struct print_lin {
    uint8_t repeat_lines;
    uint8_t printhead[72];
};
//static struct print_lin lin2pr_storage[LINQLEN];

#define CG_STACK_SIZE 1024
/*
static void chargen_thread(uint32_t data);
static uint8_t chagen_stack[CG_STACK_SIZE];
static ATOM_TCB chargen_thread_tcb;
*/
static ATOM_SEM dma_busy;

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

    usart_setup();
    spi_setup();
    init_hw();
    tim2_setup();
    adc_setup();
    /*
    i2c1_setup();
    i2c2_setup();
    */

    nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);


	rtc_auto_awake(RCC_LSE, 0x7fff); //run RTC
#if 0 //reset RTC
	rtc_awake_from_off(LSE);
	rtc_set_prescale_val(0x7fff);
#endif
    /*
    usart_send_blocking(USART1, 'h');
    usart_send_blocking(USART1, 'i');
    */

    if(atomOSInit(idle_stack, 256, FALSE) != ATOM_OK) 
        fault(1);

    if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK) 
        fault(2);
    if (atomQueueCreate (&uart1_tx, uart1_tx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK) 
        fault(2);
    if (atomQueueCreate (&str2print, (void*)str2pr_storage, sizeof(struct print_str), STRQLEN) != ATOM_OK) 
        fault(3);
    if (atomSemCreate (&dma_busy, 0) != ATOM_OK) 
        fault(3);

    _write(0,"atomthreads ready\r\n",19);

    
    //high priority printer thread
    atomThreadCreate(&printer_thread_tcb, 5, printer_thread, 0,
            thread_stacks[1], STACK_SIZE, TRUE);
    
    //lower priority logic thread
    atomThreadCreate(&logic_thread_tcb, 30, logic_thread, 0,
            thread_stacks[0], STACK_SIZE, TRUE);

    grnled_on();
    atomOSStart();

    fault(254);
    while(1){};
    return 0;
}

static void print(char* p, uint8_t len, uint8_t r, uint8_t a) {
    struct print_str pr={r,a,{0}};
    memcpy(pr.str,p,len);
    atomQueuePut(&str2print, 0, (void*)&pr);
};
static void logic_thread(uint32_t args __maybe_unused) {
    uint8_t ch;
#define SLEN 40
    struct print_str s2pr;
    s2pr.repeat_lines=1;
    uint32_t sptr=0;
    while(1){
        uint8_t status = atomQueueGet(&uart1_rx, SYSTEM_TICKS_PER_SEC, (void*)&ch);
        uint32_t rtc=rtc_get_counter_val();
        _write(0,"-=-=[",5);
        incout(rtc);
        if(status == ATOM_OK){
            uint8_t done=0;
            if(ch=='\r' || ch=='\n')
                done=1;
            else
                s2pr.str[sptr++]=ch;

            if(!done && SLEN<=sptr)
                done=1;
            if(done){
                if(s2pr.str[0]=='`'){
                    _write(0,"cp866 test\r\n",12);
                    for(status=0;status<=48;status++)
                        s2pr.str[status]=status+128;
                    s2pr.str[status]=0;
                }
                if(s2pr.str[0]>='1' && s2pr.str[0]<='9'){
                    _write(0,"Set repeat\r\n",12);
                    s2pr.repeat_lines=s2pr.str[0]-'0';
                }
                atomQueuePut(&str2print, 0, (uint8_t*) &s2pr);
                sptr=0;
            }
            //_write(0,&ch,1);
        }
        _write(0,"]=-=-\r\n",7);
        //     atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    }
}

/*
static void chargen_thread(uint32_t args __maybe_unused) {
    struct print_str instr;
    struct print_lin outlin;

    //(atomQueueCreate (&lin2print, lin2pr_storage, sizeof(struct print_lin), LINQLEN) != ATOM_OK) 
    //(atomQueueCreate (&str2print, str2pr_storage, sizeof(struct print_str), STRQLEN) != ATOM_OK) 
    while(1){
        uint8_t status = atomQueueGet(&str2print, 0, &instr);
        if(status == ATOM_OK){
        }
    }
}
*/

void dma1_channel5_isr(void) { //SPI transfer to head done
    atomIntEnter();
    if ((DMA1_ISR &DMA_ISR_TCIF5) != 0) {
        DMA1_IFCR |= DMA_IFCR_CTCIF5;
    }
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
    spi_disable_tx_dma(SPI2);
    dma_disable_channel(DMA1, DMA_CHANNEL5);

    atomSemPut (&dma_busy);
    atomIntExit(0);
}

static void printer_thread(uint32_t arg __maybe_unused) {
    CRITICAL_STORE;
    uint8_t printbuf[72];
    struct print_str instr;

    const FONT_INFO *fonts[]={
        &Terminus_28ptFontInfo,
        &verdana_8ptFontInfo,
        &tahoma_8ptFontInfo,
        &lucidaConsole_10ptFontInfo,
    };
    while(1){
        uint8_t status = atomQueueGet(&str2print, 0, (void*)&instr);
        uint32_t x=1000;
        if(status == ATOM_OK){
            _write(0,instr.str,strlen(instr.str));
            uint8_t fc[FCLEN];
            memset(fc,0,FCLEN);
            uint8_t bin[72];
            uint8_t h=1;
            for(int line=0;line<h;line++){
                int repeatl=instr.repeat_lines;
                x--;
                memset(bin,0,72);
                __unused int bytes=render_line(line, instr.str, sizeof(instr.str), fonts, bin, 72, fc, NULL, &h);
                /*
                   dma_channel_reset(DMA1, DMA_CHANNEL5);
                   dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&SPI2_DR);
                   dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)printbuf);
                   dma_set_number_of_data(DMA1, DMA_CHANNEL5, 72);
                   dma_set_read_from_memory(DMA1, DMA_CHANNEL5);
                   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);

                   dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
                   dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);

                   dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_HIGH);

                   dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
                   dma_enable_channel(DMA1, DMA_CHANNEL5);
                   spi_enable_tx_dma(SPI2);
                   */

                while(repeatl--){
                    icout(repeatl);

                    int b=0;
                    while(b<72){
                        uint16_t send=bin[b];
                        spi_xfer(SPI2, send);
                        b+=1;
                    }


                    ONV();
                    MOTOR(x);
                    cdelay(500);

                    //atomSemGet (&dma_busy, 0);

                    CRITICAL_START();
                    //latch printhead register
                    gpio_clear(GPIOB, GPIO12);
                    gpio_set(GPIOB, GPIO12);

                    gpio_set(GPIOC, GPIO0);
                    gpio_set(GPIOB, GPIO14);
                    cdelay(500);
                    gpio_clear(GPIOB, GPIO14);
                    gpio_clear(GPIOC, GPIO0);

                    gpio_set(GPIOC, GPIO1);
                    gpio_set(GPIOB, GPIO14);
                    cdelay(500);
                    gpio_clear(GPIOB, GPIO14);
                    gpio_clear(GPIOC, GPIO1);

                    CRITICAL_END();
                }
            }
        }
        //atomTimerDelay(SYSTEM_TICKS_PER_SEC >> 4);
    }
}

/*
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
            }else
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
                    gpio_set(GPIOC, GPIO0);
                    gpio_set(GPIOB, GPIO14);
                    cdelay(1000);
                    gpio_clear(GPIOB, GPIO14);
                    gpio_clear(GPIOC, GPIO0);
                    gpio_set(GPIOC, GPIO1);
                    gpio_set(GPIOB, GPIO14);
                    cdelay(1000);
                    gpio_clear(GPIOB, GPIO14);
                    gpio_clear(GPIOC, GPIO1);
                }
                CRITICAL_END();
            }else
            if(data=='c' && 0){
                SBR();
                int x=0;
                int step=0;
                while(1){
                    ONV();
                    MOTOR(x);
                    x++;
                    cdelay(1500);
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
            }else{
                _write(0, (char*)&data, 1);
            }
            */
int _write(__unused int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        atomQueuePut(&uart1_tx,0, (uint8_t*) &ptr[i]);
    }
    USART_CR1(USART1) |= USART_CR1_TXEIE;
    return i;
}


