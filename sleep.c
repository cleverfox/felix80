#include "sleep.h"
#include <atom.h>
#include <atomsem.h>

extern void _fault(int, int, const char*);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)

static ATOM_SEM ibusy;

void tim2_setup(void) {
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
	timer_enable_preload(TIM2);

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
	//timer_enable_irq(TIM2, TIM_DIER_CC1IE);
        if (atomSemCreate (&ibusy, 0) != ATOM_OK) 
            fault(3);
}
void tim2_isr(void) {
    atomIntEnter();
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        //timer_clear_flag(TIM2, TIM_SR_UIF);
        TIM_SR(TIM2) &= ~TIM_SR_UIF;
        //timer_disable_irq(TIM2, TIM_DIER_UIE);
        TIM_DIER(TIM2) &= ~TIM_DIER_UIE; //disable interrupt
        atomSemPut (&ibusy);
    }
    atomIntExit(0);
}

#if 0
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

#if 1
void idelay(uint32_t t){
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
    timer_set_counter(TIM2, t);
    TIM_DIER(TIM2) |= TIM_DIER_UIE;
    //timer_enable_irq(TIM2, TIM_DIER_UIE);
    atomSemGet (&ibusy, 0);
}
#endif


