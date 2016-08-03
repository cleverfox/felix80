#ifndef SLEEP_H
#define SLEEP_H

#include <stdint.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

void tim2_setup(void);
void cdelay(uint32_t);

#endif
