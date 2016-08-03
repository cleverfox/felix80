#ifndef IIC_H
#define IIC_H

#include <stdint.h>

void i2c1_setup(void);
void i2c2_setup(void);
void iic_write(uint32_t i2c, uint8_t addr);

#endif

