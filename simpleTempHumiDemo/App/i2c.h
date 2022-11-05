/*
 * i2c.h
 *
 *  Created on: Jun 26, 2014
 *      Author: saman
 */

#ifndef I2C_H_
#define I2C_H_
#include "wisp-base.h"
#include "HDC2010.h"
void i2c_init(void);
void i2c_write(uint8_t addr, uint8_t wrData);
uint8_t i2c_read(uint8_t addr);

#endif /* I2C_H_ */
