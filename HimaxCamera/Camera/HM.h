/*
 * HM.h
 *
 *  Created on: Sep 12, 2022
 *      Author: saffaria
 */

#ifndef CAMERA_HM_H_
#define CAMERA_HM_H_

#include "HMRegs.h"




void initI2C(void);
uint8_t init_Himax(void);
void init_SPI(void);


#endif /* CAMERA_HM_H_ */
