#ifndef INC_HDC2010_H_
#define INC_HDC2010_H_

#include "stm32f1xx.h"

#define SENS_ADDR			0x82
extern I2C_HandleTypeDef i2c;

float read_temp(void);
float read_humid(void);

#endif /* INC_HDC2010_H_ */
