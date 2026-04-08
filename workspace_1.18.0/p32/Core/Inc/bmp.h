#ifndef __BMP_H__
#define __BMP_H__

#include "stm32l4xx_hal.h"

void  BMP180_Init(I2C_HandleTypeDef *hi2c);
float BMP180_ReadPressure(void);

#endif
