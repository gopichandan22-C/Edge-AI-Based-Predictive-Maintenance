#ifndef __DHT11_H__
#define __DHT11_H__

#include "stm32l4xx_hal.h"

/* ===== USER CONFIG ===== */
#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_1
/* ======================= */

typedef enum
{
    DHT11_OK = 0,
    DHT11_ERROR,
    DHT11_TIMEOUT
} DHT11_Status_t;

/* API */
void DHT11_Init(void);
DHT11_Status_t DHT11_Read(float *temperature, float *humidity);

#endif
