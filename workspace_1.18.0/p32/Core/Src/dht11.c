/*
 * dht11.c
 *
 *  Created on: Dec 5, 2025
 *      Author: Manu_s_r
 */

#include "dht11.h"

/* ================= PRIVATE BUFFER ================= */
static uint8_t dht_data[5];

/* ---------------- DWT Delay ---------------- */
static void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ---------------- GPIO Control ---------------- */
static void DHT11_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

static void DHT11_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/* ---------------- Init ---------------- */
void DHT11_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    DWT_Delay_Init();
}

/* ---------------- Read Function ---------------- */
DHT11_Status_t DHT11_Read(float *temperature, float *humidity)
{
    uint8_t i, j;

    /* Start signal */
    DHT11_SetPinOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);

    /* Switch to input */
    DHT11_SetPinInput();

    /* Wait for response */
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
    while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));

    /* Read 40 bits */
    for (j = 0; j < 5; j++)
    {
        dht_data[j] = 0;
        for (i = 0; i < 8; i++)
        {
            while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
            delay_us(35);
            if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
                dht_data[j] |= (1 << (7 - i));
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
        }
    }

    /* Checksum */
    if (dht_data[4] !=
        (dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3]))
    {
        return DHT11_ERROR;
    }

    *humidity    = (float)dht_data[0];
    *temperature = (float)dht_data[2];

    return DHT11_OK;
}
