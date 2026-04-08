/*
 * bmp.c
 *
 * BMP180 pressure-only driver
 */

#include "bmp.h"

/* BMP180 I2C address */
#define BMP180_ADDR        (0x77 << 1)

/* BMP180 registers */
#define BMP180_CALIB_START 0xAA
#define BMP180_CONTROL     0xF4
#define BMP180_TEMP_CMD    0x2E
#define BMP180_PRESS_CMD   0x34
#define BMP180_OUT_MSB     0xF6

/* Calibration coefficients */
static int16_t  AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;

static I2C_HandleTypeDef *bmp_i2c;

/* ---------------- Low-level I2C ---------------- */
static void BMP_Read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(bmp_i2c,
                     BMP180_ADDR,
                     reg,
                     I2C_MEMADD_SIZE_8BIT,
                     buf,
                     len,
                     HAL_MAX_DELAY);
}

static void BMP_Write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(bmp_i2c,
                      BMP180_ADDR,
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      HAL_MAX_DELAY);
}

/* ---------------- Init ---------------- */
void BMP180_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[22];
    bmp_i2c = hi2c;

    BMP_Read(BMP180_CALIB_START, buf, 22);

    AC1 = (buf[0] << 8) | buf[1];
    AC2 = (buf[2] << 8) | buf[3];
    AC3 = (buf[4] << 8) | buf[5];
    AC4 = (buf[6] << 8) | buf[7];
    AC5 = (buf[8] << 8) | buf[9];
    AC6 = (buf[10] << 8) | buf[11];
    B1  = (buf[12] << 8) | buf[13];
    B2  = (buf[14] << 8) | buf[15];
    MB  = (buf[16] << 8) | buf[17];
    MC  = (buf[18] << 8) | buf[19];
    MD  = (buf[20] << 8) | buf[21];
}

/* ---------------- Read Pressure ---------------- */
float BMP180_ReadPressure(void)
{
    int32_t UT, UP;
    int32_t X1, X2, X3, B3, B5, B6, p;
    uint32_t B4, B7;
    uint8_t buf[3];

    /* --- Read temperature (required for pressure) --- */
    BMP_Write(BMP180_CONTROL, BMP180_TEMP_CMD);
    HAL_Delay(5);

    BMP_Read(BMP180_OUT_MSB, buf, 2);
    UT = (buf[0] << 8) | buf[1];

    X1 = ((UT - AC6) * AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;

    /* --- Read pressure --- */
    BMP_Write(BMP180_CONTROL, BMP180_PRESS_CMD);
    HAL_Delay(8);

    BMP_Read(BMP180_OUT_MSB, buf, 3);
    UP = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 8;

    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6 >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) + 2) >> 2);

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * (B6 * B6 >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * 50000;

    if (B7 < 0x80000000)
        p = (B7 << 1) / B4;
    else
        p = (B7 / B4) << 1;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);

    return (float)p;   // Pressure in Pa
}
