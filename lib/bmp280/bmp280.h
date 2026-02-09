#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* Endereços possíveis */
#define BMP280_ADDR_LOW   0x76
#define BMP280_ADDR_HIGH  0x77

/* ID esperado */
#define BMP280_ID         0x58
#define BME280_ID         0x60

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;

    /* calibração */
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t  dig_P6, dig_P7, dig_P8, dig_P9;

    int32_t t_fine;
} bmp280_t;

bool bmp280_init(bmp280_t *dev, i2c_inst_t *i2c, uint8_t addr);
bool bmp280_read_temperature(bmp280_t *dev, float *temperature);

#endif
