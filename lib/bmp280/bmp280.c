#include "bmp280.h"
#include "pico/stdlib.h"

/* Registradores */
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET    0xE0
#define BMP280_REG_CTRL     0xF4
#define BMP280_REG_CONFIG   0xF5
#define BMP280_REG_TEMP     0xFA
#define BMP280_CALIB_START  0x88

/* ------------------ I2C helpers ------------------ */

static bool bmp280_write(bmp280_t *dev, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_write_blocking(dev->i2c, dev->addr, buf, 2, false) >= 0;
}

static bool bmp280_read(bmp280_t *dev, uint8_t reg, uint8_t *buf, uint16_t len) {
    if (i2c_write_blocking(dev->i2c, dev->addr, &reg, 1, true) < 0)
        return false;
    if (i2c_read_blocking(dev->i2c, dev->addr, buf, len, false) < 0)
        return false;
    return true;
}

/* ------------------ Calibração ------------------ */

static bool bmp280_read_calibration(bmp280_t *dev) {
    uint8_t c[24];

    if (!bmp280_read(dev, BMP280_CALIB_START, c, 24))
        return false;

    dev->dig_T1 = (c[1] << 8) | c[0];
    dev->dig_T2 = (c[3] << 8) | c[2];
    dev->dig_T3 = (c[5] << 8) | c[4];

    dev->dig_P1 = (c[7] << 8) | c[6];
    dev->dig_P2 = (c[9] << 8) | c[8];
    dev->dig_P3 = (c[11] << 8) | c[10];
    dev->dig_P4 = (c[13] << 8) | c[12];
    dev->dig_P5 = (c[15] << 8) | c[14];
    dev->dig_P6 = (c[17] << 8) | c[16];
    dev->dig_P7 = (c[19] << 8) | c[18];
    dev->dig_P8 = (c[21] << 8) | c[20];
    dev->dig_P9 = (c[23] << 8) | c[22];

    return true;
}

/* ------------------ Init ------------------ */

bool bmp280_init(bmp280_t *dev, i2c_inst_t *i2c, uint8_t addr) {
    uint8_t id;

    dev->i2c = i2c;
    dev->addr = addr;

    if (!bmp280_read(dev, BMP280_REG_ID, &id, 1))
        return false;

    if (id != BMP280_ID && id != BME280_ID)
        return false;

    /* Reset */
    bmp280_write(dev, BMP280_REG_RESET, 0xB6);
    sleep_ms(5);

    if (!bmp280_read_calibration(dev))
        return false;

    /* Temp oversampling x1, normal mode */
    bmp280_write(dev, BMP280_REG_CTRL, 0x27);

    /* Standby 1000ms, filtro off */
    bmp280_write(dev, BMP280_REG_CONFIG, 0xA0);

    return true;
}

/* ------------------ Temperatura ------------------ */

bool bmp280_read_temperature(bmp280_t *dev, float *temperature) {
    uint8_t buf[3];
    int32_t adc_T;
    int32_t var1, var2;

    if (!bmp280_read(dev, BMP280_REG_TEMP, buf, 3))
        return false;

    adc_T = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));

    var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) *
            ((int32_t)dev->dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) *
            ((int32_t)dev->dig_T3)) >> 14;

    dev->t_fine = var1 + var2;

    *temperature = (dev->t_fine * 5 + 128) / 25600.0f;

    return true;
}
