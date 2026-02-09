#include "mpu6050.h"
#include "pico/stdlib.h"
#include <math.h>

// Registradores
#define REG_PWR_MGMT_1       0x6B
#define REG_ACCEL_XOUT_H     0x3B

// Configuração interna
static uint32_t samples_per_window = 0;
static uint32_t sample_count = 0;

static float sum_sq = 0.0f;
static float peak = 0.0f;
static vibration_data_t vib_data = {0};

static void write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c, MPU6050_ADDR, buf, 2, false);
}

static void read_regs(i2c_inst_t *i2c, uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(i2c, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c, MPU6050_ADDR, buf, len, false);
}

void mpu6050_init(i2c_inst_t *i2c) {
    sleep_ms(100);
    write_reg(i2c, REG_PWR_MGMT_1, 0x00);
}

void mpu6050_vibration_init(uint32_t sample_rate_hz, uint32_t window_ms) {
    samples_per_window = (sample_rate_hz * window_ms) / 1000;
    sample_count = 0;
    sum_sq = 0.0f;
    peak = 0.0f;
    vib_data.ready = false;
}

static float read_vibration_sample(i2c_inst_t *i2c) {
    uint8_t data[6];
    read_regs(i2c, REG_ACCEL_XOUT_H, data, 6);

    int16_t rx = (data[0] << 8) | data[1];
    int16_t ry = (data[2] << 8) | data[3];
    int16_t rz = (data[4] << 8) | data[5];

    float ax = rx / 16384.0f;
    float ay = ry / 16384.0f;
    float az = rz / 16384.0f;

    float mag = sqrt(ax*ax + ay*ay + az*az);
    return fabs(mag - 1.0f); // remove gravidade
}

void mpu6050_process_sample(i2c_inst_t *i2c) {
    float vib = read_vibration_sample(i2c);

    sum_sq += vib * vib;
    sample_count++;

    if (vib > peak)
        peak = vib;

    if (sample_count >= samples_per_window) {
        vib_data.rms = sqrt(sum_sq / sample_count);
        vib_data.peak = peak;
        vib_data.ready = true;

        // reset
        sample_count = 0;
        sum_sq = 0.0f;
        peak = 0.0f;
    }
}

vibration_data_t mpu6050_get_vibration(void) {
    vibration_data_t out = vib_data;
    vib_data.ready = false; // consome o dado
    return out;
}
