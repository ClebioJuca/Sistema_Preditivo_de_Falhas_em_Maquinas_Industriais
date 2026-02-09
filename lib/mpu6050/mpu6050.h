#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"
#include <stdbool.h>

#define MPU6050_ADDR 0x68

typedef struct {
    float rms;
    float peak;
    bool ready;   // indica se a janela RMS terminou
} vibration_data_t;

void mpu6050_init(i2c_inst_t *i2c);
void mpu6050_vibration_init(uint32_t sample_rate_hz, uint32_t window_ms);
void mpu6050_process_sample(i2c_inst_t *i2c);
vibration_data_t mpu6050_get_vibration(void);

#endif
