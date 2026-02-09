#include "microfone.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <math.h>

#define MIC_GPIO 28
#define MIC_ADC  2

static uint16_t nivel_base = 0;

void mic_init(void) 
{
    adc_init();
    adc_gpio_init(MIC_GPIO);
    adc_select_input(MIC_ADC);

    sleep_ms(100);
}

uint16_t mic_get_amplitude(uint16_t samples) 
{
    uint16_t min = 4095;
    uint16_t max = 0;

    for (uint16_t i = 0; i < samples; i++) {
        uint16_t v = adc_read();
        if (v < min) min = v;
        if (v > max) max = v;
    }

    return max - min;
}

bool mic_detecta_som(uint16_t samples, uint16_t threshold) 
{
    uint16_t amp = mic_get_amplitude(samples);

    if (nivel_base == 0) {
        nivel_base = amp;
    } else {
        nivel_base = (nivel_base * 15 + amp) / 16;
    }

    return (amp > (nivel_base + threshold));
}

uint8_t mic_get_nivel(uint16_t samples) 
{
    static uint16_t max_ref = 200; // começa baixo
    uint16_t amp = mic_get_amplitude(samples);

    // Atualiza referência máxima lentamente
    if (amp > max_ref) {
        max_ref = amp;
    } else {
        max_ref = (max_ref * 31 + amp) / 32;
    }

    if (max_ref < 10) max_ref = 10;

    uint32_t nivel = (amp * 100) / max_ref;
    if (nivel > 100) nivel = 100;

    return (uint8_t)nivel;
}

uint8_t mic_get_nivel_db(uint32_t samples)
{
    uint16_t amp = mic_get_amplitude(samples);
    if (amp < 1) amp = 1;

    float db = 20.0f * log10f((float)amp);
    if (db < 0) db = 0;
    if (db > 60) db = 60;

    return (uint8_t)db;
}


float mic_calcula_rms(uint16_t samples)
{
    uint32_t soma = 0;

    for (uint16_t i = 0; i < samples; i++) {
        int32_t v = adc_read() - 2048; // remove DC
        soma += v * v;
        sleep_us(50);
    }

    float media = (float)soma / samples;
    return sqrtf(media);
}