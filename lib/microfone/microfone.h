#ifndef MICROFONE_H
#define MICROFONE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float rms;
    uint32_t timestamp_ms;
} mic_rms_msg_t;

/**
 * @brief Inicializa o microfone (ADC)
 */
void mic_init(void);

/**
 * @brief Mede a amplitude do sinal de áudio
 *
 * @param samples Número de amostras usadas na medição
 * @return Amplitude (max - min)
 */
uint16_t mic_get_amplitude(uint16_t samples);

/**
 * @brief Detecção relativa de som
 *
 * @param samples Número de amostras
 * @param threshold Diferença mínima acima do nível base
 * @return true se som forte detectado
 */
bool mic_detecta_som(uint16_t samples, uint16_t threshold);

/**
 * @brief Retorna nível de som em porcentagem (0–100)
 *
 * @param samples Número de amostras
 * @return Percentual do nível sonoro
 */
uint8_t mic_get_nivel(uint16_t samples);

uint8_t mic_get_nivel_db(uint32_t samples);

float mic_calcula_rms(uint16_t samples);

#endif
