#ifndef SD_H
#define SD_H

#include <stdbool.h>
#include "pico/stdlib.h"

// ===============================
// CONFIGURAÇÃO DE PINOS
// ===============================
#define SD_CS_PIN   17

// ===============================
// API
// ===============================
bool sd_init(void);

#endif // SD_H