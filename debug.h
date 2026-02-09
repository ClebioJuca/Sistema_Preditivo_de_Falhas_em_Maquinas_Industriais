#ifndef DEBUG_H
#define DEBUG_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>

extern SemaphoreHandle_t serialMutex;

/* #define DEBUG */
// Comente a linha abaixo para desabilitar as mensagens de debug
#define DEBUG

#ifdef DEBUG

    #define BOOT_PRINT(fmt, ...) \
        printf(fmt, ##__VA_ARGS__)

    #define TASK_PRINT(fmt, ...)                          \
        do {                                               \
            xSemaphoreTake(serialMutex, portMAX_DELAY);   \
            printf(fmt, ##__VA_ARGS__);                   \
            xSemaphoreGive(serialMutex);                  \
        } while (0)

#else
    #define BOOT_PRINT(fmt, ...) do {} while (0)
    #define TASK_PRINT(fmt, ...) do {} while (0)

#endif /* DEBUG */  
#endif /* DEBUG_H */