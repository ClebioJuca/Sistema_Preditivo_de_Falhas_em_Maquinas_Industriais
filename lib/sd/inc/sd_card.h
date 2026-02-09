#ifndef SD_CARD_H
#define SD_CARD_H

#define SD_CS_PIN 17

#include <stdbool.h>

typedef enum {
    SD_OK = 0,
    SD_MOUNT_FAILED,
    SD_OPEN_FAILED,
    SD_WRITE_FAILED,
    SD_READ_FAILED,
    SD_CARD_NOT_INITIALIZED
} sd_status_t;


bool sd_card_init(void);
sd_status_t sd_card_write_text(const char* filename, const char* text);
sd_status_t sd_card_read(const char* filename, char* buffer, int buffer_size);
sd_status_t sd_card_append_json(const char* filename, const char* json_string);
sd_status_t sd_spi_reset(void);

#endif // SD_CARD_H