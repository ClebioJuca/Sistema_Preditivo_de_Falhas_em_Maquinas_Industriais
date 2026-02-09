#include "inc/sd.h"
#include <stdio.h>
#include "hardware/spi.h"
#include "inc/sd_card.h"

bool sd_init(void)
{
    gpio_init(SD_CS_PIN);
    gpio_set_dir(SD_CS_PIN, GPIO_OUT);
    gpio_put(SD_CS_PIN, 1);

    printf("Inicializando SD Card...\n");

    //sd_spi_reset();

    for (int i = 0; i < 5; i++) {

        if (sd_card_init()) {
            printf("SD Card OK!\n");
            return true;
        }

        printf("Tentativa %d falhou, resetando SD...\n", i + 1);
        //sd_spi_reset();
        sleep_ms(200);
    }

    printf("❌ SD falhou após múltiplas tentativas.\n");
    return false;
}
