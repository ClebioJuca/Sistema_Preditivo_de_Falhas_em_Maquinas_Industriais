#include "inc/ff.h"
#include "inc/diskio.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

// Status do Drive
static volatile DSTATUS Stat = STA_NOINIT;
// Flag para tipo de cartão (Standard ou High Capacity)
static BYTE CardType;

/*-----------------------------------------------------------------------*/
/* Funções de Controle do SPI (Funções Auxiliares)                       */
/*-----------------------------------------------------------------------*/

// Ativa o CS (Chip Select), selecionando o cartão SD
static void sd_select(void) {
    gpio_put(PIN_CS, 0);
}

// Desativa o CS e envia alguns clocks para liberar o barramento
static void sd_deselect(void) {
    gpio_put(PIN_CS, 1);
    uint8_t d;
    spi_read_blocking(SPI_PORT, 0xFF, &d, 1); // Envia um byte dummy
}

// Envia e recebe um byte via SPI
static uint8_t spi_transfer(uint8_t data) {
    uint8_t rx;
    spi_write_read_blocking(SPI_PORT, &data, &rx, 1);
    return rx;
}

// Espera o cartão SD ficar pronto (retorna 0xFF)
static int wait_ready(void) {
    uint8_t d;
    // O timeout aqui é importante, ~500ms
    for (int i = 0; i < 5000; i++) {
        d = spi_transfer(0xFF);
        if (d == 0xFF) return 1;
        sleep_us(100);
    }
    return 0; // Timeout
}

/*-----------------------------------------------------------------------*/
/* Funções de Comando (Comunicação com o SD Card)                        */
/*-----------------------------------------------------------------------*/

// Recebe um bloco de dados do cartão
static int rcvr_datablock(BYTE *buff, UINT btr) {
    BYTE token;

    // Espera pelo token de início de dados (0xFE)
    for (int i = 0; i < 2000; i++) {
        token = spi_transfer(0xFF);
        if (token != 0xFF) break;
        sleep_us(100);
    }
    if (token != 0xFE) return 0; // Se não for o token, erro

    // Recebe o bloco de dados
    spi_read_blocking(SPI_PORT, 0xFF, buff, btr);
    
    // Descarta o CRC
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    return 1; // Sucesso
}

#if FF_FS_READONLY == 0
// Envia um bloco de dados para o cartão
static int xmit_datablock(const BYTE *buff, BYTE token) {
    BYTE resp;

    if (!wait_ready()) return 0; // Espera o cartão ficar pronto

    spi_transfer(token); // Envia o token de dados
    if (token != 0xFD) { // Se não for um token de parada
        spi_write_blocking(SPI_PORT, buff, SD_BLOCK_SIZE); // Envia o bloco de 512 bytes
        // Envia CRC dummy
        spi_transfer(0xFF);
        spi_transfer(0xFF);
        resp = spi_transfer(0xFF); // Recebe a resposta
        if ((resp & 0x1F) != 0x05) return 0; // Se não for aceito, erro
    }
    return 1; // Sucesso
}
#endif

// Envia um comando para o cartão SD
static BYTE send_cmd(BYTE cmd, DWORD arg) {
    BYTE n, res;

    // ACMD (comandos prefixados com CMD55)
    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    // Seleciona o cartão e espera ficar pronto, exceto para CMD12 (STOP_TRANSMISSION)
    if (cmd != CMD12) {
        sd_deselect();
        sd_select();
        if (!wait_ready()) return 0xFF;
    }

    // Envia o pacote de comando
    spi_transfer(cmd | 0x40);              // Start + Transmissão + Nº Comando
    spi_transfer((BYTE)(arg >> 24));       // Argumento [31..24]
    spi_transfer((BYTE)(arg >> 16));       // Argumento [23..16]
    spi_transfer((BYTE)(arg >> 8));        // Argumento [15..8]
    spi_transfer((BYTE)arg);               // Argumento [7..0]
    
    // CRC (varia para CMD0 e CMD8)
    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;
    spi_transfer(n);

    // Recebe a resposta
    if (cmd == CMD12) spi_transfer(0xFF); // Pula um byte de stuff após CMD12
    n = 10; // Tenta ler a resposta 10 vezes
    do {
        res = spi_transfer(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

/*--------------------------------------------------------------------------

   Funções Públicas (Interface para o FatFs)

---------------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE pdrv) {
    BYTE n, cmd, ty, ocr[4];

    if (pdrv != 0) return STA_NOINIT;

    // Inicializa a porta SPI a uma velocidade baixa (100-400 kHz)
    spi_init(SPI_PORT, 400 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    sd_deselect();

    // Envia 80 pulsos de clock para o cartão "acordar"
    for (n = 10; n; n--) spi_transfer(0xFF);

    ty = 0;
    // Coloca o cartão em modo Idle (CMD0)
    if (send_cmd(CMD0, 0) == 1) {
        // Envia CMD8 para verificar se é um cartão v2.0+
        if (send_cmd(CMD8, 0x1AA) == 1) { // SDv2
            for (n = 0; n < 4; n++) ocr[n] = spi_transfer(0xFF); // Lê o resto da resposta
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) { // O cartão aceita 2.7-3.6V
                // Espera o fim da inicialização (ACMD41 com HCS bit)
                for (int i = 0; i < 1000; i++) {
                    if (send_cmd(ACMD41, 1UL << 30) == 0) break;
                    sleep_ms(1);
                }
                // Lê o registrador OCR para verificar a capacidade (CMD58)
                if (send_cmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = spi_transfer(0xFF);
                    ty = (ocr[0] & 0x40) ? CT_SDC2 | CT_BLOCK : CT_SDC2; // SDv2 (HC ou SC)
                }
            }
        } else { // SDv1 ou MMCv3
            if (send_cmd(ACMD41, 0) <= 1) {
                ty = CT_SDC1; cmd = ACMD41; // SDv1
            } else {
                ty = CT_MMC3; cmd = CMD1; // MMCv3
            }
            // Espera o fim da inicialização
            for (int i = 0; i < 1000; i++) {
                if (send_cmd(cmd, 0) == 0) break;
                sleep_ms(1);
            }
            if (send_cmd(CMD16, SD_BLOCK_SIZE) != 0) ty = 0; // Define o tamanho do bloco
        }
    }
    CardType = ty;
    sd_deselect();

    if (ty) {
        // Aumenta a velocidade do SPI para a operação normal
        spi_set_baudrate(SPI_PORT, 12500 * 1000); // 12.5 MHz
        Stat &= ~STA_NOINIT;
    } else {
        Stat = STA_NOINIT;
    }

    return Stat;
}

DSTATUS disk_status(BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;
    return Stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    // Converte LBA para endereço de byte se não for um cartão de bloco (SDv1)
    if (!(CardType & CT_BLOCK)) sector *= SD_BLOCK_SIZE;

    // Comando de leitura de múltiplos blocos
    if (count > 1) {
        if (send_cmd(CMD18, sector) != 0) return RES_ERROR;
        do {
            if (!rcvr_datablock(buff, SD_BLOCK_SIZE)) break;
            buff += SD_BLOCK_SIZE;
        } while (--count);
        send_cmd(CMD12, 0); // Para a transmissão
    } else { // Comando de leitura de bloco único
        if (send_cmd(CMD17, sector) != 0 || !rcvr_datablock(buff, SD_BLOCK_SIZE)) {
            return RES_ERROR;
        }
        count = 0;
    }
    sd_deselect();
    
    return count ? RES_ERROR : RES_OK;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (!(CardType & CT_BLOCK)) sector *= SD_BLOCK_SIZE;

    if (count > 1) { // Escrita de múltiplos blocos
        if (send_cmd(CMD25, sector) != 0) return RES_ERROR;
        do {
            if (!xmit_datablock(buff, 0xFC)) break;
            buff += SD_BLOCK_SIZE;
        } while (--count);
        if (!xmit_datablock(0, 0xFD)) count = 1; // Token de parada
    } else { // Escrita de bloco único
        if (send_cmd(CMD24, sector) != 0 || !xmit_datablock(buff, 0xFE)) {
            return RES_ERROR;
        }
        count = 0;
    }
    sd_deselect();

    return count ? RES_ERROR : RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    DRESULT res;
    BYTE n, csd[16];
    DWORD csize;

    if (pdrv != 0) return RES_PARERR;

    res = RES_ERROR;

    if (cmd == CTRL_SYNC) {
        sd_select();
        if (wait_ready()) res = RES_OK;
        sd_deselect();
    } else if (cmd == GET_SECTOR_COUNT) {
        if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) { // SDC ver 2.00
                csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(LBA_t*)buff = csize << 10;
            } else { // SDC ver 1.XX
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                *(LBA_t*)buff = csize << (n - 9);
            }
            res = RES_OK;
        }
        sd_deselect();
    } else if (cmd == GET_SECTOR_SIZE) {
        *(WORD*)buff = SD_BLOCK_SIZE;
        res = RES_OK;
    } else if (cmd == GET_BLOCK_SIZE) {
        *(DWORD*)buff = 1;
        res = RES_OK;
    }

    return res;
}