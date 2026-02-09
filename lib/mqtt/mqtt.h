#ifndef MQTT_LIB_H
#define MQTT_LIB_H

#include <stdbool.h>
#include <stdint.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"

/* ================= CÓDIGOS DE RETORNO ================= */

#define MQTT_OK                     0
#define MQTT_ERR_CLIENT_CREATE      1
#define MQTT_ERR_CONNECT_FAILED     2
#define MQTT_ERR_INVALID_COMMAND    3
#define MQTT_CMD_START              10
#define MQTT_CMD_STOP               11
#define MQTT_ERR_BUSY               20

/* ================= CONFIGURAÇÕES ================= */

#define MQTT_BROKER_PORT 1883

/* ================= VARIÁVEIS EXTERNAS ================= */

extern bool exit_code;

/* ================= API PÚBLICA ================= */

/**
 * @brief Inicia o cliente MQTT e tenta conexão
 * @return MQTT_OK ou erro
 */
uint8_t mqtt_start_client(const char *CLIENT_ID, const char *CLIENT_USER, const char *CLIENT_PASS);

/**
 * @brief Processa comandos recebidos via MQTT
 * @return Código do comando ou erro
 */
uint8_t mqtt_process_command(const uint8_t *data, uint16_t len);

/**
 * @brief Retorna o status da conexão MQTT
 * @return 1 conectado | 0 desconectado
 */
uint8_t mqtt_is_connected(void);

/**
 * @brief Retorna o status da conexão MQTT
 * @return 1 conectado | 0 desconectado
 */
uint8_t mqtt_publish_data(const char *topic,
                          const uint8_t *payload,
                          uint16_t len);

#endif /* MQTT_LIB_H */
