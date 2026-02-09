#include "mqtt.h"
#include <string.h>
#include "debug.h"

/* ================= VARIÁVEIS ================= */

typedef enum {
    MQTT_STATE_DISCONNECTED,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED
} mqtt_state_t;

static volatile mqtt_state_t mqtt_state = MQTT_STATE_DISCONNECTED;
volatile bool mqtt_can_send = true;

static mqtt_client_t *mqtt_client = NULL;
static uint8_t mqtt_connected = 0;
bool exit_code = false;

/* ================= CALLBACKS ================= */
static
void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                        mqtt_connection_status_t status)
{
    BOOT_PRINT("[MQTT CB] Status conexão: %d\n", status);

    if (status == MQTT_CONNECT_ACCEPTED) {
        BOOT_PRINT("[MQTT CB] Conectado com sucesso!\n");
        mqtt_connected = true;
    } else {
        BOOT_PRINT("[MQTT CB] Falha MQTT. Código: %d\n", status);
        mqtt_connected = false;
    }
}

static void mqtt_pub_cb(void *arg, err_t err)
{
    (void)arg;
    mqtt_can_send = true;
}

static void mqtt_incoming_publish_cb(void *arg,
                                     const char *topic,
                                     u32_t tot_len)
{
    /* Não utilizado */
    (void)arg;
    (void)topic;
    (void)tot_len;
}

static void mqtt_incoming_data_cb(void *arg,
                                  const u8_t *data,
                                  u16_t len,
                                  u8_t flags)
{
    (void)arg;
    (void)flags;
    mqtt_process_command(data, len);
}

/* ================= LÓGICA DE COMANDOS ================= */

uint8_t mqtt_process_command(const uint8_t *data, uint16_t len)
{
    if (len == 4 && memcmp(data, "stop", 4) == 0)
    {
        exit_code = true;
        return MQTT_CMD_STOP;
    }

    if (len == 5 && memcmp(data, "start", 5) == 0)
    {
        exit_code = false;
        return MQTT_CMD_START;
    }

    return MQTT_ERR_INVALID_COMMAND;
}

static void mqtt_client_cleanup(void)
{
    if (mqtt_client != NULL) {
        mqtt_disconnect(mqtt_client);
        mqtt_client_free(mqtt_client);
        mqtt_client = NULL;
    }
}

/* ================= API PÚBLICA ================= */

uint8_t mqtt_start_client(const char *CLIENT_ID,
                          const char *CLIENT_USER,
                          const char *CLIENT_PASS)
{   
    ip_addr_t broker_ip;

    mqtt_client_cleanup();
    mqtt_client = mqtt_client_new();
    if (!mqtt_client)
    {
        BOOT_PRINT("[MQTT CLIENT] Falha ao criar cliente\n");
        return MQTT_ERR_CLIENT_CREATE;
    }
    mqtt_state = MQTT_STATE_CONNECTING;
    BOOT_PRINT("[MQTT CLIENT] Cliente criado com sucesso\n");

    IP4_ADDR(&broker_ip, 192, 168, 0, 198);

    struct mqtt_connect_client_info_t client_info = {
        .client_id   = CLIENT_ID,
        .client_user = CLIENT_USER,
        .client_pass = CLIENT_PASS,
        .keep_alive  = 60,
    };

    mqtt_set_inpub_callback(mqtt_client,
                            mqtt_incoming_publish_cb,
                            mqtt_incoming_data_cb,
                            NULL);
    
    BOOT_PRINT("[MQTT CLIENT] Conectando...\n");
    err_t err = mqtt_client_connect(
        mqtt_client,
        &broker_ip,
        MQTT_BROKER_PORT,
        mqtt_connection_cb,
        NULL,
        &client_info
    );
    return (err == ERR_OK) ? MQTT_OK : MQTT_ERR_CONNECT_FAILED;
}

uint8_t mqtt_is_connected(void)
{
    return mqtt_connected;
}

uint8_t mqtt_publish_data(const char *topic,
                          const uint8_t *payload,
                          uint16_t len)
{
    if (!mqtt_client || !mqtt_connected)
        return MQTT_ERR_CONNECT_FAILED;

    if (!mqtt_can_send)
        return MQTT_ERR_BUSY;

    mqtt_can_send = false;

    err_t err = mqtt_publish(
        mqtt_client,
        topic,
        payload,
        len,
        0,
        0,
        mqtt_pub_cb,
        NULL
    );

    if (err != ERR_OK)
    {
        mqtt_can_send = true;
        return err;
    }

    return MQTT_OK;
}

