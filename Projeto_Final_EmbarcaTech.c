#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "hardware/pwm.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lib/ssd1306/ssd1306.h"
#include "lib/microfone/microfone.h"
#include "lib/bmp280/bmp280.h"
#include "lib/mpu6050/mpu6050.h"
#include "lib/sd/inc/sd.h"
#include "lib/sd/inc/sd_card.h"
#include "lib/sd/inc/ff.h"
#include "lib/mqtt/mqtt.h"
#include "lib/config/config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "debug.h"

// Define para o SD Card - Comente para desabilitar funcionalidades relacionadas ao SD
#define SD_CARD

// Defines do I2C para o BMP280 e MPU6050
#define I2C_PORT i2c0
#define I2C_SDA  0
#define I2C_SCL  1

// Defines do I2C para o Display OLED
#define I2C_SDA1 14
#define I2C_SCL1 15

// Pinos do LED RGB
#define LED_VERMELHO 13
#define LED_AZUL     12
#define LED_VERDE    11

#define BUZZER_PIN 10

#define SAMPLE_RATE_HZ  500
#define WINDOW_MS       500
#define NUM_SAMPLES     (SAMPLE_RATE_HZ * WINDOW_MS / 1000)

#define MIC_SAMPLES_RMS     400
#define MIC_ENVIO_MS        1000   // X tempo (2s)
#define MIC_PERIODO_MS      50

#define RECOVERY_MAX_FAILS 5           // Máximo de falhas consecutivas antes do reboot
#define RECOVERY_FAIL_WINDOW_MS 60000  // Janela de tempo para contar falhas (1 minuto)

SemaphoreHandle_t serialMutex, i2cMutex;
QueueHandle_t mpu6050Queue, micQueue, oledQueue, mqttQueue, sdQueue, verificationQueue;

EventGroupHandle_t sys_event_group;
#define EVT_MQTT_READY   (1 << 0)

EventGroupHandle_t fault_event_group;
#define FAULT_WIFI     (1 << 0)
#define FAULT_MQTT     (1 << 1)
#define FAULT_SD       (1 << 2)
#define FAULT_BMP280  (1 << 3)
#define FAULT_MPU6050 (1 << 4)
#define FAULT_MIC     (1 << 5)

EventGroupHandle_t recovery_event_group;
#define RECOVER_SD     (1 << 0)
#define RECOVER_WIFI   (1 << 1)
#define RECOVER_MQTT   (1 << 2)

TaskHandle_t sdTaskHandle     = NULL;
TaskHandle_t mqttTaskHandle   = NULL;
TaskHandle_t wifiTaskHandle   = NULL; 
TaskHandle_t sensorTaskHandle = NULL;

bmp280_t bmp280_dev;

// Estrutura para dados dos sensores
typedef struct {
    int contador;
    float temperatura;
    float vibracao;
    float ruido;
    time_t timestamp;      
    uint32_t uptime_ms;    
    bool timestamp_valid;
} sensor_data_t;

// Estrutura para parâmetros da OLED
typedef struct {
    uint8_t *ssd;
    struct render_area *frame_area;
}   oled_params_t;

void vInitTask(void *pvParameters);
void vSystemStateTask(void *pvParameters);
void vMpu6050Task(void *pvParameters);
void vMicTask(void *pvParameters);
void vSensorTask(void *pvParameters);
void vOledTask(void *pvParameters);
void vMqttTask(void *pvParameters);
void vWifiMonitorTask(void *pvParameters);
void vRecoveryTask(void *pvParameters);

#ifdef  SD_CARD
void vSDTask();
void format_data_to_json(const sensor_data_t *data, char *buffer, int size);
#endif

uint wifi_init();
static inline void rgb_off();

static uint buzzer_slice;
static uint buzzer_channel;

void buzzer_init();
void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms, bool free_rtos);

static bool time_synced = false;

int main() {
    stdio_init_all();
    int critical_error = 0;
    
    gpio_init(LED_VERDE);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
    gpio_put(LED_VERDE, 1);

    gpio_init(0);
    gpio_init(1);
    gpio_set_dir(0, GPIO_IN);
    gpio_set_dir(1, GPIO_IN);
    BOOT_PRINT("SDA=%d SCL=%d\n", gpio_get(0), gpio_get(1));

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1);
    gpio_pull_up(I2C_SCL1);

    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0); 
    sleep_ms(2000);

    buzzer_init();
    buzzer_beep(4000, 150, false);

    BOOT_PRINT("=== Inicializando Display OLED ===\n");
    ssd1306_init();
    static struct render_area frame_area = 
    {
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&frame_area);

    static uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    char *text[] = 
    {
    //  "               "
        "",
        "",
        " Inicializando ",
        "",
        "",
        "",
        "",
        "",
    };
    int y = 0;
    for (uint i = 0; i < count_of(text); i++)
    {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);

    static oled_params_t oled_params;
    oled_params.ssd = ssd;
    oled_params.frame_area = &frame_area;

    BOOT_PRINT("=== Inicializando BMP280 ===\n");
    if(!bmp280_init(&bmp280_dev, I2C_PORT, BMP280_ADDR_LOW)) {
        BOOT_PRINT("BMP280 não encontrado!\n");
        critical_error += 1;
    }
    else {
        BOOT_PRINT("BMP280 inicializado com sucesso.\n");
    }

    BOOT_PRINT("=== Inicializando Microfone ===\n");
    mic_init();

    BOOT_PRINT("=== Inicializando MPU6050 ===\n");
    mpu6050_init(I2C_PORT);
    mpu6050_vibration_init(SAMPLE_RATE_HZ, WINDOW_MS);

    #ifdef  SD_CARD
    BOOT_PRINT("=== Inicializando SD CARD ===\n");
    if (!sd_init()) {
        BOOT_PRINT("ERRO: SD não inicializou.\n");
        BOOT_PRINT("Continuando sem o SD Card...\n");
    }
    else {
        BOOT_PRINT("SD inicializado com sucesso.\n");
    }

    f_mkdir("log");
    #endif

    if(wifi_init()){
        BOOT_PRINT("ERRO CRÍTICO: Wi-Fi não conectou.\n");
        critical_error += 2;
    }
    else
        BOOT_PRINT("Wi-Fi conectado com sucesso.\n");

    if (critical_error != 0) {
        BOOT_PRINT("Ocorreu um erro crítico durante a inicialização. Reiniciando o sistema...\n");
        memset(ssd, 0x00, ssd1306_buffer_length);
        render_on_display(ssd, &frame_area);

        switch (critical_error)
        {
        case 1:
            BOOT_PRINT("Erro no BMP280\n");
            text[0] = "    Erro no   ";
            text[1] = "    BMP280    ";
            text[2] = "";
            text[3] = "";        
            break;
    
        case 2:
            BOOT_PRINT("Erro no Wi-Fi\n");
            text[0] = "    Erro no   ";
            text[1] = "    Wi-Fi     ";
            text[2] = "";
            text[3] = "";          
            break;
        
        case 3:
            BOOT_PRINT("Erros no BMP280 e Wi-Fi\n");
            text[0] = "    Erro no   ";
            text[1] = "    BMP280    ";
            text[2] = "    Wi-Fi     ";
            text[3] = ""; 
            break;
        }
        text[4] = " Erro Crítico! ";
        text[5] = "   Efetuando   ";
        text[6] = "   Reboot...   ";
        text[7] = "";

        int y = 0;
        for (uint i = 0; i < count_of(text); i++)
        {
            ssd1306_draw_string(ssd, 5, y, text[i]);
            y += 8;
        }
        render_on_display(ssd, &frame_area);
        rgb_off();

        gpio_init(LED_VERMELHO);
        gpio_set_dir(LED_VERMELHO, GPIO_OUT);

        for (int i = 0; i < 5; i++) {
        gpio_put(LED_VERMELHO, 1);
        sleep_ms(1000);

        gpio_put(LED_VERMELHO, 0);
        sleep_ms(1000);
        }

        // Reboot do RP2040
        watchdog_reboot(0, 0, 0);

        // Segurança: caso algo dê errado
        while (true) {
            tight_loop_contents();
        }
    }

    i2cMutex = xSemaphoreCreateMutex();
    configASSERT(i2cMutex != NULL);

    serialMutex = xSemaphoreCreateMutex();
    configASSERT(serialMutex != NULL);

    mpu6050Queue = xQueueCreate(5, sizeof(vibration_data_t));
    micQueue = xQueueCreate(5, sizeof(mic_rms_msg_t));
    oledQueue = xQueueCreate(5, sizeof(sensor_data_t));
    mqttQueue = xQueueCreate(5, sizeof(sensor_data_t));


    #ifdef  SD_CARD
    sdQueue = xQueueCreate(5, sizeof(sensor_data_t));
    #endif

    verificationQueue = xQueueCreate(5, sizeof(bool));

    sys_event_group = xEventGroupCreate();
    configASSERT(sys_event_group != NULL);
    fault_event_group = xEventGroupCreate();
    configASSERT(fault_event_group != NULL);

    // Criação das tarefas
    #ifdef  SD_CARD
    if (sdQueue != NULL && mpu6050Queue != NULL && oledQueue != NULL && mqttQueue != NULL && verificationQueue != NULL)
    #else
    if (mpu6050Queue != NULL && oledQueue != NULL && mqttQueue != NULL && verificationQueue != NULL)
    #endif 
    {  
        xTaskCreate(vSystemStateTask, "System State Task", 512, NULL, 1, NULL);
        xTaskCreate(vInitTask, "INIT", 1024, &oled_params, 3, NULL);
        xTaskCreate(vMicTask, "Microfone Task", 1024, NULL, 3, NULL);
        xTaskCreate(vMpu6050Task, "Mpu6050 Task", 1024, NULL, 3, NULL);
        xTaskCreate(vSensorTask, "Sensor Task", 1024, NULL, 2, &sensorTaskHandle);
        xTaskCreate(vOledTask, "OLED Task", 1024, &oled_params, 1, NULL);
        xTaskCreate(vMqttTask, "MQTT Task", 4096, NULL, 1, &mqttTaskHandle);
        xTaskCreate(vWifiMonitorTask, "WiFi Monitor Task", 512, NULL, 1, &wifiTaskHandle);
        xTaskCreate(vRecoveryTask, "Recovery Task", 1024, &oled_params, 1, NULL);

        #ifdef SD_CARD
        xTaskCreate(vSDTask, "SD Task", 4096, NULL, 1, &sdTaskHandle);
        #endif
    }

    BOOT_PRINT("=== Iniciando Scheduler FreeRTOS ===\n");
    vTaskStartScheduler();
    
    TASK_PRINT("Sistema falhou.\n");
    while (true) {
    }
}

void vInitTask(void *pvParameters)
{
    const TickType_t timeout = pdMS_TO_TICKS(5000);
    TickType_t start = xTaskGetTickCount();

    TASK_PRINT("[INIT MQTT] Iniciando cliente MQTT\n");
    mqtt_start_client(CLIENT_ID, CLIENT_USER, CLIENT_PASS);

    while (!mqtt_is_connected())
    {
        if (xTaskGetTickCount() - start > pdMS_TO_TICKS(10000))
        {
            TASK_PRINT("[INIT MQTT] Timeout ao conectar MQTT\n");
            xEventGroupSetBits(fault_event_group, FAULT_MQTT);
            vTaskSuspend(NULL); // Erro crítico
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    TASK_PRINT("[INIT MQTT] MQTT conectado com sucesso!\n");
    xEventGroupSetBits(sys_event_group, EVT_MQTT_READY);
    vTaskDelete(NULL);
}

void vSystemStateTask(void *pvParameters)
{
    gpio_init(LED_VERMELHO);
    gpio_init(LED_VERDE);
    gpio_init(LED_AZUL);

    gpio_set_dir(LED_VERMELHO, GPIO_OUT);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
    gpio_set_dir(LED_AZUL, GPIO_OUT);

    for (;;) {

        EventBits_t faults = xEventGroupGetBits(fault_event_group);

        rgb_off();

        if (faults == 0) {
            gpio_put(LED_VERDE, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else if (faults & (FAULT_WIFI | FAULT_MQTT)) {
            gpio_put(LED_VERMELHO, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        else {
            gpio_put(LED_VERMELHO, 1);
            gpio_put(LED_VERDE, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        rgb_off();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vMpu6050Task(void *pvParameters){
    xEventGroupWaitBits(
        sys_event_group,
        EVT_MQTT_READY,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vibration_data_t vib;
    static float last_rms = 0.0f;

    for (;;) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        mpu6050_process_sample(I2C_PORT);
        xSemaphoreGive(i2cMutex);
        vib = mpu6050_get_vibration();
        if (vib.ready) {
            if (isinf(vib.rms)) {
                TASK_PRINT("[MPU6050] RMS inválido detectado, usando último valor válido: %.5f\n", last_rms);
                vib.rms = last_rms;
            }
            else
            {
                TASK_PRINT("[MPU6050] RMS: %.5f, Peak: %.5f\n", vib.rms, vib.peak);
                last_rms = vib.rms;
            }
            xQueueSend(mpu6050Queue, &vib, 0); 
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4)); 
    }
}

void vMicTask(void *pvParameters)
{
    xEventGroupWaitBits(
        sys_event_group,
        EVT_MQTT_READY,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );
    
    TickType_t lastWake = xTaskGetTickCount();
    float soma_rms = 0.0f;
    uint16_t count = 0;

    for(;;){
        float rms = mic_calcula_rms(MIC_SAMPLES_RMS);

        soma_rms += rms;
        count++;

        if ((count * MIC_PERIODO_MS) >= MIC_ENVIO_MS) {
            mic_rms_msg_t msg;
            msg.rms = soma_rms / count;
            msg.timestamp_ms = to_ms_since_boot(get_absolute_time());
            TASK_PRINT("[MIC] Enviando RMS médio: %.5f\n", msg.rms);
            xQueueSend(micQueue, &msg, 0);

            soma_rms = 0;
            count = 0;
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MIC_PERIODO_MS));
    }
}

void vSensorTask(void *pvParameters){
    xEventGroupWaitBits(
        sys_event_group,
        EVT_MQTT_READY,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );
    static int contador = 0;
    sensor_data_t sensorData;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static float last_vib = 0.0f;
    static float last_ruido = 0.0f;

    for(;;){
        memset(&sensorData, 0, sizeof(sensor_data_t));
        contador++;
        sensorData.contador = contador;
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        if (bmp280_read_temperature(&bmp280_dev, &sensorData.temperatura)) {
            TASK_PRINT("[Sensores] Temperatura lida: %.2f C\n", sensorData.temperatura);
            xEventGroupClearBits(fault_event_group, FAULT_BMP280);
        } else {
            TASK_PRINT("[Sensores] Falha ao ler temperatura do BMP280!\n");
            xEventGroupSetBits(fault_event_group, FAULT_BMP280);
            sensorData.temperatura = NAN;
        }
        xSemaphoreGive(i2cMutex);

        if (xQueueReceive(mpu6050Queue, &sensorData.vibracao, 0) == pdPASS) {
        } else {
            sensorData.vibracao = NAN;
        }
        
        if (isinf(sensorData.vibracao) || isnan(sensorData.vibracao)) {
            TASK_PRINT("[Sensores] Vibração inválida detectada, usando último valor válido: %.5f\n", last_vib);
            sensorData.vibracao = last_vib;
        }
        TASK_PRINT("[Sensores] Vibração lida: %.2f\n", sensorData.vibracao);
        last_vib = sensorData.vibracao;

        if (xQueueReceive(micQueue, &sensorData.ruido, 0) == pdPASS) {
        } else {
            sensorData.ruido = NAN;
        }
        if (isinf(sensorData.ruido) || isnan(sensorData.ruido)) {
            TASK_PRINT("[Sensores] Ruído inválido detectado, usando último valor válido: %.5f\n", last_ruido);
            sensorData.ruido = last_ruido;
        }

        TASK_PRINT("[Sensores] Ruído lido: %.2f\n", sensorData.ruido);
        last_ruido = sensorData.ruido;

        sensorData.uptime_ms = to_ms_since_boot(get_absolute_time());

        if (time_synced) {
            sensorData.timestamp = time(NULL);
            sensorData.timestamp_valid = true;
        } else {
            sensorData.timestamp_valid = false;
        }

        EventBits_t faults = xEventGroupGetBits(fault_event_group);

        xQueueSend(oledQueue, &sensorData, portMAX_DELAY);

        if(!(faults & FAULT_MQTT)){
            xQueueOverwrite(mqttQueue, &sensorData);
        }

        #ifdef  SD_CARD
        if (!(faults & FAULT_SD)) {
            xQueueOverwrite(sdQueue, &sensorData);
        }
        #endif

        TASK_PRINT("[TIME] valid=%d uptime=%lu epoch=%lu\n",
        sensorData.timestamp_valid,
        sensorData.uptime_ms,
        (uint32_t)sensorData.timestamp);

        TASK_PRINT("[Sensores] Dados do Sensor enviados!\n");
        TASK_PRINT("[Sensores] Stack livre: %u\n",
        uxTaskGetStackHighWaterMark(NULL));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void vOledTask(void *pvParameters){
    xEventGroupWaitBits(
        sys_event_group,
        EVT_MQTT_READY,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );
    sensor_data_t sensorData;
    oled_params_t *p = (oled_params_t *)pvParameters;
    TASK_PRINT("[OLED] Iniciando OLED Task\n");
    for(;;){
        if(xQueueReceive(oledQueue, &sensorData, portMAX_DELAY) == pdTRUE){
            TASK_PRINT("[OLED] Task recebeu dados!\n");
            memset(p->ssd, 0x00, p->frame_area->buffer_length);
            
            char line_1[17];
            char line_2[17];
            char line_3[17];

            //                               "               "
            snprintf(line_1, sizeof(line_1), "     %.2f°C    ", sensorData.temperatura);
            snprintf(line_2, sizeof(line_2), "    %.3fg     ", sensorData.vibracao);
            snprintf(line_3, sizeof(line_3), "     %.2f    ", sensorData.ruido);

            //Matriz 8 x 15
            char *text[] = 
            {
            //  "               "
                "  Temperatura: ",
                line_1,
                "",
                "   Vibração:   ",
                line_2,
                "",
                "     Ruído:    ",
                line_3,
            };

            int y = 0;
            for (uint i = 0; i < count_of(text); i++)
            {
                ssd1306_draw_string(p->ssd, 5, y, text[i]);
                y += 8;
            }
            render_on_display(p->ssd, p->frame_area);
            TASK_PRINT("[OLED] Dados escritos no display com sucesso!\n");
            TASK_PRINT("[OLED] Stack livre: %u\n",
            uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

void vMqttTask(void *pvParameters){
    xEventGroupWaitBits(
        sys_event_group,
        EVT_MQTT_READY,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    sensor_data_t sensorData;
    TASK_PRINT("[MQTT] Iniciando MQTT Task\n");

    for(;;){
        if(xQueueReceive(mqttQueue, &sensorData, portMAX_DELAY) == pdTRUE){
            TASK_PRINT("[MQTT] Task recebeu dados!\n");

            static char payload[256];
            snprintf(payload, sizeof(payload),
                     "{\"Contador\": %d, \"Temperatura\": %.2f C, \"Vibracao\": %.4fg, \"Ruido\": %.2f}",
                     sensorData.contador,
                     sensorData.temperatura,
                     sensorData.vibracao,
                     sensorData.ruido
            );

            if (mqtt_is_connected())
            {
                uint8_t ret = mqtt_publish_data(
                    MQTT_TOPIC_PUBLISH,
                    (uint8_t *)payload,
                    strlen(payload)
                );
                TASK_PRINT("[MQTT] Dados publicados no broker com sucesso!\n");

                if (ret == MQTT_OK)
                    TASK_PRINT("[MQTT] Publish OK\n");
                else if (ret == MQTT_ERR_BUSY)
                    TASK_PRINT("[MQTT] MQTT ocupado, envio ignorado\n");
                else
                    TASK_PRINT("[MQTT] Erro publish: %d\n", ret);
                xEventGroupClearBits(fault_event_group, FAULT_MQTT);
            }
            else
            {
                TASK_PRINT("[MQTT] Não conectado ao broker, dados não publicados.\n");
                xEventGroupSetBits(fault_event_group, FAULT_MQTT);
                vTaskSuspend(NULL);
            }

            TASK_PRINT("[MQTT] Stack livre: %u\n",
            uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

#ifdef  SD_CARD
void vSDTask(){
    xEventGroupWaitBits(
        sys_event_group,
        EVT_MQTT_READY,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    char buffer[256];
    char buffer_json[256];
    sensor_data_t sensorData;

    for(;;){
        if (xQueueReceive(sdQueue, &sensorData, portMAX_DELAY) == pdTRUE){
            bool sd_ok = true;
            TASK_PRINT("[SD] Dados recebidos!\n");

            TASK_PRINT("[SD] Contador: %d, Temp: %.2fC, Vib: %.5fg, Ruido: %.2f\n", 
                sensorData.contador, sensorData.temperatura, sensorData.vibracao, sensorData.ruido);

            // TXT
            snprintf(buffer, sizeof(buffer),
                "Ciclo %d | Temp=%.2fC | Vib=%.5fg\n | Ruido=%.2f| Timestamp: %s\n",
                sensorData.contador, sensorData.temperatura, sensorData.vibracao, sensorData.ruido);
            if(sd_card_write_text("log/dados.txt", buffer) != 0){
                TASK_PRINT("[SD] Erro ao escrever no arquivo dados.txt\n");
                sd_ok = false;
            }

            // CSV
            snprintf(buffer, sizeof(buffer),
                "%d,%.2f,%.5f,%.2f,%s\n",
                sensorData.contador, sensorData.temperatura, sensorData.vibracao, sensorData.ruido);
            if(sd_card_write_text("log/dados.csv", buffer) != 0){
                TASK_PRINT("[SD] Erro ao escrever no arquivo dados.csv\n");
                sd_ok = false;
            }

            // JSON
            format_data_to_json(&sensorData, buffer_json, sizeof(buffer_json));
            if(sd_card_append_json("log/dados.json", buffer_json) != 0){
                TASK_PRINT("[SD] Erro ao escrever no arquivo dados.json\n");
                sd_ok = false;
            }

            if (sd_ok){
                TASK_PRINT("[SD] Dados escritos no SD Card com sucesso!\n");
                xEventGroupClearBits(fault_event_group, FAULT_SD);
            }
            else{
                TASK_PRINT("[SD] Ocorreu um erro ao escrever no SD Card.\n");
                xEventGroupSetBits(fault_event_group, FAULT_SD);
                vTaskSuspend(NULL);
            }

            vTaskDelay(pdMS_TO_TICKS(10));
            TASK_PRINT("[SD] Stack livre: %u\n",
            uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

// Gera JSON simples
void format_data_to_json(const sensor_data_t *data, char *buffer, int size) {

    snprintf(buffer, size,
        "{"
        "\"contador\": %d, "
        "\"temperatura\": %.2f, "
        "\"vibracao\": %.5f, "
        "\"ruido\": %.2f"
        "}\n",
        data->contador,
        data->temperatura,
        data->vibracao,
        data->ruido
    );
}
#endif

void vWifiMonitorTask(void *pv) {
    xEventGroupWaitBits(
        sys_event_group,
        EVT_MQTT_READY,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    for (;;) {
        if (!cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA)) {
            xEventGroupSetBits(fault_event_group, FAULT_WIFI);
            vTaskSuspend(NULL);
        } else {
            xEventGroupClearBits(fault_event_group, FAULT_WIFI);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vRecoveryTask(void *pv)
{
    const TickType_t wifi_retry_delay = pdMS_TO_TICKS(10000); // 10s
    const TickType_t mqtt_retry_delay = pdMS_TO_TICKS(5000);  // 5s
    TickType_t last_wifi_attempt = 0;
    TickType_t last_mqtt_attempt = 0;

    int fail_count = 0;
    TickType_t first_fail_tick = 0;

    for (;;)
    {
        EventBits_t faults = xEventGroupGetBits(fault_event_group);
        TickType_t now = xTaskGetTickCount();

        bool wifi_failed = false;
        bool mqtt_failed = false;

        // 🔁 RECUPERAÇÃO BMP280
        if (faults & FAULT_BMP280)
        {
            bmp280_init(&bmp280_dev, I2C_PORT, BMP280_ADDR_LOW);
        }

        // 🔁 RECUPERAÇÃO SD
        if (faults & FAULT_SD)
        {
            TASK_PRINT("[RECOVERY] Tentando recuperar SD...\n");

            if (sd_init())
            {
                TASK_PRINT("[RECOVERY] SD recuperado!\n");
                xEventGroupClearBits(fault_event_group, FAULT_SD);
                vTaskResume(sdTaskHandle);
            }
            else
            {
                TASK_PRINT("[RECOVERY] SD ainda indisponível\n");
            }
        }

        // 🔁 RECUPERAÇÃO WIFI
        if ((faults & FAULT_WIFI) && (now - last_wifi_attempt > wifi_retry_delay))
        {
            last_wifi_attempt = now;
            TASK_PRINT("[RECOVERY] Tentando reconectar Wi-Fi...\n");

            if (!wifi_init())
            {
                TASK_PRINT("[RECOVERY] Wi-Fi reconectado!\n");
                xEventGroupClearBits(fault_event_group, FAULT_WIFI);
                xEventGroupSetBits(recovery_event_group, RECOVER_MQTT);
            }
            else
            {
                TASK_PRINT("[RECOVERY] Falha ao reconectar Wi-Fi.\n");
                wifi_failed = true;
            }
        }

        // 🔁 RECUPERAÇÃO MQTT
        if ((faults & FAULT_MQTT || 
             (xEventGroupGetBits(recovery_event_group) & RECOVER_MQTT)) &&
             !(faults & FAULT_WIFI) &&           
             (now - last_mqtt_attempt > mqtt_retry_delay))
        {
            last_mqtt_attempt = now;
            TASK_PRINT("[RECOVERY] Tentando reconectar MQTT...\n");

            mqtt_start_client(CLIENT_ID, CLIENT_USER, CLIENT_PASS);

            TickType_t start = xTaskGetTickCount();
            while (!mqtt_is_connected())
            {
                if (xTaskGetTickCount() - start > pdMS_TO_TICKS(10000))
                {
                    TASK_PRINT("[RECOVERY] Timeout ao conectar MQTT\n");
                    mqtt_failed = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }

            if (mqtt_is_connected())
            {
                TASK_PRINT("[RECOVERY] MQTT reconectado!\n");
                xEventGroupClearBits(fault_event_group, FAULT_MQTT);
                xEventGroupClearBits(recovery_event_group, RECOVER_MQTT);
                xEventGroupSetBits(sys_event_group, EVT_MQTT_READY);
                vTaskResume(mqttTaskHandle);
            }
        }

        // 🔁 Contagem de falhas críticas
        if (wifi_failed || mqtt_failed)
        {
            if (fail_count == 0)
                first_fail_tick = now;

            fail_count++;

            // Reinicia a contagem se a janela passou
            if (now - first_fail_tick > pdMS_TO_TICKS(RECOVERY_FAIL_WINDOW_MS))
            {
                fail_count = 1;
                first_fail_tick = now;
            }

            TASK_PRINT("[RECOVERY] Falha crítica #%d detectada.\n", fail_count);

            if (fail_count >= RECOVERY_MAX_FAILS)
            {
                TASK_PRINT("[RECOVERY] Muitas falhas consecutivas! Suspender tasks e reiniciar...\n");

                // Suspende todas as tasks exceto a SystemStateTask
                TaskStatus_t *taskStatusArray;
                UBaseType_t numTasks = uxTaskGetNumberOfTasks();
                taskStatusArray = pvPortMalloc(numTasks * sizeof(TaskStatus_t));

                if (taskStatusArray != NULL)
                {
                    numTasks = uxTaskGetSystemState(taskStatusArray, numTasks, NULL);

                    for (UBaseType_t i = 0; i < numTasks; i++)
                    {
                        if (strcmp(taskStatusArray[i].pcTaskName, "System State Task") != 0 &&
                            strcmp(taskStatusArray[i].pcTaskName, "Recovery Task") != 0)
                        {
                            vTaskSuspend(taskStatusArray[i].xHandle);
                        }
                    }

                    vPortFree(taskStatusArray);
                }

                TASK_PRINT("[RECOVERY] Tasks suspensas. Reiniciando sistema...\n");
                gpio_put(BUZZER_PIN, 0);

                watchdog_reboot(0, 0, 0);
            }
        }
        else
        {
            // ✅ Reset automático do contador se Wi-Fi e MQTT estiverem OK
            if (fail_count > 0)
            {
                TASK_PRINT("[RECOVERY] Wi-Fi e MQTT OK, resetando contador de falhas.\n");
                fail_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // loop leve
    }
}

uint wifi_init()
{
    if (cyw43_arch_init())
    {
        BOOT_PRINT("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    BOOT_PRINT("Conectando ao Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        BOOT_PRINT("Falha ao conectar ao Wi-Fi\n");
        return 1;
    }
    else 
        BOOT_PRINT("Wi-Fi conectado!\n");
    return 0;
}

void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(slice, false);
}

void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms, bool free_rtos)
{
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PIN);

    // Clock padrão do RP2040: 125 MHz
    uint32_t clock = 125000000;
    uint32_t top = clock / freq_hz - 1;

    pwm_set_wrap(slice, top);
    pwm_set_chan_level(slice, channel, top / 2); // 50% duty
    pwm_set_enabled(slice, true);

    if (free_rtos) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    } else {
        sleep_ms(duration_ms);
    }

    pwm_set_enabled(slice, false);
}

static inline void rgb_off(void) {
    gpio_put(LED_VERMELHO, 0);
    gpio_put(LED_VERDE, 0);
    gpio_put(LED_AZUL, 0);
}