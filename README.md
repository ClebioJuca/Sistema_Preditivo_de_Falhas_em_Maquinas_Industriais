# Sistema Preditivo de Falhas em Máquinas Industriais

## Descrição
Este projeto propõe um sistema embarcado de monitoramento remoto para máquinas industriais utilizando a Raspberry Pi Pico W (RP2040). O sistema coleta dados de temperatura, vibração e ruído, processa essas informações localmente e as envia via protocolo MQTT para um servidor remoto.

O objetivo é permitir análise preditiva de falhas, reduzindo tempo de inatividade, aumentando a segurança operacional e melhorando a eficiência do processo industrial.

O firmware é baseado em FreeRTOS, com arquitetura modular e multitarefa.

## Requisitos
- **Hardware:**
  - Raspberry Pi Pico W (RP2040)
  - Sensor de temperatura BMP280
  - Sensor de vibração MPU6050
  - Display OLED SSD1306
  - Microfone Eletreto ou Módulo equivalente
  - Leitor de Cartão SD (Opcional)
  - LED RGB (debug visual de status)

- **Software:**
  - Raspberry Pi Pico SDK
  - Protocolo MQTT
  - CMake
  - GCC ARM
  - FreeRTOS Kernel
  - Broker MQTT (Mosquitto ou HiveMQ Cloud)

## Instalação
1. Clone este repositório:
   ```sh
   git clone https://github.com/ClebioJuca/Sistema_Preditivo_de_Falhas_em_Maquinas_Industriais
   ```
2. Acesse a pasta do projeto:
   ```sh
   cd Sistema_Preditivo_de_Falhas_em_Maquinas_Industriais
   ```
3. Compile o firmware:
   ```sh
   mkdir build && cd build
   cmake ..
   make
   ```
4. Coloque a Pico W em modo BOOTSEL.
5. Envie o arquivo .uf2 para o Raspberry Pi Pico W.

## Funcionamento

1. Inicialização do Wi-Fi.
2. Conexão ao broker MQTT.
3. Liberação das tasks de aquisição.
4. Leitura periódica:
- Temperatura
- Vibração (aceleração nos eixos)
- Ruído (RMS do sinal do microfone)
5. Exibição no display OLED.
6. Publicação via MQTT.
7. Salvamento opcional no cartão SD.

## Licença
Este projeto está licenciado sob a **GPLv3**. Consulte o arquivo LICENSE para mais informações.

## Contato
Caso tenha dúvidas ou sugestões, entre em contato pelo [GitHub](https://github.com/ClebioJuca/Sistema_Preditivo_de_Falhas_em_Maquinas_Industriais).
