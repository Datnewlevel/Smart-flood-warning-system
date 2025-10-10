/*
 * Main application file for the ESP32-S3 LoRa Control Station.
 *
 * This file contains the logic for:
 * - Initializing WiFi and connecting to the network.
 * - Initializing and connecting to an MQTT broker.
 * - Receiving LoRa data packets with water level information.
 * - Providing audible (buzzer, DFPlayer) and visual (LEDs, OLED) alerts.
 * - Publishing received data to the MQTT broker.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "dfplayer.h"
#include "esp_task_wdt.h"
#include "ssd1306.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "mqtt_client.h"

// Khai báo dữ liệu chứng chỉ được nhúng từ file PEM
extern const uint8_t hivemq_isrg_root_x1_pem_start[] asm("_binary_hivemq_isrg_root_x1_pem_start");
extern const uint8_t hivemq_isrg_root_x1_pem_end[]   asm("_binary_hivemq_isrg_root_x1_pem_end");

static const char *TAG = "CONTROL_STATION";

// --- WiFi Definitions ---
#define WIFI_SSID      "The Bao - Tang 1"
#define WIFI_PASS      "0342936197"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// --- MQTT Definitions ---
#define MQTT_BROKER_URL "mqtts://df973d5a054e4b299a614f0f9da093d2.s1.eu.hivemq.cloud"
#define MQTT_BROKER_PORT 8883
#define MQTT_USERNAME   "Dat_smart_flood"
#define MQTT_PASSWORD   "23.4.2005Dat"
#define MQTT_TOPIC      "esp32/tramkiemsoat/data"
esp_mqtt_client_handle_t mqtt_client;


// --- Pin Definitions ---
// LoRa (SPI)
#define LORA_HOST       SPI2_HOST
#define LORA_MOSI       GPIO_NUM_11
#define LORA_MISO       GPIO_NUM_13
#define LORA_SCLK       GPIO_NUM_12
#define LORA_CS         GPIO_NUM_10
#define LORA_RST        GPIO_NUM_14
#define LORA_DIO0       GPIO_NUM_2

// Warning LEDs
#define LED_GREEN       GPIO_NUM_15 // Low/Normal
#define LED_YELLOW      GPIO_NUM_16 // Medium
#define LED_RED         GPIO_NUM_4  // High

// Buzzer
#define BUZZER_PIN      GPIO_NUM_5

// --- LoRa Registers and Modes ---
#define REG_OP_MODE              0x01
#define REG_FIFO                 0x00
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

#define MODE_LORA                0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_RX_CONTINUOUS       0x05
#define IRQ_RX_DONE_MASK         0x40

// --- LoRa Packet Addresses ---
#define LORA_CONTROL_STATION_ADDRESS      0x01
#define LORA_MEASUREMENT_STATION_ADDRESS  0x02

// --- Global Variables ---
spi_device_handle_t spi_handle_lora;
static QueueHandle_t lora_rx_queue;
static int last_warning_level = 0;

// =========================================================================
// HELPER FUNCTIONS: WIFI
// =========================================================================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// =========================================================================
// HELPER FUNCTIONS: MQTT
// =========================================================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        ssd1306_clear();
        ssd1306_print_str(15, 10, "MQTT Connected", false);
        ssd1306_display();
        vTaskDelay(pdMS_TO_TICKS(2000));
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
        .broker.verification.certificate = (const char *)hivemq_isrg_root_x1_pem_start,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}


// =========================================================================
// HELPER FUNCTIONS: LORA
// =========================================================================

void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_data[2] = {reg | 0x80, val};
    spi_transaction_t t = {.length = 16, .tx_buffer = tx_data};
    gpio_set_level(LORA_CS, 0);
    spi_device_polling_transmit(spi_handle_lora, &t);
    gpio_set_level(LORA_CS, 1);
}

uint8_t lora_read_reg(uint8_t reg) {
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};
    uint8_t rx_data[2];
    spi_transaction_t t = {.length = 16, .tx_buffer = tx_data, .rx_buffer = rx_data};
    gpio_set_level(LORA_CS, 0);
    spi_device_polling_transmit(spi_handle_lora, &t);
    gpio_set_level(LORA_CS, 1);
    return rx_data[1];
}

void lora_reset() {
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void lora_set_frequency(uint64_t freq) {
    uint64_t frf = (freq << 19) / 32000000;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_enter_rx_mode() {
    lora_write_reg(REG_DIO_MAPPING_1, 0x00); // DIO0 -> RxDone
    lora_write_reg(REG_OP_MODE, MODE_RX_CONTINUOUS | MODE_LORA);
    ESP_LOGI(TAG, "LoRa in continuous receive mode.");
}

void lora_init() {
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_CS, GPIO_MODE_OUTPUT);
    lora_reset();

    spi_bus_config_t buscfg = {
        .mosi_io_num = LORA_MOSI,
        .miso_io_num = LORA_MISO,
        .sclk_io_num = LORA_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    spi_bus_initialize(LORA_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 9000000,
        .mode = 0,
        .spics_io_num = -1, // Manage CS manually
        .queue_size = 7,
    };
    spi_bus_add_device(LORA_HOST, &devcfg, &spi_handle_lora);

    lora_write_reg(REG_OP_MODE, MODE_SLEEP);
    lora_write_reg(REG_OP_MODE, MODE_LORA);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t version = lora_read_reg(REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "LoRa chip not detected! Version: 0x%02X", version);
        while(1) { vTaskDelay(1); }
    }
    ESP_LOGI(TAG, "LoRa version: 0x%02X", version);

    // Configuration from PDF guide
    lora_write_reg(0x39, 0x34);
    lora_write_reg(0x20, 0x00);
    lora_write_reg(0x21, 0x08);
    lora_set_frequency(434000000);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);

    lora_write_reg(REG_OP_MODE, MODE_STDBY | MODE_LORA);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "LoRa Config Check:");
    ESP_LOGI(TAG, "  OpMode: 0x%02X", lora_read_reg(0x01));
    ESP_LOGI(TAG, "  ModemCfg1: 0x%02X", lora_read_reg(0x1D));
    ESP_LOGI(TAG, "  ModemCfg2: 0x%02X", lora_read_reg(0x1E));
    ESP_LOGI(TAG, "  SyncWord: 0x%02X", lora_read_reg(0x39));
    ESP_LOGI(TAG, "  Freq: 0x%02X%02X%02X", lora_read_reg(0x06), lora_read_reg(0x07), lora_read_reg(0x08));

    ESP_LOGI(TAG, "LoRa initialized.");
}

static void IRAM_ATTR lora_dio0_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(lora_rx_queue, &gpio_num, NULL);
}

// =========================================================================
// MAIN LOGIC
// =========================================================================

void beep(int duration_ms) {
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(BUZZER_PIN, 0);
}

void handle_water_level(float distance) {
    // Reset LEDs
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_YELLOW, 0);
    gpio_set_level(LED_RED, 0);

    // Prepare data and level string
    int new_warning_level;
    char level_str[16];
    char line1[16];
    snprintf(line1, sizeof(line1), "Muc nuoc:%.1fcm", distance);

    if (distance < 30.0) {
        strcpy(level_str, "NGUY HIEM!");
        new_warning_level = 3;
    } else if (distance < 50.0) {
        strcpy(level_str, "CANH BAO!");
        new_warning_level = 2;
    } else {
        strcpy(level_str, "BINH THUONG");
        new_warning_level = 1;
    }

    // Display on OLED - This should always update
    ssd1306_clear();
    ssd1306_print_str(5, 10, line1, false);
    ssd1306_print_str(28, 37, level_str, false);
    ssd1306_display();

    // Publish to MQTT - This should always update
    char json_payload[100];
    snprintf(json_payload, sizeof(json_payload), "{\"distance\": %.2f, \"level\": \"%s\"}", distance, level_str);
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_payload, 0, 1, 0);
    ESP_LOGI(TAG, "Sent publish successful, msg_id=%d", msg_id);

    // Set LEDs - This should always update
    if (new_warning_level == 3) {
        ESP_LOGE(TAG, "Water level: DANGER (%.2f cm)", distance);
        gpio_set_level(LED_RED, 1);
    } else if (new_warning_level == 2) {
        ESP_LOGW(TAG, "Water level: WARNING (%.2f cm)", distance);
        gpio_set_level(LED_YELLOW, 1);
    } else { // new_warning_level == 1
        ESP_LOGI(TAG, "Water level: NORMAL (%.2f cm)", distance);
        gpio_set_level(LED_GREEN, 1);
    }

    // Play sound only if the warning level has changed
    if (new_warning_level != last_warning_level) {
        ESP_LOGI(TAG, "Warning level changed from %d to %d. Playing sound.", last_warning_level, new_warning_level);
        if (new_warning_level == 3) {
            dfplayer_play(3); // Play danger track
        } else if (new_warning_level == 2) {
            dfplayer_play(2); // Play warning track
        } else { // new_warning_level == 1
            dfplayer_play(1); // Play normal track
        }
        last_warning_level = new_warning_level; // Update the last level
    } else {
        ESP_LOGI(TAG, "Warning level unchanged (%d). Skipping sound.", last_warning_level);
    }
}

void lora_rx_task(void *pvParameters) {
    ESP_LOGI(TAG, "LoRa RX task started.");
    uint32_t io_num;

    ESP_LOGI(TAG, "DIO0 current level: %d", gpio_get_level(LORA_DIO0));
    while (1) {
        if (xQueueReceive(lora_rx_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "LoRa packet received!");
            beep(500); // Notify on any packet
            
            lora_write_reg(REG_OP_MODE, MODE_STDBY | MODE_LORA);
            
            uint8_t irq_flags = lora_read_reg(REG_IRQ_FLAGS);
            if (irq_flags & 0x20) {
                ESP_LOGE(TAG, "CRC Error detected!");
                lora_write_reg(REG_IRQ_FLAGS, 0xFF);
                lora_enter_rx_mode();
                continue;
            }
            
            int packet_len = lora_read_reg(REG_RX_NB_BYTES);
            uint8_t fifo_rx_addr = lora_read_reg(REG_FIFO_RX_BASE_ADDR);
            lora_write_reg(REG_FIFO_ADDR_PTR, fifo_rx_addr);

            if (packet_len < 4) { 
                ESP_LOGW(TAG, "Received packet is too short (len=%d). Discarding.", packet_len);
            } else {
                uint8_t recipient = lora_read_reg(REG_FIFO);
                uint8_t sender = lora_read_reg(REG_FIFO);
                (void)sender;
                uint8_t payload_len = lora_read_reg(REG_FIFO);

                if (recipient != LORA_CONTROL_STATION_ADDRESS) {
                    ESP_LOGW(TAG, "Packet for another station (0x%02X).", recipient);
                } else if (payload_len != (packet_len - 3)) {
                    ESP_LOGE(TAG, "Packet length mismatch! Header: %d, Actual: %d.", payload_len, packet_len - 3);
                } else {
                    char payload_buffer[256] = {0};
                    for (int i = 0; i < payload_len; i++) {
                        payload_buffer[i] = lora_read_reg(REG_FIFO);
                    }
                    ESP_LOGI(TAG, "Received Payload: %s", payload_buffer);

                    char *payload_start = strstr(payload_buffer, "Distance: ");
                    if (payload_start) {
                        float distance;
                    // Cố gắng đọc một số thực từ chuỗi, nếu thành công (trả về 1) thì mới xử lý
                    if (sscanf(payload_start, "Distance: %f", &distance) == 1) {
                        handle_water_level(distance);
                    } else {
                        ESP_LOGW(TAG, "Failed to parse distance from payload: %s", payload_buffer);
                    }
                    } else {
                        ESP_LOGW(TAG, "Payload format unknown: %s", payload_buffer);
                    } 
                }
            }

            lora_write_reg(REG_IRQ_FLAGS, 0xFF); // Clear all IRQ flags
            lora_enter_rx_mode();
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "--- Starting Application: LoRa Control Station ---");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize GPIOs for LEDs and Buzzer
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_YELLOW, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_YELLOW, 0);
    gpio_set_level(LED_RED, 0);
    gpio_set_level(BUZZER_PIN, 0);

    // Initialize OLED
    init_ssd1306();
    ssd1306_clear();
    ssd1306_print_str(12, 27, "Tram Kiem Soat", false);
    ssd1306_display();
    vTaskDelay(pdMS_TO_TICKS(1000));

    
    // Connect to WiFi
    ssd1306_clear();
    ssd1306_print_str(10, 10, "Connecting WiFi...", false);
    ssd1306_display();
    wifi_init_sta();
    
    // Connect to MQTT
    ssd1306_clear();
    ssd1306_print_str(10, 10, "Connecting MQTT...", false);
    ssd1306_display();
    mqtt_app_start();
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait a bit for connection
    

    // Startup blink sequence
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_GREEN, 1);
        gpio_set_level(LED_YELLOW, 1);
        gpio_set_level(LED_RED, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_level(LED_GREEN, 0);
        gpio_set_level(LED_YELLOW, 0);
        gpio_set_level(LED_RED, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    // Initialize peripherals
    ESP_ERROR_CHECK(dfplayer_init());
    lora_init();

    // Initial sound setup
    dfplayer_set_volume(25); // Set volume to 25 (max 30)
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    beep(1000); // LoRa init success beep

    // Configure interrupt for LoRa
    lora_rx_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);
    gpio_set_intr_type(LORA_DIO0, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO0, lora_dio0_isr_handler, (void*) LORA_DIO0);

    // Create LoRa task
    xTaskCreate(lora_rx_task, "lora_rx_task", 4096, NULL, 10, NULL);

    // Put LoRa into receive mode
    lora_enter_rx_mode();

    ESP_LOGI(TAG, "Initialization complete. Waiting for LoRa packets...");
    ssd1306_clear();
    ssd1306_print_str(5, 10, "Cho tin hieu...", false);
    ssd1306_display();

    // Safely handle WDT for the main task
    if (esp_task_wdt_status(NULL) == ESP_OK) {
        ESP_LOGI(TAG, "Main task is subscribed to WDT. Unsubscribing.");
        esp_task_wdt_delete(NULL);
    } else {
        ESP_LOGI(TAG, "Main task is not subscribed to WDT. No action needed.");
    }
}
