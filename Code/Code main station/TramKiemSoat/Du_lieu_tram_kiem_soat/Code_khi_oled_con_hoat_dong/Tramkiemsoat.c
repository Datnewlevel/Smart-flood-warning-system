/*
 * Main application file for the ESP32-S3 LoRa Control Station.
 *
 * This file contains the logic for receiving LoRa data,
 * and providing audible and visual alerts.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "dfplayer.h"
#include "esp_task_wdt.h"
#include "ssd1306.h"

static const char *TAG = "CONTROL_STATION";


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

    // Display on OLED
    ssd1306_clear();  // Xóa buffer trước
    
    char line1[16];  // Giảm độ dài buffer
    snprintf(line1, sizeof(line1), "Muc nuoc:%.1fcm", distance);  // Rút gọn text
    ssd1306_print_str(5, 10, line1, false);

    if (distance < 30.0) {
        ssd1306_print_str(28, 37, "NGUY HIEM!", false);
    } else if (distance < 50.0) {
        ssd1306_print_str(28, 37, "CANH BAO!", false);
    } else {
        ssd1306_print_str(28, 37, "BINH THUONG", false);
    }
    ssd1306_display();

    // Determine warning level and take action
    if (distance < 30.0) {
        ESP_LOGE(TAG, "Water level: DANGER (%.2f cm)", distance);
        gpio_set_level(LED_RED, 1);
        dfplayer_play(3); // Play danger track
    } else if (distance < 50.0) {
        ESP_LOGW(TAG, "Water level: WARNING (%.2f cm)", distance);
        gpio_set_level(LED_YELLOW, 1);
        dfplayer_play(2); // Play warning track
    } else { // distance >= 50.0
        ESP_LOGI(TAG, "Water level: NORMAL (%.2f cm)", distance);
        gpio_set_level(LED_GREEN, 1);
        dfplayer_play(1); // Play normal track
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

                    char *dist_str = strstr(payload_buffer, "Distance: ");
                    if (dist_str) {
                        float distance = atof(dist_str + 10);
                        handle_water_level(distance);
                    } else {
                        ESP_LOGW(TAG, "Payload has unknown format.");
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

    // Initialize GPIOs for LEDs and Buzzer
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_YELLOW, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_YELLOW, 0);
    gpio_set_level(LED_RED, 0);
    gpio_set_level(BUZZER_PIN, 0);

    init_ssd1306();
    ssd1306_clear();  // Xóa màn hình trước
    ssd1306_print_str(12, 27, "Tram Kiem Soat", false);
    ssd1306_display();
   

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

    // Safely handle WDT for the main task
    if (esp_task_wdt_status(NULL) == ESP_OK) {
        ESP_LOGI(TAG, "Main task is subscribed to WDT. Unsubscribing.");
        esp_task_wdt_delete(NULL);
    } else {
        ESP_LOGI(TAG, "Main task is not subscribed to WDT. No action needed.");
    }

    // This loop is kept to prevent app_main from returning, which is good practice.
}