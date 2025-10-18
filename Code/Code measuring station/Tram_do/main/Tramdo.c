/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h> // For memcpy and memset
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h" // Include for Watchdog Timer

// Pin configuration based on the provided diagram
#define LORA_SCK_PIN    18
#define LORA_MISO_PIN   19
#define LORA_MOSI_PIN   23
#define LORA_CS_PIN     5
#define LORA_RST_PIN    14
#define LORA_DIO0_PIN   2

#define SENSOR_TRIG_PIN 25
#define SENSOR_ECHO_PIN 26
#define SENSOR_PWR_PIN  27

#define LED_RED_PIN     32
#define LED_GREEN_PIN   33

#define LORA_SPI_HOST   SPI2_HOST

// Watchdog timeout period in seconds
#define WDT_TIMEOUT_S 60

// LoRa Registers and Modes
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
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_PAYLOAD_LENGTH       0x22
#define REG_VERSION              0x42

#define MODE_LORA                0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03

// LoRa Addresses
#define LORA_CONTROL_STATION_ADDRESS      0x01
#define LORA_MEASUREMENT_STATION_ADDRESS  0x02

static const char *TAG = "MAIN";
spi_device_handle_t spi_handle_lora;

// LoRa Basic Functions

void lora_reset(void) {
    gpio_set_level(LORA_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(LORA_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}


void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_data[2] = {reg | 0x80, val};
    spi_transaction_t t = {.length = 16, .tx_buffer = tx_data};
    gpio_set_level(LORA_CS_PIN, 0);
    spi_device_polling_transmit(spi_handle_lora, &t);
    gpio_set_level(LORA_CS_PIN, 1);
}


uint8_t lora_read_reg(uint8_t reg) {
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};
    uint8_t rx_data[2];
    spi_transaction_t t = {.length = 16, .tx_buffer = tx_data, .rx_buffer = rx_data};
    gpio_set_level(LORA_CS_PIN, 0);
    spi_device_polling_transmit(spi_handle_lora, &t);
    gpio_set_level(LORA_CS_PIN, 1);
    return rx_data[1];
}


void lora_enable_crc(void) {
    lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
    ESP_LOGI(TAG, "CRC enabled: 0x%02X", lora_read_reg(REG_MODEM_CONFIG_2));
}


void lora_set_tx_power(int level) {
    if (level < 2) level = 2;
    else if (level > 17) level = 17;
    lora_write_reg(REG_PA_CONFIG, 0x80 | (level - 2));
    ESP_LOGI(TAG, "TX Power set to %d dBm: REG_PA_CONFIG=0x%02X", level, lora_read_reg(REG_PA_CONFIG));
}

bool lora_send_packet(uint8_t recipient, uint8_t sender, uint8_t *payload, uint8_t len) {
    ESP_LOGI(TAG, "Preparing to send packet: To=0x%02X, From=0x%02X, Len=%d", recipient, sender, len);
    

    lora_write_reg(REG_OP_MODE, MODE_STDBY | MODE_LORA);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);
    lora_write_reg(REG_PAYLOAD_LENGTH, 0);
    lora_write_reg(REG_FIFO, recipient);
    lora_write_reg(REG_FIFO, sender);
    lora_write_reg(REG_FIFO, len);
    
    for (int i = 0; i < len; i++) {
        lora_write_reg(REG_FIFO, payload[i]);
    }
    

    uint8_t total_len = len + 3;
    lora_write_reg(REG_PAYLOAD_LENGTH, total_len);
    
    ESP_LOGI(TAG, "Packet written to FIFO. Total length: %d", total_len);
    

    lora_write_reg(REG_OP_MODE, MODE_TX | MODE_LORA);
    ESP_LOGI(TAG, "TX Mode - IRQ Flags: 0x%02X", lora_read_reg(REG_IRQ_FLAGS));
    
    int64_t tx_start_time = esp_timer_get_time();
    while ((lora_read_reg(REG_IRQ_FLAGS) & 0x08) == 0) {
        if ((esp_timer_get_time() - tx_start_time) > 5000000) {
            ESP_LOGE(TAG, "LoRa TX timeout! Resetting LoRa module.");
            
            lora_reset();
            
            lora_write_reg(REG_OP_MODE, MODE_SLEEP | MODE_LORA);
            lora_write_reg(REG_OP_MODE, MODE_STDBY | MODE_LORA);
            
            lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
            lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);

            ESP_LOGI(TAG, "LoRa module has been reset and is in standby.");
            
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    lora_write_reg(REG_IRQ_FLAGS, 0x08);
    ESP_LOGI(TAG, "Packet sent successfully!");
    lora_write_reg(REG_OP_MODE, MODE_STDBY | MODE_LORA);
    return true;
}


// Ultrasonic Sensor Functions
void sort_floats(float *arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

float measure_distance(void) {
    const int NUM_SAMPLES = 5;
    float samples[NUM_SAMPLES];
    int valid_samples_count = 0;


    gpio_set_level(SENSOR_PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 0; i < NUM_SAMPLES; i++) {

        gpio_set_level(SENSOR_TRIG_PIN, 0);
        esp_rom_delay_us(2);
        gpio_set_level(SENSOR_TRIG_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level(SENSOR_TRIG_PIN, 0);

        int64_t wait_start_time = esp_timer_get_time();
        while (gpio_get_level(SENSOR_ECHO_PIN) == 0) {
            if ((esp_timer_get_time() - wait_start_time) > 30000) {
                samples[i] = -1.0f;
                ESP_LOGW(TAG, "Sample %d: Timeout waiting for echo pulse to start.", i);
                goto next_sample;
            }
        }

        int64_t start_time = esp_timer_get_time();
        while (gpio_get_level(SENSOR_ECHO_PIN) == 1) {
            if ((esp_timer_get_time() - start_time) > 30000) {
                samples[i] = -1.0f;
                ESP_LOGW(TAG, "Sample %d: Timeout waiting for echo pulse to end.", i);
                goto next_sample;
            }
        }
        int64_t end_time = esp_timer_get_time();
        uint32_t duration = end_time - start_time;
        samples[i] = (duration * 0.0343) / 2.0;
        
        if (samples[i] > 0 && samples[i] < 450) {
            valid_samples_count++;
        }

    next_sample:
        vTaskDelay(pdMS_TO_TICKS(50));
    }


    gpio_set_level(SENSOR_PWR_PIN, 0);


    if (valid_samples_count < 3) {
        ESP_LOGE(TAG, "Measurement failed: Not enough valid samples (%d/%d).", valid_samples_count, NUM_SAMPLES);
        return -1.0f;
    }

    sort_floats(samples, NUM_SAMPLES);
    ESP_LOGI(TAG, "Samples: %.1f, %.1f, %.1f, %.1f, %.1f -> Median: %.1f", samples[0], samples[1], samples[2], samples[3], samples[4], samples[NUM_SAMPLES / 2]);
    return samples[NUM_SAMPLES / 2];
}

// Main Task

void measurement_task(void *pvParameters) {

    ESP_LOGI(TAG, "Measurement task started. Subscribing to Watchdog Timer.");
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "Will measure and send data every 7 seconds.");

    while (1) {
        int64_t cycle_start_time = esp_timer_get_time();

        ESP_LOGI(TAG, "--- Starting new measurement cycle (WDT reset) ---");
        esp_task_wdt_reset();

        float distance = measure_distance();
        float water_level = 100.0f - distance;

        if (distance > 0 && distance < 450) {
            ESP_LOGI(TAG, "Muc nuoc: %.2f cm. Nhay den GREEN mot lan.", water_level);
            gpio_set_level(LED_GREEN_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GREEN_PIN, 0);
            char payload[32];
            int payload_len = snprintf(payload, sizeof(payload), "Muc nuoc: %.2f", water_level);
            if (lora_send_packet(LORA_CONTROL_STATION_ADDRESS, LORA_MEASUREMENT_STATION_ADDRESS, (uint8_t *)payload, payload_len)) {
                ESP_LOGI(TAG, "LoRa sent successfully. Blinking RED once.");
                gpio_set_level(LED_RED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_RED_PIN, 0);
            } else {
                ESP_LOGE(TAG, "LoRa send failed. Blinking RED twice.");
                gpio_set_level(LED_RED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(250));
                gpio_set_level(LED_RED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(250));
                gpio_set_level(LED_RED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(250));
                gpio_set_level(LED_RED_PIN, 0);
            }
        } else {
            ESP_LOGW(TAG, "Measurement failed. Blinking GREEN twice.");
            gpio_set_level(LED_GREEN_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(LED_GREEN_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(LED_GREEN_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(LED_GREEN_PIN, 0);
        }
        int64_t cycle_end_time = esp_timer_get_time();
        int64_t cycle_duration_ms = (cycle_end_time - cycle_start_time) / 1000;
        int delay_ms = 7000 - cycle_duration_ms;
        if (delay_ms < 0) { delay_ms = 0; }
        ESP_LOGI(TAG, "Cycle finished. Waiting for %d ms...", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Measurement Station Initializing");

    ESP_LOGI(TAG, "Reconfiguring Task Watchdog Timer...");
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,
        .trigger_panic = true,
        .idle_core_mask = (1 << 0) | (1 << 1),
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));

    ESP_LOGI(TAG, "Initializing GPIO pins...");
    gpio_set_direction(LORA_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SENSOR_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SENSOR_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SENSOR_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(SENSOR_PWR_PIN, 0);
    gpio_set_level(LED_RED_PIN, 0);
    gpio_set_level(LED_GREEN_PIN, 0);

    ESP_LOGI(TAG, "GPIO initialization complete.");



    ESP_LOGI(TAG, "Initializing SPI for LoRa module...");
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO_PIN,
        .mosi_io_num = LORA_MOSI_PIN,
        .sclk_io_num = LORA_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 9000000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 7,
    };

    esp_err_t ret = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);


    ret = spi_bus_add_device(LORA_SPI_HOST, &devcfg, &spi_handle_lora);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "SPI initialization complete.");

    ESP_LOGI(TAG, "Initializing LoRa module...");
    lora_reset();

    lora_write_reg(0x01, 0x00);
    lora_write_reg(0x01, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t version = lora_read_reg(0x42);
    if (version != 0x12) {
        ESP_LOGE(TAG, "LoRa chip not detected! Version: 0x%02X", version);
        while(1) { vTaskDelay(1); }
    }
    ESP_LOGI(TAG, "LoRa version: 0x%02X", version);

    lora_write_reg(0x39, 0x34);
    lora_write_reg(0x20, 0x00);
    lora_write_reg(0x21, 0x08);
    lora_write_reg(0x06, 0x6C);
    lora_write_reg(0x07, 0x80);
    lora_write_reg(0x08, 0x00);
    ESP_LOGI(TAG, "LoRa Config Check:");
    ESP_LOGI(TAG, "  OpMode: 0x%02X", lora_read_reg(0x01));
    ESP_LOGI(TAG, "  ModemCfg1: 0x%02X", lora_read_reg(0x1D));
    ESP_LOGI(TAG, "  ModemCfg2: 0x%02X", lora_read_reg(0x1E));
    ESP_LOGI(TAG, "  SyncWord: 0x%02X", lora_read_reg(0x39));
    ESP_LOGI(TAG, "  Freq MSB: 0x%02X", lora_read_reg(0x06));
    ESP_LOGI(TAG, "  Freq MID: 0x%02X", lora_read_reg(0x07));
    ESP_LOGI(TAG, "  Freq LSB: 0x%02X", lora_read_reg(0x08));


    lora_write_reg(0x0E, 0);
    lora_write_reg(0x0F, 0);

    lora_write_reg(0x0C, lora_read_reg(0x0C) | 0x03);
    lora_write_reg(0x26, 0x04);
    lora_write_reg(0x1D, 0x72);
    lora_write_reg(0x1E, 0x74);

    lora_enable_crc();
    lora_set_tx_power(14);
    lora_write_reg(0x01, 0x81);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "LoRa Config Check:");
    ESP_LOGI(TAG, "  OpMode: 0x%02X", lora_read_reg(0x01));
    ESP_LOGI(TAG, "  ModemCfg1: 0x%02X", lora_read_reg(0x1D));
    ESP_LOGI(TAG, "  ModemCfg2: 0x%02X (CRC enabled)", lora_read_reg(0x1E));
    ESP_LOGI(TAG, "  SyncWord: 0x%02X", lora_read_reg(0x39));
    ESP_LOGI(TAG, "  Freq MSB: 0x%02X", lora_read_reg(0x06));
    ESP_LOGI(TAG, "  Freq MID: 0x%02X", lora_read_reg(0x07));
    ESP_LOGI(TAG, "  Freq LSB: 0x%02X", lora_read_reg(0x08));
    ESP_LOGI(TAG, "  PA_CONFIG: 0x%02X (TX Power: 14dBm)", lora_read_reg(REG_PA_CONFIG));

    ESP_LOGI(TAG, "LoRa module initialized and in standby mode.");
    ESP_LOGI(TAG, "Initialization Complete");
    ESP_LOGI(TAG, "Creating measurement task.");
    xTaskCreate(measurement_task, "measurement_task", 4096, NULL, 5, NULL);

    printf("Hello from Measurement Station!\n");
}
