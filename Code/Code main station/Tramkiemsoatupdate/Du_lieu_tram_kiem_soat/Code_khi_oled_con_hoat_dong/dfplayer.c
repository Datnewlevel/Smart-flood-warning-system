#include "dfplayer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DFPLAYER";
static const int UART_BUF_SIZE = 1024;

static void dfplayer_send_cmd(uint8_t cmd, uint16_t arg) {
    uint8_t buffer[10];
    buffer[0] = 0x7E; // Start
    buffer[1] = 0xFF; // Version
    buffer[2] = 0x06; // Length
    buffer[3] = cmd; // Command
    buffer[4] = 0x00; // Feedback (0=no, 1=yes)
    buffer[5] = (arg >> 8) & 0xFF;
    buffer[6] = arg & 0xFF;

    // Checksum
    int16_t checksum = -(buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6]);
    buffer[7] = (checksum >> 8) & 0xFF;
    buffer[8] = checksum & 0xFF;
    buffer[9] = 0xEF; // End

    uart_write_bytes(UART_NUM, (const char*)buffer, 10);
}

esp_err_t dfplayer_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, DFPLAYER_TXD, DFPLAYER_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    vTaskDelay(pdMS_TO_TICKS(500)); // DFPlayer cần thời gian khởi động
    ESP_LOGI(TAG, "DFPlayer initialized");
    return ESP_OK;
}

void dfplayer_set_volume(uint8_t volume) {
    if (volume > 30) volume = 30;
    dfplayer_send_cmd(0x06, volume);
}

void dfplayer_play(uint8_t track) {
    dfplayer_send_cmd(0x03, track);
}

void dfplayer_deinit(void) {
    uart_driver_delete(UART_NUM);
}
