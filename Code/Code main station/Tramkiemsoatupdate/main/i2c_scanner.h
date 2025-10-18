// I2C Scanner để tìm địa chỉ thiết bị I2C
#pragma once

#include "esp_log.h"
#include "driver/i2c_master.h"

static const char* I2C_SCAN_TAG = "I2C_SCAN";

void i2c_scan_bus(i2c_master_bus_handle_t bus_handle) {
    ESP_LOGI(I2C_SCAN_TAG, "Scanning I2C bus...");
    
    int devices_found = 0;
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000, // 100kHz cho scan
        };
        
        i2c_master_dev_handle_t dev_handle;
        esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
        if (ret == ESP_OK) {
            // Thử probe
            ret = i2c_master_probe(bus_handle, addr, 100);
            if (ret == ESP_OK) {
                ESP_LOGI(I2C_SCAN_TAG, "Device found at address 0x%02X", addr);
                devices_found++;
            }
            i2c_master_bus_rm_device(dev_handle);
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(I2C_SCAN_TAG, "No I2C devices found!");
    } else {
        ESP_LOGI(I2C_SCAN_TAG, "Found %d device(s)", devices_found);
    }
}
