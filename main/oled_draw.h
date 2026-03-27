#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

void oledDrawAttachDevice(i2c_master_dev_handle_t dev);
esp_err_t oledDrawSendCmd(uint8_t cmd);
esp_err_t oledDrawClear(void);
esp_err_t oledDrawLine(uint8_t page, const char *text);
esp_err_t oledDrawLineBig(uint8_t pageStart, const char *text);
esp_err_t oledDrawLineHuge(uint8_t pageStart, const char *text);
