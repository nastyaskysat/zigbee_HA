// multi_endpoint.h
#pragma once

#include "esp_err.h"
#include "ha/esp_zigbee_ha_standard.h"

#define HA_ESP_LIGHT_ENDPOINT 10  // Базовый эндпоинт

esp_err_t multi_endpoint_init(void);
esp_err_t multi_endpoint_set_state(uint8_t endpoint, bool state);