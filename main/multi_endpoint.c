// multi_endpoint.c
#include "multi_endpoint.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MULTI_ENDPOINT";

typedef struct {
    uint8_t endpoint;
    gpio_num_t gpio_pin;
    bool state;
} endpoint_config_t;

static endpoint_config_t endpoints[] = {
    {HA_ESP_LIGHT_ENDPOINT, GPIO_NUM_3, false},  // Первый эндпоинт на GPIO3
    {HA_ESP_LIGHT_ENDPOINT + 1, GPIO_NUM_4, false}  // Второй эндпоинт на GPIO4
};

esp_err_t multi_endpoint_init(void) {
    for (int i = 0; i < sizeof(endpoints)/sizeof(endpoint_config_t); i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << endpoints[i].gpio_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(endpoints[i].gpio_pin, endpoints[i].state);
        ESP_LOGI(TAG, "GPIO %d initialized to %s for endpoint %d", 
                endpoints[i].gpio_pin, 
                endpoints[i].state ? "ON" : "OFF",
                endpoints[i].endpoint);
    }
    return ESP_OK;
}


esp_err_t multi_endpoint_set_state(uint8_t endpoint, bool state) {
    for (int i = 0; i < sizeof(endpoints)/sizeof(endpoint_config_t); i++) {
        if (endpoints[i].endpoint == endpoint) {
            gpio_set_level(endpoints[i].gpio_pin, state);
            endpoints[i].state = state;
            ESP_LOGI(TAG, "GPIO %d set to %s for endpoint %d", 
                    endpoints[i].gpio_pin, 
                    state ? "ON" : "OFF",
                    endpoint);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Endpoint %d not found", endpoint);
    return ESP_ERR_NOT_FOUND;
}