#include "driver/gpio.h"
#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_GPIO_SWITCH";
#define ESP_MANUFACTURER_NAME "ESP_CUSTOM"
#define ESP_MODEL_IDENTIFIER "ESP_LIGHT"
#define GPIO_OUTPUT_PIN_1 3  // GPIO для первого эндпоинта
#define GPIO_OUTPUT_PIN_2 4  // GPIO для второго эндпоинта

// Объявление функций
static esp_err_t gpio_driver_init(void);
static void gpio_driver_set_state(uint8_t endpoint, bool state);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static void esp_zb_task(void *pvParameters);

// Инициализация GPIO
static esp_err_t gpio_driver_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_PIN_1) | (1ULL << GPIO_OUTPUT_PIN_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed");
        return ret;
    }
    gpio_set_level(GPIO_OUTPUT_PIN_1, false);
    gpio_set_level(GPIO_OUTPUT_PIN_2, false);
    ESP_LOGI(TAG, "GPIOs initialized");
    return ESP_OK;
}

// Установка состояния GPIO для конкретного эндпоинта
static void gpio_driver_set_state(uint8_t endpoint, bool state)
{
    gpio_num_t pin = (endpoint == HA_ESP_LIGHT_ENDPOINT) ? GPIO_OUTPUT_PIN_1 : GPIO_OUTPUT_PIN_2;
    gpio_set_level(pin, state);
    ESP_LOGI(TAG, "GPIO %d (endpoint %d) set to %s", pin, endpoint, state ? "ON" : "OFF");
}

// Callback для запуска комиссинга
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start commissioning");
}

// Обработчик сигналов Zigbee
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver init: %s", gpio_driver_init() ? "failed" : "success"); // Убрал false из вызова
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", 
                   esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGI(TAG, "Start network steering failed, status: %s, retry after 1s", 
                   esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                 ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04x, Channel:%d, Short Address: 0x%04x)",
                   extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                   extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                   esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering failed, status: %s, retry after 1s", 
                   esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                 ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", 
               esp_zb_zdo_signal_to_string(sig_type), sig_type,
               esp_err_to_name(err_status));
        break;
    }
}
// Обработчик атрибутов Zigbee
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if ((message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT || 
         message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT + 1) &&
        message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        bool state = *(bool *)message->attribute.data.value;
        gpio_driver_set_state(message->info.dst_endpoint, state);
    }
    return ESP_OK;
}

// Обработчик действий Zigbee
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
    default:
        ESP_LOGW(TAG, "Unhandled Zigbee action callback: 0x%x", callback_id);
        return ESP_OK;
    }
}

// Главная задача Zigbee
static void esp_zb_task(void *pvParameters)
{
    // Инициализация стека Zigbee
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Создаем список эндпоинтов
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    
    // Конфигурация кластера On/Off
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };


    // Создаем и добавляем первый эндпоинт
    esp_zb_cluster_list_t *cluster_list1 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list1, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list1, esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_ep_list_add_ep(ep_list, cluster_list1, 
                         (esp_zb_endpoint_config_t){
                             .endpoint = HA_ESP_LIGHT_ENDPOINT,
                             .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                             .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID
                         });
    // Добавляем информацию о производителе для первого эндпоинта
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, HA_ESP_LIGHT_ENDPOINT, &info);

    // Создаем и добавляем второй эндпоинт
    esp_zb_cluster_list_t *cluster_list2 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list2, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list2, esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_ep_list_add_ep(ep_list, cluster_list2, 
                         (esp_zb_endpoint_config_t){
                             .endpoint = HA_ESP_LIGHT_ENDPOINT + 1,
                             .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                             .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID
                         });
    // Добавляем информацию о производителе для второго эндпоинта
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, HA_ESP_LIGHT_ENDPOINT + 1, &info);

    // Регистрируем устройство
    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Конфигурация платформы Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Запуск задачи Zigbee
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}