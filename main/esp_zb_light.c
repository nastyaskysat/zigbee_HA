#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "string.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_USB_UART";
#define ESP_MANUFACTURER_NAME "ESP_CUSTOM"
#define ESP_MODEL_IDENTIFIER "ESP_LIGHT"

// UART конфигурации для передачи через реальные выводы
#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      256
#define UART_QUEUE_SIZE    20
#define UART_TX_PIN        GPIO_NUM_16  // Выберите подходящие GPIO для вашей платы
#define UART_RX_PIN        GPIO_NUM_17  // Выберите подходящие GPIO для вашей платы

// Protocol definition
#define CMD_PREFIX        "CMD:"
#define CMD_ON_TEMPLATE   "CMD:EP%d:ON"
#define CMD_OFF_TEMPLATE  "CMD:EP%d:OFF"
#define CMD_END           "\r\n"

// Global variables
static QueueHandle_t uart_queue;

// Заранее объявляю функции
static esp_err_t uart_driver_init(void);
static void send_command_to_wirenboard(uint8_t endpoint, bool state);
static void uart_event_task(void *pvParameters);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static void esp_zb_task(void *pvParameters);

// UART инициализация с реальными выводами
static esp_err_t uart_driver_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver with event queue
    esp_err_t ret = uart_driver_install(UART_PORT_NUM, 
                                      UART_BUF_SIZE * 2,
                                      UART_BUF_SIZE * 2,
                                      UART_QUEUE_SIZE,
                                      &uart_queue,
                                      0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return ret;
    }
    
    ret = uart_param_config(UART_PORT_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        return ret;
    }
    
    // Устанавливаем пины для реального UART
    ret = uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART pin config failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "UART initialized with TX=%d, RX=%d", UART_TX_PIN, UART_RX_PIN);
    return ESP_OK;
}



// UART обработчик событий
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t rx_buf[UART_BUF_SIZE];
    
    for (;;) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Считываем с юарта
                    int len = uart_read_bytes(UART_PORT_NUM, rx_buf, event.size, portMAX_DELAY);
                    if (len > 0) {
                        rx_buf[len] = '\0'; 
                        //ESP_LOGI(TAG, "Received from Wirenboard: %s", rx_buf);
                    }
                    break;
                    
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer overflow");
                    uart_flush_input(UART_PORT_NUM);
                    break;
                    
                default:
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}


static void send_command_to_wirenboard(uint8_t endpoint, bool state)
{
    char command[32];
    
    if (state) {
        snprintf(command, sizeof(command), CMD_ON_TEMPLATE, endpoint);
    } else {
        snprintf(command, sizeof(command), CMD_OFF_TEMPLATE, endpoint);
    }
    
    strcat(command, CMD_END);
    
    int len = strlen(command);
    int sent = uart_write_bytes(UART_PORT_NUM, command, len);
    
    // if (sent == len) {
    //     ESP_LOGI(TAG, "Sent command to Wirenboard: %s", command);
    // } else {
    //     ESP_LOGE(TAG, "Failed to send command to Wirenboard, sent %d/%d bytes", sent, len);
    // }
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
            ESP_LOGI(TAG, "Deferred driver init: %s", uart_driver_init() ? "failed" : "success");
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
    
    if ((message->info.dst_endpoint >= HA_ESP_LIGHT_ENDPOINT && 
         message->info.dst_endpoint <= HA_ESP_LIGHT_ENDPOINT + 1) &&
        message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        bool state = *(bool *)message->attribute.data.value;
        send_command_to_wirenboard(message->info.dst_endpoint, state);
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
       // ESP_LOGW(TAG, "Unhandled Zigbee action callback: 0x%x", callback_id);
        return ESP_OK;
    }
}

static void esp_zb_task(void *pvParameters)
{
    // Инициализация стека Zigbee
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Создаем список эндпоинтов
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    
    // Конфигурация кластера On/Off 
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,
    };

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = "ESP_CUSTOM",
        .model_identifier = "ESP_LIGHT",
    };

     // Первый эндпоинт
    esp_zb_cluster_list_t *cluster_list1 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list1, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_attribute_list_t *on_off_attr_list1 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    uint8_t attr_access = 0x02;
    esp_zb_cluster_add_attr(on_off_attr_list1, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, 
                           ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, 
                           ESP_ZB_ZCL_ATTR_TYPE_BOOL, 
                           ESP_ZB_ZCL_ATTR_ACCESS_WRITE_ONLY,  //явно прописываю write-only
                           &(bool){false});
    esp_zb_cluster_list_add_on_off_cluster(cluster_list1, on_off_attr_list1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_ep_list_add_ep(ep_list, cluster_list1, 
                         (esp_zb_endpoint_config_t){
                             .endpoint = HA_ESP_LIGHT_ENDPOINT,
                             .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                             .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID
                         });

    // Второй эндпоинт 
    esp_zb_cluster_list_t *cluster_list2 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list2, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_attribute_list_t *on_off_attr_list2 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    esp_zb_cluster_add_attr(on_off_attr_list2, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, 
                           ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, 
                           ESP_ZB_ZCL_ATTR_TYPE_BOOL, 
                           ESP_ZB_ZCL_ATTR_ACCESS_WRITE_ONLY, //явно прописываю write-only
                           &(bool){false});
    esp_zb_cluster_list_add_on_off_cluster(cluster_list2, on_off_attr_list2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_ep_list_add_ep(ep_list, cluster_list2, 
                         (esp_zb_endpoint_config_t){
                             .endpoint = HA_ESP_LIGHT_ENDPOINT + 1,
                             .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                             .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID
                         });

    // Регистрация устройства
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

    // Инициализация UART
    ESP_ERROR_CHECK(uart_driver_init());
    
    // Запуск юарта
    xTaskCreate(uart_event_task, "uart_task", 2048, NULL, 10, NULL);

    // Zigbee конфигцрации
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Zigbee задача
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
