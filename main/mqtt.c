#include "mqtt.h"

// C includes
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ESP-IDF
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// project
#include "sensor.h"     // sensor_bme280_read()
#include "ethernet.h"   // eth_connected, active_iface, mqtt_running
#include "fan_control.h" // send commands 

#include "lvgl.h"
extern _lock_t lvgl_api_lock;


static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *TAG = "MQTT";
static TaskHandle_t mqtt_publish_task = NULL; //track publish task so i dont create multiple when failures occur

extern float T_set;
extern bool eth_connected;
extern bool mqtt_running;
extern active_iface_t active_iface;
extern bool wifi_active;

/*-------------MQTT FUNCTION SECTION--------------*/
/*----Modified from ESP-IDF example code on vscode---*/

// Loop Posting to the MQTT Broker with following function
// Currently this function is in main...
extern void mqtt_publish_loop(void *param);

 // Event handler registered to receive MQTT events
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    //int msg_id;
    // example message to be posted to broker!!
    //const char *json = "{\"temp\":22.3,\"humidity\":48.0,\"pressure\":1012.8}";

   // ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        
        // Subscribe for recieving temp commads
        esp_mqtt_client_subscribe(client, MQTT_TOPIC_SUB, 0);
        // Prepare context for publish loop
        extern void *global_ui_handle;   // defined in main.c
        mqtt_task_ctx_t *ctx = malloc(sizeof(mqtt_task_ctx_t));
        ctx->client = client;
        ctx->ui = global_ui_handle;

        // Start unified publish/display task
        xTaskCreate(mqtt_publish_loop, "mqtt_loop", 4096, ctx, 5, &mqtt_publish_task);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED... Attempting new connection...");
        //mqtt_running = false;
        if (mqtt_publish_task != NULL) {
            vTaskDelete(mqtt_publish_task);
            mqtt_publish_task = NULL;
        }
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d, reason code=0x%02x ", event->msg_id, (uint8_t)*event->data);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        esp_mqtt_client_disconnect(client);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
    {
        // topic to string with null
        char topic[64];
        memcpy(topic, event->topic, event->topic_len);
        topic[event->topic_len] = '\0';

        // payload to string with null
        char payload[128];
        memcpy(payload, event->data, event->data_len);
        payload[event->data_len] = '\0';

        //display event
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG, "TOPIC=%s", topic);
        ESP_LOGI(TAG, "DATA=%s", payload);

        // Check if message is on control topic
        if (strcmp(topic, MQTT_TOPIC_SUB) == 0) {
            process_control_message(payload);
        }

        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        ESP_LOGI(TAG, "Transport error");
        //mqtt_running = false;
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// Configures and starts MQTT CLIENT
void mqtt5_app_start(active_iface_t iface)
{
    static char client_id[32];

        // Stop old publish loop task if running
    if (mqtt_publish_task != NULL) {
        vTaskDelete(mqtt_publish_task);
        mqtt_publish_task = NULL;
    }

    // Stop any old MQTT client
    if (mqtt_client != NULL) {
        ESP_LOGI("MQTT", "Stopping previous MQTT client");
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }

    ESP_LOGI("MQTT", "Starting MQTT on %s",
             iface == IFACE_ETH ? "ETHERNET" : "WIFI");
   
    active_iface = iface;  // record chosen interface
    mqtt_running = true;

    if (iface == IFACE_ETH) {
        strcpy(client_id, "esp32_eth_client");
    } else if (iface == IFACE_WIFI) {
        strcpy(client_id, "esp32_wifi_client");
    } else {
        strcpy(client_id, "esp32_unknown_client");
    }

    const esp_mqtt_client_config_t mqtt5_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
        },
        .credentials = {
            .client_id = client_id,
        },
        .session = {
            .protocol_ver = MQTT_PROTOCOL_V_5,
        },
    };


    mqtt_client = esp_mqtt_client_init(&mqtt5_cfg);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// helper to send commands to oled
static void ui_print_cmd(const char *json)
{
    ui_handles_t *ui = (ui_handles_t *)global_ui_handle;
    _lock_acquire(&lvgl_api_lock);
    lv_label_set_text_fmt(ui->cmd_label, "Cmd: %s", json);
    _lock_release(&lvgl_api_lock);
}

// process incoming json commands
void process_control_message(const char *json)
{
    ESP_LOGI("CTRL", "Processing command: %s", json);

    // see commands below for how to send struc to IOT device
    
    //mosquitto_pub -h 192.168.1.240 -t /kitchen/control -m "fan_on"
    if (strcmp(json, "fan_on") == 0) {
        manual_mode = true;
        fan_control_force(100.0f);
        ui_print_cmd(json);
        return;
    }
    //mosquitto_pub -h 192.168.1.240 -t /kitchen/control -m "fan_off"
    if (strcmp(json, "fan_off") == 0) {
        manual_mode = true;
        fan_control_force(0.0f);
        ui_print_cmd(json);
        return;
    }

    // Fan speed: { "fan": 55 }
    //mosquitto_pub -h 192.168.1.240 -t /kitchen/control -m '{"fan": 75}'
    float fan_speed = 0.0f;
    if (sscanf(json, "{ \"fan\" : %f }", &fan_speed) == 1) {
        manual_mode = true;
        fan_control_force(fan_speed);
        ESP_LOGI("CTRL", "Fan speed set to %.1f%%", fan_speed);
        ui_print_cmd(json);
        return;
    }

    // Temperature target: { "set_temp": 22.5 }
    //mosquitto_pub -h 192.168.1.240 -t /kitchen/control -m '{"set_temp": 22.5}'
    float new_temp = 0.0f;
    if (sscanf(json, "{ \"set_temp\" : %f }", &new_temp) == 1) {
        T_set = new_temp;
        ESP_LOGI("CTRL", "New target temperature = %.2f C", new_temp);
        // switch to auto if set temp
        manual_mode = false;
        // different to print just the num
        ui_print_cmd(json);
        return;
    }

    // turn fan back to auto
    //mosquitto_pub -h 192.168.1.240 -t /kitchen/control -m "auto"
    if (strcmp(json, "auto") == 0) {
        manual_mode = false;
        T_set = 21.7;

        extern float integrator;
        integrator = 0.0f;

        ui_print_cmd(json);
        return;
    }

    ESP_LOGW("CTRL", "Unknown command: %s", json);
}


