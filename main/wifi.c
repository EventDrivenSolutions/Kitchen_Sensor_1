#include "wifi.h"
#include "ethernet.h"
#include "mqtt.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

/* WiFi credentials ------------------ Modify before uploading-------------*/ 
#define WIFI_SSID      "ssid"
#define WIFI_PASS      "pass"
#define MAX_RETRY      10

static const char *TAG = "myWIFI";

// -----------modified from esp idf example code------------//
// internal events handle it
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG,"WiFi STA started on standby");
        //esp_wifi_connect();
    }else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi Connected (Associated)");
        wifi_active = true;
    }else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG,"WiFi Diconnected");
        wifi_active = false;

            // Only connect if WiFi was intentionally activated (Ethernet down)
        if (!eth_connected) {
            ESP_LOGI(TAG, "WiFi will retry (Ethernet is DOWN)");
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG, "WiFi staying OFF because Ethernet is UP");
        }
    }
}

// function to begin connection after initialization
void wifi_start_connect(void)
{
    ESP_LOGI(TAG, "WiFi manual connect triggered");
    esp_wifi_start();
    esp_wifi_connect();
}

// mqtt loop caller if have IP 
void wifi_got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    //static bool mqtt_started = false;
    if (eth_connected) {
            ESP_LOGI("wifi", "Ignoring WiFi IP (Ethernet already active)");
            esp_wifi_disconnect();
            wifi_active = false;
            return;
        }

    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI("wifi", "WiFi Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

    
    active_iface = IFACE_WIFI;
    wifi_active = true;
    if (!mqtt_running) {
        ESP_LOGI("wifi", "Starting MQTT on WiFi (Ethernet is down)");
        mqtt5_app_start(IFACE_WIFI);
        mqtt_running = true;
    }
}

//wifii start
void wifi_init_sta(void)
{
    ESP_LOGI(TAG, "Initializing WiFi station...");

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                           ESP_EVENT_ANY_ID,
                                           &event_handler,
                                           NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                           IP_EVENT_STA_GOT_IP,
                                           &wifi_got_ip_event_handler,
                                           NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
   // ESP_LOGI(TAG, "WiFi initialized (not started)");
   // ESP_LOGI(TAG, "wifi_init_sta finished.");
   ESP_LOGI(TAG, "WiFi initialized (STA mode, not started yet)");
}

// wifi stop
void wifi_stop(void)
{
    ESP_LOGI(TAG, "WiFi stopping");
    esp_wifi_disconnect();
    esp_wifi_stop();
    //esp_wifi_deinit(); do not deninit
}
