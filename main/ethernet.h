#pragma once

#include <stdbool.h>
// interface identifier
typedef enum {
    IFACE_NONE = 0,
    IFACE_ETH,
    IFACE_WIFI
} active_iface_t;

extern bool eth_connected;
extern bool wifi_active;
extern bool mqtt_running;
extern active_iface_t active_iface;
//extern esp_mqtt_client_handle_t mqtt_client;

// ---- Ethernet PHY settings (ESP32 Ethernet Kit - LAN8720) ----
#define ETH_PHY_ADDR      1
#define ETH_PHY_POWER_PIN 5   
#define ETH_MDC_PIN       23
#define ETH_MDIO_PIN      18
#define ETH_RMII_CLK_PIN  0

#define ETH_TAG "ETH_CONNECTION"

void ethernet_start(void);
bool ethernet_is_connected(void);
