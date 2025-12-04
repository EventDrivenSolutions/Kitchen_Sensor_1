#include "mqtt.h"
#include "wifi.h"     
#include "esp_wifi.h"

#include "ethernet.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include <string.h>

bool eth_connected = false;
bool wifi_active = false;
bool mqtt_running = false;
active_iface_t active_iface = IFACE_NONE;

//esp_mqtt_client_handle_t mqtt_client = NULL;

extern void wifi_start_connect(void);
extern void wifi_stop(void);
extern void mqtt5_app_start(active_iface_t iface);

/*-------------ETHERNET FUNCTION SECTION------------------------------*/
/*-------------Modified from ESP_IDF ethernet example code-------------*/

// Ethernet Function Calls
static esp_err_t eth_init(esp_eth_handle_t *eth_handle_out)
{
    if (eth_handle_out == NULL) {
        ESP_LOGE(ETH_TAG, "invalid argument: eth_handle_out cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Init common MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    // Update PHY config based on board specific configuration
    phy_config.phy_addr = ETH_PHY_ADDR; // common addr
    phy_config.reset_gpio_num = ETH_PHY_POWER_PIN; // no rst pin needed

    // Init vendor specific MAC config to default
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    // Update vendor specific MAC config based on board configuration
    esp32_emac_config.smi_gpio.mdc_num = ETH_MDC_PIN;
    esp32_emac_config.smi_gpio.mdio_num = ETH_MDIO_PIN;

    esp32_emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
    esp32_emac_config.clock_config.rmii.clock_gpio = ETH_RMII_CLK_PIN; // standard

    // Create new ESP32 Ethernet MAC instance
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    if (mac == NULL) {
        ESP_LOGE(ETH_TAG, "create MAC instance failed");
        return ESP_FAIL;
    }

    // Create new generic PHY instance
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
    if (phy == NULL) {
        ESP_LOGE(ETH_TAG, "create PHY instance failed");
        mac->del(mac);
        return ESP_FAIL;
    }

    // Init Ethernet driver to default and install it
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    if (esp_eth_driver_install(&config, &eth_handle) != ESP_OK) {
        ESP_LOGE(ETH_TAG, "Ethernet driver install failed");
        mac->del(mac);
        phy->del(phy);
        return ESP_FAIL;
    }

    *eth_handle_out = eth_handle;

    return ESP_OK;
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    // modified events to run IOT device. Ethernet Primary - wifi secondary

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        eth_connected = true;
        ESP_LOGI(ETH_TAG, "Ethernet Link Up");
        //esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        //ESP_LOGI(ETH_TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        if (wifi_active){
            ESP_LOGI(ETH_TAG, "Ethernet Up: WiFi Deactivated");
            wifi_stop();
            wifi_active = false;
        }
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        eth_connected = false;
        ESP_LOGI(ETH_TAG, "Ethernet Link Down");

        // adding delay in case of small dissconection
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(ETH_TAG, "Reinitializing WiFi driver...");
        wifi_init_sta();

         if (!wifi_active){
            ESP_LOGI(ETH_TAG, "Ethernet Down: WiFi Activated.");
            wifi_start_connect();
            wifi_active = true;
        }
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(ETH_TAG, "Ethernet Started");
        // If link is not up shortly after start → failover to WiFi
        vTaskDelay(pdMS_TO_TICKS(150));
        if (!eth_connected && !wifi_active) {
            ESP_LOGW(ETH_TAG, "Ethernet cable not detected. Enabling WiFi...");
            wifi_start_connect();
            wifi_active = true;
        }
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(ETH_TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void eth_got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    //static bool mqtt_started = false;

    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(ETH_TAG, "Ethernet Got IP Address");
    ESP_LOGI(ETH_TAG, "~~~~~~~~~~~");
    ESP_LOGI(ETH_TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(ETH_TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(ETH_TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(ETH_TAG, "~~~~~~~~~~~");

    // Ethernet preffered
    active_iface = IFACE_ETH;

    if (wifi_active) {
        ESP_LOGI(ETH_TAG, "Stopping WiFi to use Ethernet");
        wifi_stop();
        wifi_active = false;
    }

    // Added for eth functionality
    if (!mqtt_running) {
        ESP_LOGI(ETH_TAG, "Starting MQTT on Ethernet");
        mqtt5_app_start(IFACE_ETH);
        mqtt_running = true;
    }
}

/** Ethernet Startup Function Call */
void ethernet_start(void)
{
    // Initialize Ethernet driver
    // Create Handle to store Ethernet Driver
    esp_eth_handle_t eth_handle;
    // Initialize Ethernet hardware with program halting on failure
    ESP_ERROR_CHECK(eth_init(&eth_handle));

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    // Create Eth network interface
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    // Create layer between Eth driver and TCP/IP stack
    esp_eth_netif_glue_handle_t eth_netif_glue = esp_eth_new_netif_glue(eth_handle);
    // Attach Ethernet driver to TCP/IP stack - Connect Hardware (MAC/PHYSICAL to ETH Driver to Netif Glue to TCP/IP Stack)
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, eth_netif_glue));

    // Register user defined event handlers - Whenever anything happens with Ethernet call Eth Event Handler
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    // When IP address gotten, call got ip event handler
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_got_ip_event_handler, NULL));

    // Start Ethernet driver state machine (Link negotiations, cable detection, DHCP runs)
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

// vestigial
bool ethernet_is_connected(void)
{
    return eth_connected;
}