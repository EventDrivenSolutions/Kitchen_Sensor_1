#pragma once

#include "mqtt_client.h"
#include "ethernet.h"   // for active_iface_t
#include "lvgl.h"

// ---- MQTT settings --------
#define MQTT_BROKER_URI "mqtt://192.168.1.240:1883"
#define MQTT_TOPIC_PUB   "/kitchen/sensor"
#define MQTT_TOPIC_SUB  "/kitchen/control"
#define MQTT_QOS_LEVEL 0 // 0 is forget, less overhead --> 1 is at least once, was getting jumpy and not really needed here
#define MQTT_RETAIN_FLAG 0

// added in for UI
typedef struct {
    lv_obj_t *temp_label;
    lv_obj_t *hum_label;
    lv_obj_t *pres_label;
    lv_obj_t *conn_label;
    lv_obj_t *cmd_label;
} ui_handles_t;

extern void *global_ui_handle;

//struct for publish loop, pass to client and to screen
typedef struct {
    esp_mqtt_client_handle_t client;
    void *ui;   // ui_handles_t* TODO: RECAST
} mqtt_task_ctx_t;

// Start MQTT 
void mqtt5_app_start(active_iface_t iface);

// for commanding pwm fan
void process_control_message(const char *json);
