#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_event.h"

extern bool wifi_active;

void wifi_stop(void);
void wifi_init_sta(void);
void wifi_start_connect(void);
void wifi_got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);