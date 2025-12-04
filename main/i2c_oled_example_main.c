
#include "sensor.h"
#include "ethernet.h"
#include "mqtt.h"
#include "wifi.h"
#include "fan_control.h"



#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "lvgl.h"


#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *TAG = "IOT DEVICE MAIN";

#define I2C_BUS_PORT  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           GPIO_NUM_14
#define EXAMPLE_PIN_NUM_SCL           GPIO_NUM_15
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              CONFIG_EXAMPLE_SSD1306_HEIGHT
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    5
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2
#define EXAMPLE_LVGL_PALETTE_SIZE      8
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ

void *global_ui_handle = NULL;

// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
static uint8_t oled_buffer[EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
_lock_t lvgl_api_lock;

// Fault detection
static void net_status_task(void *arg);

//------------ OLED Example Code----------//

extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += EXAMPLE_LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            /* The order of bits is MSB first
                        MSB           LSB
               bits      7 6 5 4 3 2 1 0
               pixels    0 1 2 3 4 5 6 7
                        Left         Right
            */
            bool chroma_color = (px_map[(hor_res >> 3) * y  + (x >> 3)] & 1 << (7 - x % 8));

            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, EXAMPLE_LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, EXAMPLE_LVGL_TASK_MAX_DELAY_MS);
        usleep(1000 * time_till_next_ms);
    }
}

// ------- Added Functionality to OLED example code to print IOT screen------//

// IOT UI
ui_handles_t ui_init(void)
{
    ui_handles_t ui;

    // --- Create labels ---
    ui.temp_label = lv_label_create(lv_scr_act());
    ui.hum_label  = lv_label_create(lv_scr_act());
    ui.pres_label = lv_label_create(lv_scr_act());
    ui.conn_label = lv_label_create(lv_scr_act());
    ui.cmd_label = lv_label_create(lv_scr_act());

    lv_label_set_long_mode(ui.cmd_label, LV_LABEL_LONG_SCROLL);
    lv_obj_set_width(ui.cmd_label, EXAMPLE_LCD_H_RES);

    // --- Position labels on screen ---
    lv_obj_align(ui.conn_label, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_align(ui.temp_label,  LV_ALIGN_TOP_LEFT, 0, 14);
    lv_obj_align(ui.hum_label, LV_ALIGN_TOP_LEFT, 0, 26);
    lv_obj_align(ui.pres_label, LV_ALIGN_TOP_LEFT, 0, 38);
    lv_obj_align(ui.cmd_label, LV_ALIGN_TOP_LEFT, 0, 50);

    // --- Set initial placeholder text ---
    lv_label_set_text(ui.conn_label, "No Connection");
    lv_label_set_text(ui.temp_label, "Temp: --.- C");
    lv_label_set_text(ui.hum_label,  "Hum : --.- %");
    lv_label_set_text(ui.pres_label, "Pres: ---- hPa");
    lv_label_set_text(ui.cmd_label, "CMD: auto ");
    

    return ui;
}

// Moved example OLED main to initializing function to clean up main .. see below
// OLED init
static lv_display_t *init_display(i2c_master_bus_handle_t *out_bus){

    //i2c_master_bus_handle_t i2c_bus = NULL;

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,                     // According to SH1107 datasheet
        .flags =
        {
            .disable_control_phase = 1,
        }
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();
    // create a lvgl display
    lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    // associate the i2c panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    //flipping panel--------------------------------------------------------------------------------KC 12 2 25
    esp_lcd_panel_mirror(panel_handle, true, true);  // Flip horizontally + vertically

    // create draw buffer
    void *buf = NULL;
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers");
    // LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette.
    size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8 + EXAMPLE_LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);

    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display);

    ESP_LOGI(TAG, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");

    // return siaplay.. and bus for other i2c devices
    *out_bus = i2c_bus;
    return display;
}

// Moved publish loop to main for clarity
void mqtt_publish_loop(void *param)
{

    mqtt_task_ctx_t *ctx = (mqtt_task_ctx_t *)param;
    esp_mqtt_client_handle_t client = ctx->client;
    ui_handles_t *ui = (ui_handles_t *)ctx->ui;

    float t, h, p;
    char json[128];

    while (1) {

        //TODO: Set real values from added sensors
        sensor_bme280_read(&t, &h, &p);
        //send temp to fan - if command is a static command (manual), do not update
        if (!manual_mode) {
            fan_control_update(t);
        }
        // check whether and send for Rishi requirment
        bool using_eth = eth_connected;
        // get time for rishi
        int64_t now_ms  = esp_timer_get_time() / 1000;
        // create message to send over mqtt5
        snprintf(json, sizeof(json),
            "{"
                "\"con_type\":%d,"
                "\"dev_id\":\"kitchen_node_1\","
                "\"time\":%lld,"
                "\"temp\":%.2f,"
                "\"humidity\":%.2f,"
                "\"pressure\":%.2f"
            "}",
            using_eth ? 1 : 0,
            (long long)now_ms,
            t, h, p
        );

        // 0,1,0 eventually change to 0,0,0?
        esp_mqtt_client_publish(client, MQTT_TOPIC_PUB, json, 0, MQTT_QOS_LEVEL, MQTT_RETAIN_FLAG);

        //----update the oled display
        _lock_acquire(&lvgl_api_lock);

        char buf[32];

        snprintf(buf, sizeof(buf), "Temp: %.2f C", t);
        lv_label_set_text(ui->temp_label, buf);

        snprintf(buf, sizeof(buf), "Hum : %.2f %%", h);
        lv_label_set_text(ui->hum_label, buf);

        snprintf(buf, sizeof(buf), "Pres: %.2f hPa", p);
        lv_label_set_text(ui->pres_label, buf);

       // now updating conn in fault task

        _lock_release(&lvgl_api_lock);
        // -------------------------- REQ BELOW ----- read every second//
        vTaskDelay(pdMS_TO_TICKS(1000));  // every 1 seconds -- Req 
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    // Initialize display and get soon to be shared bus
    i2c_master_bus_handle_t shared_bus = NULL;
    init_display(&shared_bus);

    //Initialize sensor with shared bus
    sensor_bme280_init(shared_bus);

    // Init the fan 
    fan_control_init();
    
    // Initialize UI
    ui_handles_t ui;
    _lock_acquire(&lvgl_api_lock);
    ui = ui_init();                 
    _lock_release(&lvgl_api_lock);

    global_ui_handle = &ui;

    // Start connection monitoring task
    xTaskCreate(net_status_task, "net_status", 4096, &ui, 3, NULL);


    // init connectivity

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // get Wifi ready and then launch Ethernet, users wont mind the few extra seconds frontloaded

    wifi_init_sta();
    wifi_active = false;

    // now try Eth, if its on great otherwise it will switch to WiFi
    ESP_LOGI(TAG, "[APP] Ethernet preferred. WiFi on standby.");
    ethernet_start();

        // Prevent app_main() from returning which was an issue
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

static void net_status_task(void *arg)
{
    ui_handles_t *ui = (ui_handles_t *)arg;

    static int no_conn_seconds = 0;

    while (1) {

        // count no connectivity
        if (!eth_connected && !wifi_active) {
            no_conn_seconds++;
        } else {
            no_conn_seconds = 0;
        }

        // print fault at 10 seconds intervals when no connection
        if (no_conn_seconds > 0 && no_conn_seconds % 10 == 0) {
            ESP_LOGW("FAULT", "BOTH CONNECTIONS DOWN for %d seconds",
                     no_conn_seconds);
        }

        // update oled connection label
        _lock_acquire(&lvgl_api_lock);

        if (!eth_connected && !wifi_active) {
            lv_label_set_text(ui->conn_label, "No Connection");
        }
        else if (eth_connected) {
            lv_label_set_text(ui->conn_label, "Conn: ETH");
        }
        else if (wifi_active) {
            lv_label_set_text(ui->conn_label, "Conn: WIFI");
        }

        _lock_release(&lvgl_api_lock);

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second loop
    }
}
