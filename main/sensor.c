








#include "sensor.h"
#include "esp_log.h"
#include <stdint.h>

static const char *TAG = "BME280";

static i2c_master_bus_handle_t bme_bus = NULL;
static i2c_master_dev_handle_t bme_dev = NULL;

// bme280 calibration registers

static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static uint8_t  dig_H1;
static int16_t  dig_H2;
static uint8_t  dig_H3;
static int16_t  dig_H4, dig_H5;
static int8_t   dig_H6;

static int32_t t_fine;

// --------I2c sensor modified imported mbed
// init
static void sensor_i2c_init(i2c_master_bus_handle_t shared_bus)
{
    bme_bus = shared_bus;

    // Configure the BME280 as an I2C device on this bus
    i2c_device_config_t dev_cfg = {
        .device_address = BME280_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Add device to I2C bus passed in
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bme_bus, &dev_cfg, &bme_dev));

    ESP_LOGI(TAG, "BME280 added to shared I2C bus");
}

// i2c read 
static void bme_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    ESP_ERROR_CHECK(i2c_master_transmit(bme_dev, &reg, 1, 1000));
    ESP_ERROR_CHECK(i2c_master_receive(bme_dev, buf, len, 1000));
}

// i2c write
static void bme_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    ESP_ERROR_CHECK(i2c_master_transmit(bme_dev, data, 2, 1000));
}

// load calibration data
static void bme_load_calibration(void)
{
    uint8_t calib[32];
    bme_read_bytes(0x88, calib, 24);

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];

    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];

    dig_H1 = calib[24];

    uint8_t hcalib[8];
    bme_read_bytes(0xE1, hcalib, 7);

    dig_H2 = (hcalib[1] << 8) | hcalib[0];
    dig_H3 = hcalib[2];
    dig_H4 = (hcalib[3] << 4) | (hcalib[4] & 0x0F);
    dig_H5 = (hcalib[5] << 4) | (hcalib[4] >> 4);
    dig_H6 = hcalib[6];
}

// initialize sensor
void sensor_bme280_init(i2c_master_bus_handle_t shared_bus)
{
    sensor_i2c_init(shared_bus);

    bme_write(0xF2, 0x01); // Humidity oversampling x1
    bme_write(0xF4, 0x27); // Temp + press oversampling x1, normal mode
    bme_write(0xF5, 0xA0); // Filter off, standby 1000 ms

    bme_load_calibration();

    ESP_LOGI(TAG, "BME280 initialized & calibration loaded");
}

// temperature compensation
static float bme_comp_temp(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * dig_T2) >> 11;
    int32_t var2 =
        (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) *
         dig_T3) >>
        14;

    t_fine = var1 + var2;

    return (t_fine * 5 + 128) >> 8;
}

// read temperature, humidity, pressure
void sensor_bme280_read(float *temp, float *hum, float *pres)
{
    uint8_t data[8];
    bme_read_bytes(0xF7, data, 8);

    int32_t adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t adc_h = (data[6] << 8) | data[7];

    *temp = bme_comp_temp(adc_t) / 100.0f;

    // humidity calc 
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r =
        (((((adc_h << 14) - (((int32_t)dig_H4) << 20) -
            (((int32_t)dig_H5) * v_x1_u32r)) +
           ((int32_t)16384)) >>
          15) *
         (((((((v_x1_u32r * (int32_t)dig_H6) >> 10) *
              (((v_x1_u32r * (int32_t)dig_H3) >> 11) + 32768)) >>
             10) +
            2097152) *
               (int32_t)dig_H2 +
           8192) >>
          14));
    v_x1_u32r =
        (v_x1_u32r -
         (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
           dig_H1) >>
          4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r =
        (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *hum = (v_x1_u32r >> 12) / 1024.0f;

    // pressure calc
    int64_t var1, var2, p64;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 =
        ((var1 * var1 * (int64_t)dig_P3) >> 8) +
        ((var1 * (int64_t)dig_P2) << 12);
    var1 =
        (((((int64_t)1) << 47) + var1)) *
        ((int64_t)dig_P1) >>
        33;

    if (var1 == 0)
        return;

    p64 = 1048576 - adc_p;
    p64 = (((p64 << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p64 >> 13) * (p64 >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p64) >> 19;

    p64 = ((p64 + var1 + var2) >> 8) +
          (((int64_t)dig_P7) << 4);

    *pres = p64 / 25600.0f;
}