#pragma once

#include "driver/i2c_master.h"

#define BME280_ADDR 0x76

// i2c pins and bus settings
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_SDA_IO     14
#define I2C_MASTER_SCL_IO     15
#define I2C_MASTER_FREQ_HZ    100000

void sensor_bme280_init(i2c_master_bus_handle_t shared_bus);
void sensor_bme280_read(float *temp, float *hum, float *pres);