
ESP32 OLED IoT Node
===================
Created with multiple ESP-IDF libraries, including wifi, ethernet, mqtt, i2c, and oled.
This project implements a small ESP32-based IoT environmental node with:
 - SSD1306 I2C OLED display using LVGL 9
 - BME280 temperature, humidity, and pressure sensor
 - MQTT 5.0 publishing
 - MQTT command handling (set temperature, fan override)
 - Automatic Ethernet/WiFi failover logic
 - Fan control (automatic PI loop + manual override)
 - Network fault monitoring with on-screen “No Connection”
 - Smooth scrolling command display on OLED
 - Modular drivers for Ethernet, WiFi, Sensor, Fan, and MQTT

Features
--------
1. **OLED UI**
   - Shows connection state (ETH, WiFi, or No Connection)
   - Displays Temp / Humidity / Pressure
   - Scrolls incoming MQTT commands

2. **MQTT 5.0 Integration**
   - Publishes JSON every second:
     - temp, humidity, pressure
     - connection type (0=WiFi, 1=Ethernet)
     - timestamp
   - Accepts commands:
     - `{"set_temp": XX.X}`
     - `{"fan": XX}`

3. **Ethernet + WiFi Failover**
   - Ethernet preferred
   - WiFi connects only when Ethernet is down
   - OLED updates connection status live

4. **Network Fault Detection**
   - If both interfaces are down:
     - OLED shows **"No Connection"**
     - Log prints warnings every 10 seconds

5. **Fan Control**
   - Automatically adjusts fan speed based on temperature
   - Manual mode when MQTT fan command is received

Project Structure
-----------------
main/

  ├── i2c_oled_example_main.c   (main application + LVGL)
  
  ├── mqtt.c / mqtt.h           (MQTT handling)
  
  ├── wifi.c / wifi.h           (WiFi station logic)
  
  ├── ethernet.c / ethernet.h   (Ethernet initialization + callbacks)
  
  ├── sensor.c / sensor.h       (BME280 driver)
  
  ├── fan_control.c / fan_control.h (Fan PI controller)

Usage
-----
1. Set WiFi credentials in `wifi.c`
2. Configure MQTT broker IP in `mqtt.h`
3. Build and flash with ESP-IDF 5.3+
4. Use `mosquitto_pub` to control it:
   - `mosquitto_pub -t /kitchen/control -m '{"set_temp": 22.5}'`
   - `mosquitto_pub -t /kitchen/control -m '{"fan": 75}'`

Hardware
--------
- ESP32-Ethernet-Kit_A_V1.2
- SSD1306 128x64 OLED
- BME280 sensor
- 5V 2-wire PWM fan + MOSFET (GPIO PWM)
- Ethernet PHY (LAN8720) board

Notes
-----
- LVGL is mutex-protected (`_lock_t lvgl_api_lock`)
- Scrolling command text is handled with LVGL long-scroll mode
- Ethernet always takes priority when available



