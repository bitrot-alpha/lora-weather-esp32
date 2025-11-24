# LVGL Demo with SHT31-D

This is a demo of LVGL on an ST7796S 4.0 inch SPI display with an SHT31-D sensor.  
The red board with SD card slot, https://www.lcdwiki.com/4.0inch_SPI_Module_ST7796  
It shows loading images and dynamic text updates with the SHT sensor values.  
This uses a standard ESP32 dev board, NOT the Heltec board with the LoRa modem.  

![ST7796S display with temp + humidity demo](/doc/pic/lvgl_demo.jpg)

## Display Pins
```
DISPLAY -> ESP32 (DOIT Devkit V1, not Heltec board)
CS ----------- HSPI_CS0 (GPIO15)
RESET -------- GPIO32
DC/RS -------- GPIO33
SDI (MOSI) --- HSPI_MOSI (GPIO13)
SCK ---------- HSPI_CLK (GPIO14)
LED ---------- GPIO27 or +3V3 for no backlight adjustment
SDO (MISO) --- HSPI_MISO (GPIO12)
```

## SHT31-D Pins
```
SHT -> ESP32
------------
SDA -> SDA (GPIO21)
SCL -> SCL (GPIO22)
```
## Board Pinout
![ESP32 DOIT Devkit V1 pinout from ElectronicsHub](/doc/datasheets/ESP32-DOIT-Devkit-V1-Pinout.jpg)
