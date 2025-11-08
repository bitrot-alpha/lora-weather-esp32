# lora-weather-esp32
Weather station with ESP32 and LoRa

## Hardware
[MCU Board][mcu] - Heltec WiFi LoRa 32 v3 (ESP32 with LoRa and 0.96" OLED Display)  
* I'm using two of them to send and receive over LoRa. This is to provide indoor/remote display functions.  

Wind Speed, Wind Direction, and Rain Gauge kit - Available from [Argent Data][argent data kit] or [SparkFun][sparkfun kit].  
[HiLetGo BME280 breakout board][bme280 board] - measures temperature, humidity, and pressure. Not to be confused with the BM**P**280.  
[DFRobot MPPT 6V Solar Panel Controller][dfrobot board] - charges a LiPo battery with the help of the sun  
[Ambient Weather Solar Shield/Stevenson Screen][atmo sensor housing] - I didn't want to 3D print anything major for this project, so I found this on Amazon.  
[5V 1W Solar Panel][solar panel]  
[3.7V 1000mah LiPo battery][lipo battery]  
[RJ11 Jack Breakouts][rj11 jacks] - The wind and rain sensor kit uses RJ11 connectors.  
[Generic LCD2004A LCD module with I2C][lcd i2c module] - for the indoor display.  

[mcu]: https://heltec.org/project/wifi-lora-32-v3/
[sparkfun kit]: https://www.sparkfun.com/products/15901
[argent data kit]: https://www.argentdata.com/catalog/product_info.php?products_id=145
[bme280 board]: https://www.amazon.com/gp/product/B01N47LZ4P/
[dfrobot board]: https://www.amazon.com/gp/product/B07MML4YJV/
[atmo sensor housing]: https://www.amazon.com/gp/product/B09T4QDBN3/
[solar panel]: https://www.amazon.com/gp/product/B081YV4ZWD/
[lipo battery]: https://www.amazon.com/dp/B07CXNQ3ZR/
[rj11 jacks]: https://www.amazon.com/dp/B09HTRKRSH
[lcd i2c module]: https://www.amazon.com/dp/B01GPUMP9C/

## Tools
[Arduino IDE][arduino]  
[Heltec custom ESP32 library][heltec library] with support for the onboard OLED display and LoRa module  
> Note: As of March 2023, the Heltec Library has issues with the WiFi LoRa 32 v3 board (ESP32-S3). See [this issue][heltec library fix] for the fix.  
> **Don't install https://github.com/HelTecAutomation/Heltec_ESP32**! As of March 2023, it provides conflicting header files with v0.0.7 of the Aaron Lee library.  
> If you haven't installed the regular ESP32 Arduino library from Espressif, you can download `esptool` from [here][esptool git]  

[Adafruit Sensor library for BME280][adafruit bme280 library]  
[Matthias Hertel's PCF8574 LCD library][lcd library]  

[arduino]: https://www.arduino.cc/en/software
[heltec library]: https://docs.heltec.org/en/node/esp32/esp32_general_docs/quick_start.html 
[heltec library fix]: https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/issues/159
[esptool git]: https://github.com/espressif/esptool/releases/latest
[adafruit bme280 library]: https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout/arduino-test
[lcd library]: https://github.com/mathertel/LiquidCrystal_PCF8574
