/*  Author: Bitrot_alpha
 *  This is a basic test of the BME280 sensor. I have the "HiLetGo" board.
 *    That board uses I2C address 0x76 (and has no protection diode..)
 */
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//replace this value with the given sea level pressure at your location
//example: https://metar-taf.com/KLAX -- scroll down to "Remarks" section and look for "sea level pressure"
#define SEALEVELPRESSURE_HPA (1017.8)

//serial debug output
//#define USING_SERIAL

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
Adafruit_BME280 bme;

float temperature = 0.0F;
float pressure = 0.0F;
float humidity = 0.0F;
float altitude = 0.0F;

char temp_str[20] = "";
char press_str[20] = "";
char humid_str[20] = "";
char alt_str[20] = "";

void setup()
{
  #ifdef USING_SERIAL
  Serial.begin(115200);
  //wait for serial
  while(!Serial);
  #endif
  Wire1.begin(39, 40);
  oled.init();
  oled.clear();
  oled.setFont(ArialMT_Plain_16);

  if (!bme.begin(0x76, &Wire1) ) 
  {
      Serial.print("BME280 not found. Check wiring!\n");
      while(true)
      {
        delay(100);
      }
  }
  //setup the sensor for weather station scenario per datasheet section 3.5
  //see the Adafruit library "advanced_settings" example
  bme.setSampling(
                  Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF
                 );
}


void loop() 
{ 
  bme.takeForcedMeasurement();
  temperature = bme.readTemperature();
  pressure = (bme.readPressure() / 100.0F);
  humidity = bme.readHumidity();
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  printValues();
  oled_display();
  
  delay(60000);
}

void oled_display()
{
  oled.clear();

  snprintf(temp_str, 20, "T: %4.2f °C\0", temperature);
  oled.drawString(0,0, temp_str);
  snprintf(press_str, 20, "P: %6.2f hPa\0", pressure);
  oled.drawString(0,16, press_str);
  snprintf(humid_str, 20, "RH: %5.2f%%\0", humidity);
  oled.drawString(0,32, humid_str);
  snprintf(alt_str, 20, "Alt: %5.0fM\0", altitude);
  oled.drawString(0,48, alt_str);

  oled.display();
}

void printValues() 
{
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();
}
