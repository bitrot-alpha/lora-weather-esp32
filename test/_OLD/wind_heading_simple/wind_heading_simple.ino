/* Author: Bitrot_alpha
 *    Apparently the ADC of the ESP32 isn't very good. See https://forum.arduino.cc/t/fixing-the-non-linear-adc-of-an-esp32/699190
 *      and https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/adc.html#minimizing-noise for more details.
 */

#include "Arduino.h"
#include "HT_SSD1306Wire.h"

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
uint16_t heading_raw = 0;
char adc_val[30] = "";
char heading_str[30] = "";
const char * heading_map[9] = 
{
  "N\0", "NE\0",
  "E\0", "SE\0",
  "S\0", "SW\0",
  "W\0", "NW\0",
  "X\0"  
};

void displayHeading()
{
  oled.clear();
  heading_raw = getADCValue();
  snprintf(adc_val, 20, "ADC Value: %5d\0", heading_raw);
  oled.drawString(0, 0, adc_val);
  snprintf(heading_str, 20, "Heading: %s\0", determineHeading(heading_raw) );
  oled.drawString(0, 15, heading_str);
  oled.display();
}

uint16_t getADCValue()
{
  //multisampling
  uint16_t adc_value = 0;
  for (int i = 0; i < 16; i++) 
  {
    adc_value += analogRead(2);
    delay(2);
  }
  adc_value /= 16;

  return adc_value;
}
const char * determineHeading(uint16_t adc_value)
{
  //uses heading_map array, the if statements go in order from 0 to 7
  if (adc_value >= 1520 && adc_value <= 1540)
    return heading_map[0];
  if (adc_value >= 870 && adc_value <= 890)
    return heading_map[1];
  if (adc_value >= 150 && adc_value <= 170)
    return heading_map[2];
  if (adc_value >= 325 && adc_value <= 345)
    return heading_map[3];
  if (adc_value >= 525 && adc_value <= 545)
    return heading_map[4];
  if (adc_value >= 1200 && adc_value <= 1220)
    return heading_map[5];
  if (adc_value >= 1955 && adc_value <= 1975)
    return heading_map[6];
  if (adc_value >= 1775 && adc_value <= 1795)
    return heading_map[7];

  return heading_map[8];
}

void setup()
{
  oled.init();
  oled.clear();
  oled.setFont(ArialMT_Plain_16);
  analogReadResolution(11);
  pinMode(2, ANALOG);
}

void loop()
{
  displayHeading();
  delay(250);
}

