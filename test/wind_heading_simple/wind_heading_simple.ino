#include "Arduino.h"
#include "HT_SSD1306Wire.h"

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
char heading_str[30] = "";
uint16_t heading_raw = 0;

void displayHeading()
{
  oled.clear();
  heading_raw = analogRead(2);
  snprintf(heading_str, 20, "ADC Value: %5d\0", heading_raw);
  oled.drawString(0, 0, heading_str);
  oled.display();
}

void setup()
{
  oled.init();
  oled.clear();
  pinMode(2, ANALOG);
}

void loop()
{
  displayHeading();
  delay(250);
}

