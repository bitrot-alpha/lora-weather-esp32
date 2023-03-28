#include "Arduino.h"
#include "HT_SSD1306Wire.h"

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
//const char hello[] = "Hello World!\0";

void myDisplay()
{
  oled.init();
  oled.clear();
  oled.drawString(0, 0, "Hello world!");
  oled.display();
}

void setup()
{
  myDisplay();  
}

void loop()
{

}

