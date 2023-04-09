// Author: Bitrot_alpha
// lora receive test
// prints out data from LoRa to 2004A LCD
// obviously this pairs with lora send on another device

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "Wire.h"
#include "LiquidCrystal_PCF8574.h"
#include "lora_data.h"

//I2C Setup here
const uint8_t SDA_PIN = 41;
const uint8_t SCK_PIN = 42;
const uint8_t I2C_ADDR = 0x27;

LiquidCrystal_PCF8574 lcd(I2C_ADDR);
//our custom degree character
uint8_t degree_char[] = 
{
  0b00111,
  0b00101,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000, 
  0b00000
};

char wind_str[21] = "";
char temperature_str[21] = "";
char humidity_str[21] = "";
char pressure_str[21] = "";

lora_packet_t receivedData = 
{
  //dummy values for testing
  //3, 5.5F, 75.3F, 26.3F, 25.83F
};

long prevDisplayTime = 0;

const char * heading_map[9] = 
{
  "N\0", "NE\0",
  "E\0", "SE\0",
  "S\0", "SW\0",
  "W\0", "NW\0",
  "X\0"  
};

void setup()
{
  Wire1.begin(SDA_PIN, SCK_PIN);  // custom i2c port on ESP
  lcd.begin(20,4, Wire1);
  lcd.createChar(1, degree_char); // degree character for LCD
  lcd.setBacklight(255);
  //reset LCD
  //don't call clear() every refresh to avoid flashing
  lcd.home();
  lcd.clear();
}

void loop()
{
  long now = millis();

  //wait 1 second between outputs to display
  if(now - prevDisplayTime > 1000)
  {
    prevDisplayTime = now;
    //use the width part of the format specifier well since we're not calling clear() every time
    snprintf(wind_str, 21, "Wind: %2s %4.1f MPH\0", heading_map[receivedData.wind_heading], receivedData.wind_speed);
    snprintf(temperature_str, 21, "Temperature: %4.1f\01F\0", receivedData.temperature);
    snprintf(humidity_str, 21, "Humidity: %5.2f%%\0", receivedData.humidity);
    snprintf(pressure_str, 21, "Pressure: %4.2f inHg\0", receivedData.pressure);

    lcd.home();
    lcd.print(wind_str);
    lcd.setCursor(0,1);
    lcd.print(temperature_str);
    lcd.setCursor(0,2);
    lcd.print(humidity_str);
    lcd.setCursor(0,3);
    lcd.print(pressure_str);
  }
}
