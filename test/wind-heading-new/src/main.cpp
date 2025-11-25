#include <Arduino.h>

// pins
const uint8_t WIND_DIR_PIN = 2;

// functions
const uint16_t getADCValue();
const uint8_t determineHeading(uint16_t adc_value);

// globals
uint16_t heading_raw = 0;
uint8_t heading_conv = 0;

const char * heading_map[] = 
{
  "N\0", "NNE\0", "NE\0", "ENE\0",
  "E\0", "ESE\0", "SE\0", "SSE\0",
  "S\0", "SSW\0", "SW\0", "WSW\0",
  "W\0", "WNW\0", "NW\0", "NNW\0",
  "X\0"  
};

void setup() 
{
  Serial.begin(115200);
  //setup wind vane
  analogReadResolution(12);
  pinMode(WIND_DIR_PIN, ANALOG);
}

void loop() 
{
  heading_raw = getADCValue();
  heading_conv = determineHeading(heading_raw);

  Serial.printf("ADC: %d Heading: %d\r\n%s\r\n", heading_raw, heading_conv, heading_map[heading_conv]);

  delay(2000);
}

const uint16_t getADCValue()
{
  //multisampling
  uint16_t adc_value = 0;
  for (int i = 0; i < 16; i++) 
  {
    //use ESP32-S3 factory calibration with analogReadMillivolts();
    adc_value += analogReadMilliVolts(WIND_DIR_PIN);
    delay(2);
  }
  adc_value /= 16;

  return adc_value;
}

const uint8_t determineHeading(uint16_t adc_value)
{ 
  //headings go clockwise every 22.5 degrees from N (0 deg) to NNW (337.5 deg)
  //bounds are plus minus 75mV, sadly some overlap
  if (adc_value >= 2458 && adc_value <= 2598)    return 0;
  if (adc_value >= 1233 && adc_value <= 1383)    return 1;
  if (adc_value >= 1412 && adc_value <= 1562)    return 2;
  if (adc_value >= 225 && adc_value <= 375)    return 4;
  if (adc_value >= 195 && adc_value <= 345)    return 3;
  if (adc_value >= 137 && adc_value <= 287)    return 5;
  if (adc_value >= 520 && adc_value <= 670)    return 6;
  if (adc_value >= 333 && adc_value <= 483)    return 7;
  if (adc_value >= 851 && adc_value <= 1001)    return 8;
  if (adc_value >= 714 && adc_value <= 864)    return 9;
  if (adc_value >= 1956 && adc_value <= 2106)    return 10;
  if (adc_value >= 1857 && adc_value <= 2007)    return 11;
  if (adc_value >= 2971 && adc_value <= 3121)    return 12;
  if (adc_value >= 2592 && adc_value <= 2742)    return 13;
  if (adc_value >= 2784 && adc_value <= 2934)    return 14;
  if (adc_value >= 2190 && adc_value <= 2340)    return 15;

  return 16;
}
