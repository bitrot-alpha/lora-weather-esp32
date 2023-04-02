// Author: Bitrot_alpha
// dynamic print test
// prints out data from wind sensors to 2004A LCD
// most of this is the same as wind_sensors_combined

#include "Arduino.h"
#include "Wire.h"
#include "LiquidCrystal_PCF8574.h"

//I2C Setup here
const uint8_t SDA_PIN = 40;
const uint8_t SCK_PIN = 39;
const uint8_t I2C_ADDR = 0x27;

LiquidCrystal_PCF8574 lcd(I2C_ADDR);

//anemometer pin
const uint8_t WIND_SPD_PIN = 4;
//wind heading pin
const uint8_t WIND_DIR_PIN = 2;
//if your anemometer is bouncy, set this to something other than 1 to "calibrate" it
const uint8_t BOUNCE_COMPENSATION = 2;

uint16_t heading_raw = 0;
char adc_val[21] = "";
char heading_str[21] = "";
char revcount_str[21] = "";
char windmph_str[21] = "";
long prevDisplayTime = 0;

const char * heading_map[9] = 
{
  "N\0", "NE\0",
  "E\0", "SE\0",
  "S\0", "SW\0",
  "W\0", "NW\0",
  "X\0"  
};

//globals for tracking wind
long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile uint8_t windRevs = 0;


void wind_spd_IRQ()
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
	{
		lastWindIRQ = millis(); //Grab the current time
		windRevs++; //There is 1.492MPH for each click per second.
	}
}

float get_wind_speed()
{
	float deltaTime = millis() - lastWindCheck;

	deltaTime /= 1000.0; //Convert to seconds

	float windSpeed = (float)(windRevs / BOUNCE_COMPENSATION) / deltaTime;

	windRevs = 0; //Reset and start watching for new wind
	lastWindCheck = millis();

	windSpeed *= 1.492;

	return(windSpeed);
}

uint16_t getADCValue()
{
  //multisampling
  uint16_t adc_value = 0;
  for (int i = 0; i < 16; i++) 
  {
    adc_value += analogRead(WIND_DIR_PIN);
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
  Wire1.begin(SDA_PIN, SCK_PIN);  // custom i2c port on ESP
  lcd.begin(20,4, Wire1);
  lcd.setBacklight(255);
  //reset LCD
  //don't call clear() every refresh to avoid flashing
  lcd.home();
  lcd.clear();

  analogReadResolution(11);
  pinMode(WIND_DIR_PIN, ANALOG);
  pinMode(WIND_SPD_PIN, INPUT);
  attachInterrupt(WIND_SPD_PIN, wind_spd_IRQ, FALLING);
}

void loop()
{
  long now = millis();

  //wait 1 second between outputs to display
  if(now - prevDisplayTime > 1000)
  {
    prevDisplayTime = now;
    heading_raw = getADCValue();
    //use the width part of the format specifier well since we're not calling clear() every time
    snprintf(revcount_str, 21, "Anemometer Revs: %2u\0", (windRevs / BOUNCE_COMPENSATION) );
    snprintf(windmph_str, 21, "Wind MPH: %4.1f\0", get_wind_speed() );
    snprintf(adc_val, 21, "ADC Value: %5d\0", heading_raw);
    snprintf(heading_str, 21, "Wind Heading: %2s\0", determineHeading(heading_raw) );

    lcd.home();
    lcd.print(revcount_str);
    lcd.setCursor(0,1);
    lcd.print(windmph_str);
    lcd.setCursor(0,2);
    lcd.print(adc_val);
    lcd.setCursor(0,3);
    lcd.print(heading_str);
  }
}
