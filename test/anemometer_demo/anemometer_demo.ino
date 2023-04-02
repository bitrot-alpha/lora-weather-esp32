/*  Author: Bitrot_alpha, nseidle
 *  This is demo code for the anemometer.
 *    A quirk of the unit I'm using is that it double-switches for each pass of the magnet (reed switch sensor)..
 *    I've found this to happen AFTER using a RC filter for debouncing (it's slow enough to bypass the filter).
 *    The workaround I've applied is to divide the count value by 2.
 *
 *  Most of this is lifted from https://github.com/sparkfun/Wimp_Weather_Station (nseidle's code)
 */

#include "Arduino.h"
#include "HT_SSD1306Wire.h"

//anemometer pin
const uint8_t WIND_SPD_PIN = 4;
//if your anemometer is bouncy, set this to something other than 1 to "calibrate" it
const uint8_t BOUNCE_COMPENSATION = 2;

//display stuff
SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
char revcount_str[20] = "";
char windmph_str[20] = "";
long prevDisplayTime = 0;

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

void setup()
{
    //Serial.begin(115200);
    pinMode(WIND_SPD_PIN, INPUT);
    attachInterrupt(WIND_SPD_PIN, wind_spd_IRQ, FALLING);
    oled.init();
    oled.clear();
    oled.setFont(ArialMT_Plain_16);
}

void loop() 
{
  long now = millis();

  //wait 1 second between outputs to display
  if(now - prevDisplayTime > 1000)
  {
    prevDisplayTime = now;

    snprintf(revcount_str, 20, "Revs: %3u\0", (windRevs / BOUNCE_COMPENSATION) );
    snprintf(windmph_str, 20, "Wind MPH: %4.2f\0", get_wind_speed() );
    oled.clear();
    oled.drawString(0, 0, revcount_str);
    oled.drawString(0, 15, windmph_str);
    oled.display();
  }
}
