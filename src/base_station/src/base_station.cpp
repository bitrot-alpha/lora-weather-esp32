// Author: Bitrot_alpha, nseidle
// Weather Station and LoRa transmitter
//    Much of the wind and rain sensor code is based off of nseidle's project
//    located at https://github.com/sparkfun/Wimp_Weather_Station
//    Buy Nate a beer if you see him ("beerware license")
//#define USE_OLD_VAL

#include <Arduino.h>
#include "lora_data.h"
#include "heltec_board_pins.h"
#ifndef USE_OLD_VAL
//#include "adc_lut.h"
#endif
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SHTSensor.h>
#include <ESP32Time.h> //onboard rtc library

//Sleep time (50 seconds)
const unsigned int SLEEP_TIME = 50 * 1000000;
//Number of hours to keep persistent data like rainfall before resetting
const unsigned int RESET_MAX = 48;

//Values that are kept when sleeping and waking up
RTC_DATA_ATTR uint16_t bootcycles = 0;
//RTC_DATA_ATTR unsigned long lastWakeup = 0;
RTC_DATA_ATTR unsigned long lastWakeupMillis = 0;
RTC_DATA_ATTR float raincount = 0.0F;

//Use the onboard RTC to keep track of time
ESP32Time rtc;

// **** LoRa setup section ****
//Set this to be the same on both base station and receivers
uint16_t STATION_KEY = 0xF00D;
lora_packet_t dataPacket = 
{
  //dummy values for testing
  //0xF00D, 5, 5.5F, 81.4F, 70.5F, 25.83F
};
SX1262 radio = new Module(LORA_NSS, LORA_DIO, LORA_RST, LORA_BSY);
// ****** END LoRa Setup ******

// **** SENSORS SECTION ****
//un-comment to get serial readout of barometer
//#define CAL_BAROMETER
//un-comment to get serial readout of wind heading
//#define WIND_DEBUG
//comment out to keep SI units instead of converting to US customary
#define CONV_IMPERIAL

Adafruit_BME280 bme;
SHTSensor sht(SHTSensor::SHT3X);

const uint8_t SDA_PIN = 39;
const uint8_t SCK_PIN = 40;
const uint8_t BME_ADDRESS = 0x76;
// calibrate the pressure value using an offset
// as demonstrated in this video: https://www.youtube.com/watch?v=Wq-Kb7D8eQ4
const float PRESSURE_OFFSET = -1.65F;
//feet to meters: divide by 3.2[...]
//delete the divide portion if you're not using US customary units
const float ALTITUDE_METERS = 1346.0F / 3.28084F;

//anemometer pin
const uint8_t WIND_SPD_PIN = 4;
//wind heading pin
const uint8_t WIND_DIR_PIN = 2;
//if your anemometer is bouncy, set this to something other than 1 to "calibrate" it
const uint8_t BOUNCE_COMPENSATION = 2;
//rain gauge pin
const uint8_t RAIN_GAUGE_PIN = 5;
const gpio_num_t RAIN_GAUGE_ESP_PIN = GPIO_NUM_5;

uint16_t heading_raw = 0;

//globals for tracking wind
long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile uint8_t windRevs = 0;

volatile long raintime = 0;
long raininterval = 0;
long rainlast = 0;
// ****** END SENSORS SECTION ******

void wind_spd_IRQ()
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
	{
		lastWindIRQ = millis(); //Grab the current time
		windRevs++; //There is 1.492MPH (2.4 km/h) for each click per second.
	}
}

float get_wind_speed()
{
	float deltaTime = millis() - lastWindCheck;

	deltaTime /= 1000.0F; //Convert to seconds

	float windSpeed = (float)(windRevs / BOUNCE_COMPENSATION) / deltaTime;

	windRevs = 0; //Reset and start watching for new wind
	lastWindCheck = millis();

  #ifdef CONV_IMPERIAL
  windSpeed *= 1.492;
  #else
  windSpeed *= 2.4;
  #endif

	return(windSpeed);
}

uint16_t getADCValue()
{
  //multisampling
  uint16_t adc_value = 0;
  for (int i = 0; i < 16; i++) 
  {
    #ifdef USE_OLD_VAL
    adc_value += analogRead(WIND_DIR_PIN);
    #else
    //use ESP32-S3 factory calibration with analogReadMillivolts();
    adc_value += analogReadMilliVolts(WIND_DIR_PIN);
    #endif
    delay(2);
  }
  adc_value /= 16;

  return adc_value;
}

const uint8_t determineHeading(uint16_t adc_value)
{ 
  #ifdef USE_OLD_VAL
  //OLD VALUES (uncalibrated)
  //the headings go clockwise every 45 degrees from N (0 degrees) to NW (315 degrees)
  //bounds are 5% either side of what the ADC value should be
  if (adc_value >= 1454 && adc_value <= 1607)
    return 0;
  if (adc_value >= 834 && adc_value <= 922)
    return 1;
  if (adc_value >= 153 && adc_value <= 169)
    return 2;
  if (adc_value >= 320 && adc_value <= 354)
    return 3;
  if (adc_value >= 507 && adc_value <= 561)
    return 4;
  if (adc_value >= 1151 && adc_value <= 1273)
    return 5;
  if (adc_value >= 1867 && adc_value <= 2063)
    return 6;
  if (adc_value >= 1695 && adc_value <= 1873)
    return 7;

  return 8;
  #else

  //headings go clockwise every 22.5 degrees from N (0 deg) to NNW (337.5 deg)
  //bounds are plus minus 25mV
  if (adc_value >= 2508 && adc_value <= 2558)    return 0;
  if (adc_value >= 1283 && adc_value <= 1333)    return 1;
  if (adc_value >= 1462 && adc_value <= 1512)    return 2;
  //try and get the higher value first
  if (adc_value >= 275 && adc_value <= 325)    return 4;
  //these overlap sadly
  if (adc_value >= 245 && adc_value <= 295)    return 3;
  if (adc_value >= 187 && adc_value <= 237)    return 5;
  if (adc_value >= 570 && adc_value <= 620)    return 6;
  if (adc_value >= 383 && adc_value <= 433)    return 7;
  if (adc_value >= 901 && adc_value <= 951)    return 8;
  if (adc_value >= 764 && adc_value <= 814)    return 9;
  if (adc_value >= 2006 && adc_value <= 2056)    return 10;
  if (adc_value >= 1907 && adc_value <= 1957)    return 11;
  if (adc_value >= 3021 && adc_value <= 3071)    return 12;
  if (adc_value >= 2642 && adc_value <= 2692)    return 13;
  if (adc_value >= 2834 && adc_value <= 2884)    return 14;
  if (adc_value >= 2240 && adc_value <= 2290)    return 15;

  return 16;
  #endif
}

void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge
{
	raintime = millis(); // grab current time
	raininterval = raintime - rainlast; // calculate interval between this and last event

	if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
	{
    #ifdef CONV_IMPERIAL
    raincount += 0.011; //Each dump is 0.011" of water
    #else
    raincount += 0.2794; //Each dump is 0.2794mm of water
    #endif

		rainlast = raintime; // set up for next event
	}
}



void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program"); 
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\r\n",wakeup_reason);
      break;
  }
}

void setup() 
{
  //increment our boot counter
  bootcycles++;

  //reset rain count every RESET_MAX hours-ish
  if ( bootcycles > (RESET_MAX * 60) )
  {
    bootcycles = 0;
    raincount = 0.0F;
  }

  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
  {
    #ifdef CONV_IMPERIAL
    if (rtc.getMillis() - lastWakeupMillis > 10)
      raincount += 0.011;
    #else
    if (rtc.getMillis() - lastWakeupMillis > 10)
      raincount += 0.2794;
    #endif
    //record our last wakeup time
    //lastWakeup = rtc.getEpoch();
    lastWakeupMillis = rtc.getMillis();
  }
  
  Serial.begin(115200);
  print_wakeup_reason();

  //LoRa setup
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  radio.reset();
  int16_t res = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  if(RADIOLIB_ERR_NONE != res)
  {
    //why is my white led blinking??
    //check here: https://jgromes.github.io/RadioLib/group__status__codes.html
    uint8_t ledOn = 0;
    pinMode(LED_BUILTIN, OUTPUT);

    while(1)
    {
      ledOn = ~ledOn;
      Serial.println("Could not init SX1262 radio!!!");
      Serial.printf("Code: %d\r\n", res);
      digitalWrite(LED_BUILTIN, ledOn);
      delay(500);
    }
  }
  radio.setOutputPower(LORA_TX_POWER);

  //setup wind sensors and rain gauge
  analogReadResolution(12);
  pinMode(WIND_DIR_PIN, ANALOG);
  pinMode(WIND_SPD_PIN, INPUT);
  attachInterrupt(WIND_SPD_PIN, wind_spd_IRQ, FALLING);
  attachInterrupt(RAIN_GAUGE_PIN, rainIRQ, FALLING);

  //I2C sensors setup
  Wire1.begin(SDA_PIN, SCK_PIN);
  if (!bme.begin(BME_ADDRESS, &Wire1) ) 
  {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
    Serial.print("BME280 not found. Check wiring!\n");
    delay(5000);
    digitalWrite(LED_BUILTIN, 0);
    pinMode(LED_BUILTIN, INPUT);
    delay(1);
    //try again
    ESP.restart();
  }
  if (!sht.init(Wire1) )
  {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
    Serial.print("SHT31 not found. Check wiring!\n");
    delay(5000);
    digitalWrite(LED_BUILTIN, 0);
    pinMode(LED_BUILTIN, INPUT);
    delay(1);
    //try again
    ESP.restart();
  }
  
  //setup the sensor for weather station scenario per datasheet section 3.5
  //see the Adafruit library "advanced_settings" example
  bme.setSampling(
                  Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_NONE, // humidity
                  Adafruit_BME280::FILTER_OFF
                 );
  //wait for 10 seconds to gather wind data
  delay(10000);

  // **** MAIN SECTION ****
  dataPacket.station_key = STATION_KEY;
  dataPacket.hours_up = bootcycles / 60;
  //bme measure
  //measurements are converted to US/Imperial units
  bme.takeForcedMeasurement();
  #ifdef CONV_IMPERIAL
  dataPacket.temperature = ( (bme.readTemperature() * 1.8F) + 32 );
  dataPacket.pressure = ( (bme.seaLevelForAltitude(ALTITUDE_METERS, bme.readPressure() * 0.01) + PRESSURE_OFFSET) / 33.86F);
  #else
  dataPacket.temperature = ( (bme.readTemperature() );
  dataPacket.pressure = ( bme.seaLevelForAltitude(ALTITUDE_METERS, bme.readPressure() * 0.01) + PRESSURE_OFFSET );
  #endif
  //dataPacket.humidity = bme.readHumidity();
  dataPacket.humidity = sht.getHumidity();

  //wind stuff
  heading_raw = getADCValue();
  dataPacket.wind_speed = get_wind_speed();
  dataPacket.wind_heading = determineHeading(heading_raw);
  #ifdef WIND_DEBUG
  Serial.printf("ADC: %d Heading: %d\r\n", heading_raw, dataPacket.wind_heading);
  #endif
  //rain gauge count
  dataPacket.rainfall = raincount;

	Serial.printf("\r\nsending packet\r\n");
  #ifdef CAL_BAROMETER
  Serial.printf("Barometer reading: %.2f hPa, %.2f inHg\r\n", dataPacket.pressure * 33.86F, dataPacket.pressure);
  Serial.printf("Offset: %.2f\r\n", PRESSURE_OFFSET);
  #endif
  radio.transmit((const uint8_t *) &dataPacket, sizeof(dataPacket));

  delay(100);
  Serial.printf("Processing done. Going to sleep now.\r\n");
  Serial.flush();
  radio.sleep(false);

  //set our last wakeup times
  //lastWakeup = rtc.getEpoch();
  lastWakeupMillis = rtc.getMillis();

  //sleep for a minute at a time
  //wakeup when the rain gauge triggers
  esp_sleep_enable_ext0_wakeup(RAIN_GAUGE_ESP_PIN, 0);
  esp_sleep_enable_timer_wakeup(SLEEP_TIME);
  esp_deep_sleep_start();
  // **** END MAIN SECTION ****
}

void loop()
{
  //should never get here because we're sleeping
}
