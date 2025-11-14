// Author: Bitrot_alpha, nseidle
// Weather Station and LoRa transmitter
//    Much of the wind and rain sensor code is based off of nseidle's project
//    located at https://github.com/sparkfun/Wimp_Weather_Station
//    Buy Nate a beer if you see him ("beerware license")

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "lora_data.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <ESP32Time.h> //onboard rtc library
#include "HT_SSD1306Wire.h" //onboard OLED display

//Sleep time (50 seconds)
const unsigned int SLEEP_TIME = 50 * 1000000;
//Number of hours to keep persistent data like rainfall before resetting
const unsigned int RESET_MAX = 48;

//Values that are kept when sleeping and waking up
RTC_DATA_ATTR uint16_t bootcycles = 0;
RTC_DATA_ATTR unsigned long lastWakeup = 0;
RTC_DATA_ATTR unsigned long lastWakeupMillis = 0;
RTC_DATA_ATTR float raincount = 0.0F;

//Use the onboard RTC to keep track of time
ESP32Time rtc;

// **** LoRa setup section ****
#define RF_FREQUENCY                915000000 // Hz
#define TX_OUTPUT_POWER             14        // dBm
#define LORA_BANDWIDTH              0         // [0: 125 kHz, //  1: 250 kHz,
                                              //  2: 500 kHz, //  3: Reserved]
#define LORA_SPREADING_FACTOR       7         // [SF7..SF12]
#define LORA_CODINGRATE             1         // [1: 4/5, //  2: 4/6,
                                              //  3: 4/7, //  4: 4/8]
#define LORA_PREAMBLE_LENGTH        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false

//Set this to be the same on both base station and receivers
uint16_t STATION_KEY = 0xF00D;

lora_packet_t dataPacket = 
{
  //dummy values for testing
  //0xF00D, 5, 5.5F, 81.4F, 70.5F, 25.83F
};

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
// ****** END LoRa Setup ******

// **** SENSORS SECTION ****
//un-comment to get serial readout of barometer
//#define CAL_BAROMETER
//comment out to keep SI units instead of converting to US customary
#define CONV_IMPERIAL

Adafruit_BME280 bme;
Adafruit_SHT31 sht = Adafruit_SHT31(&Wire1);
const uint8_t SDA_PIN = 39;
const uint8_t SCK_PIN = 40;
const uint8_t BME_ADDRESS = 0x76;
const uint8_t SHT_ADDRESS = 0x44;
// calibrate the pressure value using an offset
// as demonstrated in this video: https://www.youtube.com/watch?v=Wq-Kb7D8eQ4
const float PRESSURE_OFFSET = 46.34F * 100.0F;

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

// onboard OLED display
// SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

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

	deltaTime /= 1000.0; //Convert to seconds

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
    adc_value += analogRead(WIND_DIR_PIN);
    delay(2);
  }
  adc_value /= 16;

  return adc_value;
}

const uint8_t determineHeading(uint16_t adc_value)
{
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

void OnTxDone( void )
{
  Radio.Sleep();
	Serial.println("TX done......");
}

void OnTxTimeout( void )
{
  Radio.Sleep();
  Serial.println("TX Timeout......");
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
      Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason);
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
    lastWakeup = rtc.getEpoch();
    lastWakeupMillis = rtc.getMillis();
  }
  
  Serial.begin(115200);
  print_wakeup_reason();
  //lora setup stuff
  Mcu.begin(WIFI_LORA_32_V3,SLOW_CLK_TPYE);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
  
  //setup wind sensors and rain gauge
  analogReadResolution(11);
  pinMode(WIND_DIR_PIN, ANALOG);
  pinMode(WIND_SPD_PIN, INPUT);
  attachInterrupt(WIND_SPD_PIN, wind_spd_IRQ, FALLING);
  attachInterrupt(RAIN_GAUGE_PIN, rainIRQ, FALLING);

  //I2C sensors setup
  Wire1.begin(SDA_PIN, SCK_PIN);
  if (!bme.begin(BME_ADDRESS, &Wire1) ) 
  {
    Serial.print("BME280 not found. Check wiring!\n");
    while(true)
    {
      delay(100);
    }
  }
  if (!sht.begin(SHT_ADDRESS) )
  {
    Serial.print("SHT31 not found. Check wiring!\n");
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
  dataPacket.temperature = ( (bme.readTemperature() * 1.8F) + 32 );
  dataPacket.pressure = ( (bme.readPressure() + PRESSURE_OFFSET) / 3386.0F);
  //dataPacket.humidity = bme.readHumidity();
  datapacket.humidity = sht.readHumidity();

  //wind stuff
  heading_raw = getADCValue();
  dataPacket.wind_speed = get_wind_speed();
  dataPacket.wind_heading = determineHeading(heading_raw);
  //rain gauge count
  dataPacket.rainfall = raincount;

	Serial.printf("\r\nsending packet\r\n");
  #ifdef CAL_BAROMETER
  Serial.printf("Barometer reading: %.2f hPa\r\n", dataPacket.pressure * 33.86F);
  Serial.printf("Offset: %.2f\r\n", PRESSURE_OFFSET * 0.01F);
  #endif
  Radio.Send( (uint8_t *) &dataPacket, sizeof(dataPacket) ); //send the package out
  Radio.IrqProcess();
  delay(100);
  Serial.printf("Processing done. Going to sleep now.\r\n");
  Serial.flush();
  Radio.Sleep();

  //set our last wakeup times
  lastWakeup = rtc.getEpoch();
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
