#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "lora_data.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Sleep time (50 seconds)
const unsigned int SLEEP_TIME = 50 * 1000000;

//LoRa setup section
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

//#define RX_TIMEOUT_VALUE            1000

lora_packet_t dataPacket = 
{
  //dummy values for testing
  5, 5.5F, 81.4F, 70.5F, 25.83F
};

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

// **** SENSORS SECTION ****

Adafruit_BME280 bme;
//anemometer pin
const uint8_t WIND_SPD_PIN = 4;
//wind heading pin
const uint8_t WIND_DIR_PIN = 2;
//if your anemometer is bouncy, set this to something other than 1 to "calibrate" it
const uint8_t BOUNCE_COMPENSATION = 2;

uint16_t heading_raw = 0;

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
  Serial.begin(115200);
  print_wakeup_reason();
  //lora setup stuff
  Mcu.begin();

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
  
  //setup wind sensors
  analogReadResolution(11);
  pinMode(WIND_DIR_PIN, ANALOG);
  pinMode(WIND_SPD_PIN, INPUT);
  attachInterrupt(WIND_SPD_PIN, wind_spd_IRQ, FALLING);
  
  //BME280 setup
  Wire1.begin(39, 40);
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
  //wait for 10 seconds to gather wind data
  delay(10000);

  // **** MAIN SECTION ****
  //bme measure
  bme.takeForcedMeasurement();
  dataPacket.temperature = ( (bme.readTemperature() * 1.8F) + 32 );
  dataPacket.pressure = (bme.readPressure() / 3386.0F);
  dataPacket.humidity = bme.readHumidity();

  //wind stuff
  heading_raw = getADCValue();
  dataPacket.wind_speed = get_wind_speed();
  dataPacket.wind_heading = determineHeading(heading_raw);

	Serial.printf("\r\nsending packet\r\n");
  Radio.Send( (uint8_t *) &dataPacket, sizeof(dataPacket) ); //send the package out
  Radio.IrqProcess();
  delay(100);
  Serial.printf("Processing done. Going to sleep now.\r\n");
  Radio.Sleep();
  //sleep for a minute at a time
  esp_sleep_enable_timer_wakeup(SLEEP_TIME);
  esp_deep_sleep_start();
  // **** END MAIN SECTION ****
}

void loop()
{
  //should never get here because we're sleeping
}
