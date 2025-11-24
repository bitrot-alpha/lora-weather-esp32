// Author: Bitrot_alpha
// lora receive test
// prints out data from LoRa to 2004A LCD
// obviously this pairs with lora send on another device

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "Wire.h"
#include "LiquidCrystal_PCF8574.h"
#include "lora_data.h"

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

#define RX_TIMEOUT_VALUE            1000
//const uint8_t BUFFER_SIZE = sizeof(lora_packet_t);

static RadioEvents_t RadioEvents;
//END LoRa setup section

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
char temperature_str[13] = "";
char humidity_str[11] = "";
char pressure_str[21] = "";
char update_str[21] = "";

lora_packet_t receivedData =
{
  //dummy values for testing
  //3, 5.5F, 75.3F, 26.3F, 25.83F
};

long prevDisplayTime = 0;
long lastUpdate = 0;

const char * heading_map[9] = 
{
  "N\0", "NE\0",
  "E\0", "SE\0",
  "S\0", "SW\0",
  "W\0", "NW\0",
  "X\0"  
};

//bool lora_idle = true;

void setup()
{
  Serial.begin(115200);
  Wire1.begin(SDA_PIN, SCK_PIN);  // custom i2c port on ESP
  lcd.begin(20,4, Wire1);
  lcd.createChar(1, degree_char); // degree character for LCD
  lcd.setBacklight(255);
  //reset LCD
  //don't call clear() every refresh to avoid flashing
  lcd.home();
  lcd.clear();

  //lora init section
  Mcu.begin();
  RadioEvents.RxDone = OnRxDone;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                              LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                              LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                              0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

  Serial.printf("Setup done!\r\n");
}

void loop()
{
  long now = millis();

  //wait 1 second between outputs to display
  if(now - prevDisplayTime > 10000)
  {
    prevDisplayTime = now;
    //use the width part of the format specifier well since we're not calling clear() every time
    snprintf(wind_str, 21, "Wind: %2s %4.1f MPH\0", heading_map[receivedData.wind_heading], receivedData.wind_speed);
    snprintf(temperature_str, 14, "Temp:%4.1f\01F\0", receivedData.temperature);
    snprintf(humidity_str, 11, " Hum:%3.0f%%\0", receivedData.humidity);
    snprintf(pressure_str, 21, "Pressure: %4.2f inHg\0", receivedData.pressure);

    lcd.home();
    lcd.print(wind_str);
    lcd.setCursor(0,1);
    lcd.print(temperature_str);
    //lcd.setCursor(0,2);
    lcd.print(humidity_str);
    lcd.setCursor(0,2);
    lcd.print(pressure_str);
    snprintf(update_str, 21, "Updated %4d sec ago\0", ( (now - lastUpdate) / 1000) );
    lcd.setCursor(0,3);
    lcd.print(update_str);

    Radio.Rx(0); //put the LoRa radio into receive mode
  }
  else if (now - prevDisplayTime > 1000)
  {
    snprintf(update_str, 21, "Updated %4d sec ago\0", ( (now - lastUpdate) / 1000) );
    lcd.setCursor(0,3);
    lcd.print(update_str);
  }

  Radio.IrqProcess(); //Wait for LoRa packet
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  lastUpdate = millis();
  memcpy( &receivedData, payload, sizeof(lora_packet_t) );
  Radio.Sleep();
  Serial.printf("\r\nreceived packet with rssi %d , length %d\r\n",rssi,size);
}
