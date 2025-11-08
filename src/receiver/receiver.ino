// Author: Bitrot_alpha
// Weather station receiver
// prints out data from LoRa to 2004A LCD
// Pairs with base_station to display weather data

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "Wire.h"
#include "LiquidCrystal_PCF8574.h"
#include "HT_SSD1306Wire.h"
#include "lora_data.h"
//wifi stuff
#include "wifi_credentials.h"
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/inet.h>

#define HOSTNAME "WeatherStation"

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
const uint16_t STATION_KEY = 0xF00D;

static RadioEvents_t RadioEvents;
//END LoRa setup section

//I2C Setup here
const uint8_t SDA_PIN = 41;
const uint8_t SCK_PIN = 42;
const uint8_t I2C_ADDR = 0x27;

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

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
char time_str[21] = "";
char ip_str[21] = "";

lora_packet_t receivedData =
{
  //dummy values for testing
  //0xF00D, 3, 5.5F, 75.3F, 26.3F, 25.83F, 0.422F
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

//get the time from network
//refresh every 3 hours
//arg 3 is time offset (can use it for time zone) and arg 4 is refresh interval in milliseconds
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "north-america.pool.ntp.org", (-7 * 3600), ( (3 * 3600 * 1000) + 1000) );

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
  Mcu.begin(WIFI_LORA_32_V3,SLOW_CLK_TPYE);
  RadioEvents.RxDone = OnRxDone;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                              LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                              LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                              0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

  oled.init();
  oled.clear();
  oled.screenRotate(ANGLE_180_DEGREE);
  oled.setFont(ArialMT_Plain_16);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //wait 5 seconds for WiFi
  delay(5000);
  if(WiFi.status() == WL_CONNECTED)
  {
    timeClient.begin();
  }
  Serial.printf("Setup done!\r\n");
}

void loop()
{
  unsigned long now = millis();

  //wait 1 second between outputs to display
  if(now - prevDisplayTime > 1000)
  {
    prevDisplayTime = now;
    //use the width part of the format specifier well since we're not calling clear() every time
    snprintf(wind_str, 21, "Wind: %2s %4.1f MPH\0", heading_map[receivedData.wind_heading], receivedData.wind_speed);
    snprintf(temperature_str, 14, "Temp:%4.1f\01F\0", receivedData.temperature);
    snprintf(humidity_str, 11, " Hum:%3.0f%%\0", receivedData.humidity);
    snprintf(pressure_str, 21, "Pressure:%4.2f inHg\0", receivedData.pressure);
    snprintf(update_str, 21, "Rain:%03.2fin %3ds ago\0", receivedData.rainfall, ( (now - lastUpdate) / 1000) );

    lcd.home();
    lcd.print(wind_str);
    lcd.setCursor(0,1);
    lcd.print(temperature_str);
    lcd.print(humidity_str);
    lcd.setCursor(0,2);
    lcd.print(pressure_str);
    lcd.setCursor(0,3);
    lcd.print(update_str);

    if(WiFi.status() != WL_CONNECTED)
    {
      snprintf(time_str, 21, "No WiFi!\0");
      snprintf(ip_str, 21, "IP: X\0");
    }
    else
    {
      timeClient.update();
      oled.clear();
      snprintf(time_str, 21, "Time: %02d:%02d:%02d\0", 
               timeClient.getHours(),
               timeClient.getMinutes(),
               timeClient.getSeconds()
              );
      
      uint32_t ip = WiFi.localIP();
      snprintf(ip_str, 21, "IP: %s\0", inet_ntoa(ip) );
    }

    oled.drawString(0, 0, time_str);
    oled.drawString(0, 14, ip_str);
    oled.display();

    Radio.Rx(0); //put the LoRa radio into receive mode
  }

  Radio.IrqProcess(); //Wait for LoRa packet
}

//lora receive function
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  Serial.printf("\r\nreceived packet with rssi %d , length %d\r\n",rssi,size);

  //dumb pointer trick to save memory
  lora_packet_t* temp = (lora_packet_t*)(void*) payload;

  //check that what we received is from our weather station
  //only update values if it is
  if(size == sizeof(lora_packet_t))
  {
    if( temp->station_key == STATION_KEY )
    {
      lastUpdate = millis();
      memcpy( &receivedData, payload, sizeof(lora_packet_t) );
      Serial.printf("Wind: %2s %4.1f MPH\n\r", heading_map[receivedData.wind_heading], receivedData.wind_speed);
      Serial.printf("Temp:%4.1f F", receivedData.temperature);
      Serial.printf(" Hum:%3.0f%%\n\r", receivedData.humidity);
      Serial.printf("Pressure:%4.2f inHg\n\r", receivedData.pressure);
      Serial.printf("Rain:%05.3fin\n\r", receivedData.rainfall);
    }
    else
    {
      Serial.printf("Wrong key!\r\n");
    }
  }
  else
  {
    Serial.printf("Not data from weather station\r\n");
  }
  Radio.Sleep();
}
