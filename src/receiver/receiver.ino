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
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/inet.h>

//enable WPS or use hardcoded wifi credentials
//#define USE_WPS
//use OLED
//#define OLED_ENABLED

#ifndef USE_WPS
  #include "wifi_credentials.h"
#else
  #include "esp_wps.h"
#endif

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
#ifdef OLED_ENABLED
SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#endif

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
uint8_t deg_f_char[] =
{
  0b11100,
  0b10100,
  0b11100,
  0b00111,
  0b00100,
  0b00111,
  0b00100,
  0b00100
};
uint8_t deg_c_char[] =
{
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b00111,
  0b00100,
  0b00100,
  0b00111
};
uint8_t in_char[] =
{
  0b00000,
  0b00000,
  0b10000,
  0b00111,
  0b10101,
  0b10101,
  0b10101,
  0b10101
};
uint8_t hg_char[] =
{
  0b10100,
  0b11100,
  0b10100,
  0b00011,
  0b00101,
  0b00111,
  0b00001,
  0b00111
};

char wind_str[21] = "";
char temperature_str[13] = "";
char humidity_str[11] = "";
char pressure_str[21] = "";
char rainfall_str[21] = "";
char oled_line1[21] = "";
char oled_line2[21] = "";

lora_packet_t receivedData =
{
  //dummy values for testing
  //0xF00D, 3, 5.5F, 75.3F, 26.3F, 25.83F, 0.422F
};

//time display tracking
unsigned long prevDisplayTime = 0;
unsigned long lastUpdate = 0;
unsigned long raw_sec = 0;
unsigned long upd_sec = 0;
unsigned long upd_min = 0;

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
  lcd.createChar(2, deg_f_char);  // degF character for LCD
  lcd.createChar(3, deg_c_char);  // degC character for LCD
  lcd.createChar(4, in_char);     // in character for LCD
  lcd.createChar(5, hg_char);     // Hg character for LCD
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

#ifdef OLED_ENABLED
  oled.init();
  oled.clear();
  oled.screenRotate(ANGLE_180_DEGREE);
  oled.setFont(ArialMT_Plain_16);
#endif
  WiFi.setHostname(HOSTNAME);
  #ifdef USE_WPS
  pinMode(0,INPUT_PULLUP); //set PRG button as an input
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_STA);
  //WiFi.begin();
  #else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  #endif
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
    raw_sec = (now - lastUpdate) / 1000;
    upd_sec = raw_sec % 60;
    upd_min = (raw_sec / 60) % 60;

    //use the width part of the format specifier well since we're not calling clear() every time
    snprintf(wind_str, 21, "Wind: %2s %4.1f MPH\0", heading_map[receivedData.wind_heading], receivedData.wind_speed);
    snprintf(temperature_str, 14, "Temp:%5.1f\02\0", receivedData.temperature);
    snprintf(humidity_str, 11, " Hum:%3.0f%%\0", receivedData.humidity);
    snprintf(pressure_str, 21, "Prs:%5.2f\04\05 %2d:%02dago\0", receivedData.pressure, upd_min, upd_sec);
    snprintf(rainfall_str, 21, "Rain:%5.2f\04 last %2dh\0", receivedData.rainfall, receivedData.hours_up);

    lcd.home();
    lcd.print(wind_str);
    lcd.setCursor(0,1);
    lcd.print(temperature_str);
    lcd.print(humidity_str);
    lcd.setCursor(0,2);
    lcd.print(pressure_str);
    lcd.setCursor(0,3);
    lcd.print(rainfall_str);

    if(WiFi.status() != WL_CONNECTED)
    {
      #ifdef OLED_ENABLED
      snprintf(oled_line1, 21, "No WiFi!\0");
      #ifdef USE_WPS
      snprintf(oled_line2, 21, "Press PRG 4 WPS\0");
      Serial.printf("Not connected. Reason: %d\r\n", WiFi.status() );
      #else
      snprintf(oled_line2, 21, "\0");
      #endif
      #endif
    }
    else
    {
      timeClient.update();
      #ifdef OLED_ENABLED
      oled.clear();
      snprintf(oled_line1, 21, "Time: %02d:%02d:%02d\0", 
               timeClient.getHours(),
               timeClient.getMinutes(),
               timeClient.getSeconds()
              );
      
      uint32_t ip = WiFi.localIP();
      snprintf(oled_line2, 21, "IP: %s\0", inet_ntoa(ip) );
      #endif
    }
    #ifdef OLED_ENABLED
    oled.drawString(0, 0, oled_line1);
    oled.drawString(0, 14, oled_line2);
    oled.display();
    #endif
    #ifdef USE_WPS
    if (digitalRead(0) == 0)
    {
      wpsStart();
    }
    #endif
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
      Serial.printf("Temp:%4.1f°F", receivedData.temperature);
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

#ifdef USE_WPS
void wpsStart() {
  esp_wps_config_t config;
  memset(&config, 0, sizeof(esp_wps_config_t));
  config.wps_type = WPS_TYPE_PBC;
  strcpy(config.factory_info.manufacturer, "ESPRESSIF");
  strcpy(config.factory_info.model_number, CONFIG_IDF_TARGET);
  strcpy(config.factory_info.model_name, "ESPRESSIF IOT");
  strcpy(config.factory_info.device_name, "Weather Receiver");
  strcpy(config.pin, "00000000");

  Serial.println("WPS started.");

  esp_err_t err = esp_wifi_wps_enable(&config);
  if (err != ESP_OK) {
    Serial.printf("WPS Enable Failed: 0x%x: %s\n", err, esp_err_to_name(err));
    return;
  }

  err = esp_wifi_wps_start(0);
  if (err != ESP_OK) {
    Serial.printf("WPS Start Failed: 0x%x: %s\n", err, esp_err_to_name(err));
  }
}

void wpsStop() {
  esp_err_t err = esp_wifi_wps_disable();
  if (err != ESP_OK) {
    Serial.printf("WPS Disable Failed: 0x%x: %s\n", err, esp_err_to_name(err));
  }
}

void WiFiEvent(WiFiEvent_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START: Serial.println("Station Mode Started"); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("Connected to :" + String(WiFi.SSID()));
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from station, attempting reconnection");
      WiFi.reconnect();
      break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
      Serial.println("WPS Successful, stopping WPS and connecting to: " + String(WiFi.SSID()));
      wpsStop();
      delay(10);
      WiFi.begin();
      break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
      Serial.println("WPS Failed, retrying");
      wpsStop();
      wpsStart();
      break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
      Serial.println("WPS Timedout, retrying");
      wpsStop();
      wpsStart();
      break;
    default:                       break;
  }
}

#endif
