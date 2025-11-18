// Author: Bitrot_alpha
// Weather station receiver
// prints out data from LoRa to 2004A LCD
// NEW: data reporting (simple) on http server!
// Pairs with base_station to display weather data

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <RadioLib.h>
#include "lora_data.h"
#include "lcd_chars.h"
#include <Preferences.h>
//wifi stuff
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/inet.h>
//webserver stuff
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>

//enable WPS or use hardcoded wifi credentials
#define USE_WPS
//SI/US customary units toggle, comment out to use SI units
#define USE_IMPERIAL

#ifndef USE_WPS
  #include "wifi_credentials.h"
#else
  #include "esp_wps.h"
#endif

//functions
ICACHE_RAM_ATTR void loraPacketReceived(void);
void processPacket();
char * getSimpleData();
void WiFiEvent(WiFiEvent_t event, arduino_event_info_t info);
void wpsStart();
void wpsStop();
//------------------------------------

const char HOSTNAME[] = "WeatherStation";
const char MDNS_NAME[] = "weather-station";

//LoRa setup section
SX1262 radio = new Module(LORA_NSS, LORA_DIO, LORA_RST, LORA_BSY);
const uint16_t STATION_KEY = 0xF00D;
volatile bool flagReceived = false;
//END LoRa setup section

//I2C Setup here
const uint8_t SDA_PIN = 41;
const uint8_t SCK_PIN = 42;
const uint8_t I2C_ADDR = 0x27;

LiquidCrystal_PCF8574 lcd(I2C_ADDR);

char wind_str[21] = "";
char temperature_str[13] = "";
char humidity_str[11] = "";
char pressure_str[21] = "";
char rainfall_str[21] = "";

static lora_packet_t receivedData =
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

const char * heading_map[] = 
{
  "N\0", "NNE\0", "NE\0", "ENE\0",
  "E\0", "ESE\0", "SE\0", "SSE\0",
  "S\0", "SSW\0", "SW\0", "WSW\0",
  "W\0", "WNW\0", "NW\0", "NNW\0",
  "X\0"  
};

//get the time from network
//refresh every 3 hours
//arg 3 is time offset (can use it for time zone) and arg 4 is refresh interval in milliseconds
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "north-america.pool.ntp.org", (-7 * 3600), ( (3 * 3600 * 1000) + 1000) );

Preferences savedData;
//web server stuff
AsyncWebServer server(80);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);

  savedData.begin("wifidata", false);
  Serial.begin(115200);
  LittleFS.begin(true);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  Wire1.begin(SDA_PIN, SCK_PIN);  // custom i2c port on ESP
  lcd.begin(20,4, Wire1);

  lcd.createChar(1, (uint8_t *)degree_char); // degree character for LCD
  lcd.createChar(2, (uint8_t *)deg_f_char);  // degF character for LCD
  lcd.createChar(3, (uint8_t *)deg_c_char);  // degC character for LCD
  lcd.createChar(4, (uint8_t *)in_char);     // in character for LCD
  lcd.createChar(5, (uint8_t *)hg_char);     // Hg character for LCD
  lcd.createChar(6, (uint8_t *)mm_char);     // mm character for LCD
  lcd.createChar(7, (uint8_t *)hpa_char);     // hPa character for LCD
  lcd.setBacklight(255);
  //reset LCD
  //don't call clear() every refresh to avoid flashing
  lcd.home();
  lcd.clear();

  //lora init section
  int16_t res = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  if(RADIOLIB_ERR_NONE != res)
  {
    uint8_t ledOn = 0;
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

  WiFi.setHostname(HOSTNAME);
  #ifdef USE_WPS
  pinMode(0,INPUT_PULLUP); //set PRG button as an input
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_STA);
  if(savedData.getBool("WifiIsSetup") == true)
    WiFi.begin();
  #else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  #endif
  //wait 5 seconds for WiFi
  delay(5000);
  if(WiFi.status() == WL_CONNECTED)
  {
    timeClient.begin();
  }

  if(MDNS.begin(MDNS_NAME))
    Serial.println("MDNS start broadcast");
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
      request->send(200, "text/plain", getSimpleData());
  });
  
  server.onNotFound([](AsyncWebServerRequest *request)
  {
    request->send(404, "text/plain", "404: Not found");
  });

  server.begin();

  WiFi.setSleep(true);

  radio.setDio1Action(loraPacketReceived);
  radio.startReceive();

  Serial.printf("Setup done!\r\n");
}

void loop()
{
  if(flagReceived)
  {
    processPacket();
  }

  unsigned long now = millis();
  //wait 1 second between outputs to display
  if(now - prevDisplayTime > 1000)
  {
    prevDisplayTime = now;
    raw_sec = (now - lastUpdate) / 1000;
    upd_sec = raw_sec % 60;
    upd_min = (raw_sec / 60) % 60;

    #ifdef USE_IMPERIAL
    //use the width part of the format specifier well since we're not calling clear() every time
    snprintf(wind_str, 21, "Wind: %3s %5.1f MPH\0", heading_map[receivedData.wind_heading], receivedData.wind_speed);
    snprintf(temperature_str, 14, "Temp:%5.1f\02\0", receivedData.temperature);
    snprintf(humidity_str, 11, " Hum:%3.0f%%\0", receivedData.humidity);
    snprintf(pressure_str, 21, "Prs:%5.2f\04\05 %2d:%02dago\0", receivedData.pressure, upd_min, upd_sec);
    snprintf(rainfall_str, 21, "Rain:%5.2f\04 last %2dh\0", receivedData.rainfall, receivedData.hours_up);
    #else
    //use the width part of the format specifier well since we're not calling clear() every time
    snprintf(wind_str, 21, "Wind: %3s %5.1f kph\0", heading_map[receivedData.wind_heading], receivedData.wind_speed);
    snprintf(temperature_str, 14, "Temp:%5.1f\03\0", receivedData.temperature);
    snprintf(humidity_str, 11, " Hum:%3.0f%%\0", receivedData.humidity);
    snprintf(pressure_str, 21, "Prs:%5.2f\07 %2d:%02dago\0", receivedData.pressure, upd_min, upd_sec);
    snprintf(rainfall_str, 21, "Rain:%4.0f\06 last %2dh\0", receivedData.rainfall, receivedData.hours_up);
    #endif

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
      //blink the white led here?
    }
    else
    {
      timeClient.update();
    }

    #ifdef USE_WPS
    if (digitalRead(0) == 0)
    {
      wpsStart();
    }
    #endif
  }
}

ICACHE_RAM_ATTR void loraPacketReceived(void)
{
  flagReceived = true;
}

void processPacket()
{
  flagReceived = false;
  const unsigned int packetLen = sizeof(lora_packet_t);
  lora_packet_t temp;

  Serial.println();
  Serial.println("Packet received");
  Serial.printf("RSSI: %7.3f dBm\r\n", radio.getRSSI());
  Serial.printf("SNR: %7.3f dB\r\n",radio.getSNR());
  
  if(packetLen == radio.getPacketLength())
  {
    radio.readData((uint8_t *)&temp, packetLen);
    if(temp.station_key == STATION_KEY)
    {
      memcpy(&receivedData, &temp, sizeof(lora_packet_t));
      lastUpdate = millis();
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

  radio.startReceive();
}

#ifdef USE_WPS
void wpsStart() {
  savedData.putBool("WifiIsSetup", false);
  esp_wps_config_t config;
  memset(&config, 0, sizeof(esp_wps_config_t));
  config.wps_type = WPS_TYPE_PBC;
  strcpy(config.factory_info.manufacturer, "ESPRESSIF");
  strcpy(config.factory_info.model_number, CONFIG_IDF_TARGET);
  strcpy(config.factory_info.model_name, "ESPRESSIF IOT");
  strcpy(config.factory_info.device_name, "Weather Receiver");

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
    case ARDUINO_EVENT_WIFI_STA_START: 
      Serial.println("Station Mode Started");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("Connected to :" + String(WiFi.SSID()));
      Serial.print("Got IP: ");
      Serial.println(WiFi.localIP());
      savedData.putBool("WifiIsSetup", true);
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
    default:
      break;
  }
}
#endif

char * getSimpleData()
{
  const char * fmt_str = 
  "Weather on %s, last updated %02d:%02d ago\n"
  "Temp: %5.1f deg. F Hum: %3.0f%%\n"
  "Wind: %3s %4.1f MPH\n"
  "Pressure: %5.2f inHg\n"
  "Rain: %5.2f in. the last %2d hrs";

  static char ret_str[201] = "";
  snprintf(ret_str, 201, fmt_str,
    timeClient.getFormattedTime(),
    upd_min, upd_sec, receivedData.temperature, receivedData.humidity,
    (receivedData.wind_heading > -1 && receivedData.wind_heading < 17) ? heading_map[receivedData.wind_heading]:heading_map[16], 
    receivedData.wind_speed, receivedData.pressure, receivedData.rainfall, receivedData.hours_up);
  //Serial.println(fmt_str);
  return ret_str;
}
