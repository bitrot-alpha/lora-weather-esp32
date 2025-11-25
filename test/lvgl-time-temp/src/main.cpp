#include <Arduino.h>
#include <driver/ledc.h>
#include <arduino-sht.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <esp_wps.h>
#include <Preferences.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/inet.h>

/*
------DISPLAY PINS------
DISPLAY -> ESP32 (DOIT Devkit V1, not Heltec board)
CS ----------- HSPI_CS0 (GPIO15)
RESET -------- GPIO32
DC/RS -------- GPIO33
SDI (MOSI) --- HSPI_MOSI (GPIO13)
SCK ---------- HSPI_CLK (GPIO14)
LED ---------- GPIO27 or +3V3 for no backlight adjustment
SDO (MISO) --- HSPI_MISO (GPIO12)
*/

#define SDA_PIN 21
#define SCL_PIN 22
#define BL_PIN 27

//ledc config (Display Backlight PWM)
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_IO          (27) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_11_BIT // Set duty resolution to 11 bits
#define LEDC_DUTY               (1024) // Set duty to 50%. (2 ** 11) * 50% = 1024
#define LEDC_FREQUENCY          (12000) // Frequency in Hertz. Set frequency at 12 kHz

TFT_eSPI tft;

SHTSensor sht(SHTSensor::SHT3X);

static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;
unsigned long handler_last_called = 0UL;

//static lv_color_t buf1[screenWidth * 8];
static uint32_t * disp_buf = nullptr;
static lv_display_t * disp;

static lv_obj_t * text_label_temperature;
static lv_obj_t * text_label_humidity;
static lv_obj_t * text_label_time;
static lv_obj_t * text_label_wifi;

static lv_obj_t * droplet_img;
static lv_obj_t * thermometer_img;

//general functions
void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void pwm_init(void);
void handle_button(void);
static void set_temp(lv_obj_t * label);
static void set_hum(lv_obj_t * label);
static void temp_timer_cb(lv_timer_t * timer);
static void upd_time_cb(lv_timer_t * timer);
static void set_brightness(unsigned int percent);
void create_main_gui(void);

//wifi stuff
void WiFiEvent(WiFiEvent_t event, arduino_event_info_t info);
void wpsStart();
void wpsStop();

//get the time from network
//refresh every 3 hours
//arg 3 is time offset (can use it for time zone) and arg 4 is refresh interval in milliseconds
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "north-america.pool.ntp.org", (-8 * 3600), ( (3 * 3600 * 1000) + 1000) );

//esp32 NVS flash, persistent data
Preferences savedData;

void setup()
{
    Serial.begin(115200);

    savedData.begin("wifidata", false);
    WiFi.setHostname("ESP32-clock");

    pinMode(0,INPUT_PULLUP); //set builtin BOOT button as an input
    WiFi.onEvent(WiFiEvent);
    WiFi.mode(WIFI_MODE_STA);
    if(savedData.getBool("WifiIsSetup") == true)
    {WiFi.begin();}

    Wire.begin(SDA_PIN, SCL_PIN);
    if(!sht.init(Wire))
    {
        Serial.println("couldn't find SHT!");
        Serial.flush();
        while(1);
    }
    if(!sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH))
    {
        Serial.println("Couldn't set SHT31 accuracy.");
    }
    // TFT init
    tft.init();
    tft.setSwapBytes(true);
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);

    // PWM backlight
    pwm_init();
    
    delay(1000);

    if(WiFi.status() == WL_CONNECTED)
    {
        timeClient.begin();
    }
    WiFi.setSleep(true);

    set_brightness(25);

    // LVGL init
    lv_init();

    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    //ints are 32 bit, or 4 bytes
    //VS code tells me buf_size is 38400
    //38400 * 4 = 153600 or 150kB
    //much more than 1/4 of the screen and the program dies
    //SRAM is supposedly slower than DRAM
    //but more buffer is better than less
    uint32_t buf_size = (uint32_t)(0.25F * screenWidth * screenHeight);
    disp_buf = (uint32_t *)lv_malloc(buf_size);
    lv_display_set_buffers(
        disp, disp_buf, NULL, buf_size,
        LV_DISPLAY_RENDER_MODE_PARTIAL);

    create_main_gui();
}

void loop()
{
    //lv_timer_handler();
    lv_task_handler();
    lv_tick_inc(50);

    if(!digitalRead(0))
    {
        handle_button();
    }

    delay(50);
}

static void pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    static ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 12 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    static ledc_channel_config_t ledc_channel = {
        .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_fade_func_install(0);
}

void handle_button(void)
{
    unsigned long now = millis();
    if ( (now - handler_last_called) < 800UL)
    {
        handler_last_called = now;
        return;
    }

    handler_last_called = now;
    wpsStart();
}

void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);

    tft.pushPixels((uint16_t*)px_map, w * h);

    tft.endWrite();
    lv_display_flush_ready(disp);
}

// Set the temperature value in the arc and text label
static void set_temp(lv_obj_t * label) {
    const int TEMP_ARC_MIN = -20;
    const int TEMP_ARC_MAX = 110;

    // Read sensor
    float tempF = (sht.getTemperature() * 1.8F) + 32;

    // Update text
    char temp_text[16];
    snprintf(temp_text, sizeof(temp_text), "%.1f \u00B0F", tempF);
    lv_label_set_text(label, temp_text);

    // Set text color based on temperature
    lv_color_t color;
    if (tempF <= 50.0) {
        color = lv_palette_main(LV_PALETTE_BLUE);
    } else if (tempF <= 75.6) {
        color = lv_palette_main(LV_PALETTE_GREEN);
    } else {
        color = lv_palette_main(LV_PALETTE_RED);
    }
    lv_obj_set_style_text_color(label, color, 0);

    // Force redraw
    lv_obj_invalidate(label);

    Serial.print("🌡 Temperature: ");
    Serial.println(temp_text);
}

static void set_hum(lv_obj_t * label) {
    // Read sensor
    float hum = sht.getHumidity();

    // Update text
    char temp_text[16];
    snprintf(temp_text, sizeof(temp_text), "%d %%", (int)hum);
    lv_label_set_text(label, temp_text);

    // Force redraw
    lv_obj_invalidate(label);

    Serial.print("💧 Humidity: ");
    Serial.println(temp_text);
}

static void temp_timer_cb(lv_timer_t * timer) 
{
    sht.readSample();
    set_temp(text_label_temperature);
    set_hum(text_label_humidity);
}

static void upd_time_cb(lv_timer_t * timer)
{
    if(WiFi.status() == WL_CONNECTED)
    {
        timeClient.update();
        lv_label_set_text(text_label_wifi, "WiFi: Yes");
        lv_label_set_text_fmt(text_label_time, "%02d:%02d", 
            timeClient.getHours(), timeClient.getMinutes() );
    }
    else
    {
        lv_label_set_text(text_label_wifi, "WiFi: No :(");
        lv_label_set_text(text_label_time, "XX:XX");
    }

    lv_obj_invalidate(text_label_wifi);
    lv_obj_invalidate(text_label_time);
}

static void set_brightness(unsigned int percent)
{
    if (percent > 100U)
    {return;}

    unsigned int duty_cycle = (unsigned int)(0.01F * (float)percent * 2048);
    Serial.printf("Set brightness %d%% %d\r\n", percent, duty_cycle);
    ESP_ERROR_CHECK( ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, (uint32_t)duty_cycle, 2000, LEDC_FADE_NO_WAIT) );
}

// Create the GUI elements
void create_main_gui(void)
{
    //create our text labels
    text_label_temperature = lv_label_create(lv_screen_active());
    text_label_humidity = lv_label_create(lv_screen_active());
    text_label_time = lv_label_create(lv_screen_active());
    text_label_wifi = lv_label_create(lv_screen_active());

    //show icons
    //LV_IMAGE_DECLARE(thermometer);
    thermometer_img = lv_image_create(lv_screen_active());
    lv_image_set_src(thermometer_img, "A:/thermometer.bin");
    lv_obj_align(thermometer_img, LV_ALIGN_BOTTOM_LEFT, 25, -10);

    //LV_IMAGE_DECLARE(droplet);
    droplet_img = lv_image_create(lv_screen_active());
    lv_image_set_src(droplet_img, "A:/droplet.bin");
    lv_obj_align(droplet_img, LV_ALIGN_BOTTOM_RIGHT, -125, -15);

    //set font, color, and alignment
    lv_obj_set_style_text_font(text_label_temperature, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(text_label_temperature, lv_palette_main(LV_PALETTE_BLUE),0);
    lv_label_set_text(text_label_temperature, "--.- °F");
    lv_obj_align_to(text_label_temperature, thermometer_img, LV_ALIGN_OUT_RIGHT_MID, 20,0);

    lv_obj_set_style_text_font(text_label_humidity, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(text_label_humidity, lv_palette_main(LV_PALETTE_BLUE),0);
    lv_label_set_text(text_label_humidity, " --- %");
    lv_obj_align_to(text_label_humidity, droplet_img, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

    lv_obj_set_style_text_font(text_label_time, &clock_184, 0);
    lv_obj_set_style_bg_color(text_label_time, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(text_label_time, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(text_label_time, 20, 0);
    lv_obj_set_style_text_color(text_label_time, lv_palette_main(LV_PALETTE_RED),0);
    lv_label_set_text(text_label_time, "XX:XX");
    lv_obj_align(text_label_time, LV_ALIGN_CENTER, 0, -40);

    lv_obj_set_style_text_font(text_label_wifi, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(text_label_wifi, lv_palette_main(LV_PALETTE_NONE),0);
    lv_label_set_text(text_label_wifi, "WiFi: X");
    lv_obj_align(text_label_wifi, LV_ALIGN_TOP_MID, 200, 0);

    // Create timer to update temperature every five seconds
    lv_timer_create(temp_timer_cb, 5000, nullptr);
    //timer to update time and wifi text
    lv_timer_create(upd_time_cb, 500, nullptr);
}

void wpsStart() {
  savedData.putBool("WifiIsSetup", false);
  esp_wps_config_t config;
  memset(&config, 0, sizeof(esp_wps_config_t));
  config.wps_type = WPS_TYPE_PBC;
  strcpy(config.factory_info.manufacturer, "ESPRESSIF");
  strcpy(config.factory_info.model_number, CONFIG_IDF_TARGET);
  strcpy(config.factory_info.model_name, "ESPRESSIF IOT");
  strcpy(config.factory_info.device_name, "ESP32 Clock");

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
