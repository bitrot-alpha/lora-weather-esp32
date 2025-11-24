#include <Arduino.h>
#include <driver/ledc.h>
#include <arduino-sht.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

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
#define LEDC_DUTY               (1024) // Set duty to 50%. (2 ** 11) * 50% = 512
#define LEDC_FREQUENCY          (12000) // Frequency in Hertz. Set frequency at 12 kHz

TFT_eSPI tft;

SHTSensor sht(SHTSensor::SHT3X);

static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_color_t buf1[screenWidth * 10];
static lv_display_t * disp;

static lv_obj_t * temp_arc;
static lv_obj_t * text_label_temperature;
static lv_style_t style_temp;

static lv_obj_t * hum_arc;
static lv_obj_t * text_label_humidity;
static lv_style_t style_hum;

static lv_obj_t * droplet_img;
static lv_obj_t * thermometer_img;

static bool fade_up = false;

void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void pwm_init(void);
static void set_temp(lv_obj_t * label);
static void set_hum(lv_obj_t * label);
static void temp_timer_cb(lv_timer_t * timer);
static void set_brightness(lv_timer_t * timer);
void create_main_gui(void);

void setup()
{
    Serial.begin(115200);
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
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    // LVGL init
    lv_init();

    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(
        disp, buf1, NULL, sizeof(buf1),
        LV_DISPLAY_RENDER_MODE_PARTIAL);

    // // Test object
    static lv_style_t mylabel_style;

    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "TEMP HUMIDITY DEMO");
    lv_style_init(&mylabel_style);
    lv_style_set_text_font(&mylabel_style, &lv_font_montserrat_32);
    lv_style_set_text_color(&mylabel_style, lv_palette_main(LV_PALETTE_PINK));
    lv_obj_add_style(label, &mylabel_style, 0);
    lv_obj_set_align(label,LV_ALIGN_TOP_MID);

    //lv_example_arc_1();

    create_main_gui();
}

void loop()
{
    //lv_timer_handler();
    lv_task_handler();
    lv_tick_inc(50);
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

    // Update arc
    int arc_value = map(int(tempF), TEMP_ARC_MIN, TEMP_ARC_MAX, 0, 100);
    lv_arc_set_value(temp_arc, arc_value);

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

    lv_arc_set_value(hum_arc, (int)hum);

    // Update text
    char temp_text[16];
    snprintf(temp_text, sizeof(temp_text), "%d%%", (int)hum);
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

static void set_brightness(lv_timer_t * timer)
{
    if (fade_up)
    {
        Serial.println("BACKLIGHT FADE UP");
        ESP_ERROR_CHECK( ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, (uint32_t)(0.9 * 2048), 2000, LEDC_FADE_NO_WAIT) );
        fade_up = false;
    }
    else
    {
        Serial.println("BACKLIGHT FADE DOWN");
        ESP_ERROR_CHECK( ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, (uint32_t)(0.1 * 2048), 2000, LEDC_FADE_NO_WAIT) );
        fade_up = true;
    }
}

// Create the GUI elements
void create_main_gui(void)
{
    //temperature indicator
    temp_arc = lv_arc_create(lv_screen_active());
    lv_obj_set_size(temp_arc, 200, 200);
    lv_arc_set_rotation(temp_arc, 135);
    lv_arc_set_bg_angles(temp_arc, 0, 270);
    //arc alignment
    lv_obj_align(temp_arc, LV_ALIGN_CENTER, -110, 0);
    //set colors, knob is a background piece for some reason
    lv_obj_set_style_arc_color(temp_arc, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR);
    lv_obj_set_style_border_width(temp_arc, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_color(temp_arc, lv_palette_main(LV_PALETTE_PINK), LV_PART_KNOB);

    //humidity indicator
    hum_arc = lv_arc_create(lv_screen_active());
    lv_obj_set_size(hum_arc, 200, 200);
    lv_arc_set_rotation(hum_arc, 135);
    lv_arc_set_bg_angles(hum_arc, 0, 270);
    //arc alignment
    lv_obj_align(hum_arc, LV_ALIGN_CENTER, 110, 0);
    //set colors, knob is a background piece for some reason
    lv_obj_set_style_arc_color(hum_arc, lv_palette_main(LV_PALETTE_CYAN), LV_PART_INDICATOR);
    lv_obj_set_style_border_width(temp_arc, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_color(hum_arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_KNOB);

    //create our text labels
    text_label_temperature = lv_label_create(lv_screen_active());
    text_label_humidity = lv_label_create(lv_screen_active());

    //set font size and color
    lv_style_init(&style_temp);
    lv_style_set_text_font(&style_temp, &lv_font_montserrat_32);
    lv_style_set_text_color(&style_temp, lv_palette_main(LV_PALETTE_BLUE));
    lv_obj_add_style(text_label_temperature, &style_temp, 0);

    lv_label_set_text(text_label_temperature, "--.--"); // after style
    lv_obj_align(text_label_temperature, LV_ALIGN_CENTER,-110,20);

    lv_style_init(&style_hum);
    lv_style_set_text_font(&style_hum, &lv_font_montserrat_32);
    lv_style_set_text_color(&style_hum, lv_palette_main(LV_PALETTE_BLUE));
    lv_obj_add_style(text_label_humidity, &style_hum, 0);

    lv_label_set_text(text_label_humidity, "---%"); // after style
    lv_obj_align(text_label_humidity, LV_ALIGN_CENTER,110,20);

    // Create timer to update temperature every five seconds
    lv_timer_create(temp_timer_cb, 5000, nullptr);
    //fade up/down brightness every 10 seconds to demo PWM brightness control
    lv_timer_create(set_brightness, 10000, nullptr);

    //show icons inside arcs
    LV_IMAGE_DECLARE(droplet);
    droplet_img = lv_image_create(lv_screen_active());
    lv_image_set_src(droplet_img, &droplet);
    lv_obj_align(droplet_img, LV_ALIGN_CENTER, 110, -30);
    
    LV_IMAGE_DECLARE(thermometer);
    thermometer_img = lv_image_create(lv_screen_active());
    lv_image_set_src(thermometer_img, &thermometer);
    lv_obj_align(thermometer_img, LV_ALIGN_CENTER, -110, -30);
}
