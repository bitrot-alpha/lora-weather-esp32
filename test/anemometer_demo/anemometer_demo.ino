/*  Author: Bitrot_alpha
 *  This is demo code for the anemometer.
 *    A quirk of the unit I'm using is that it double-switches for each pass of the magnet (reed switch sensor)..
 *    I've found this to happen AFTER using a RC filter for debouncing (it's slow enough to bypass the filter).
 *    The workaround I've applied is to divide the count value by 2.
 */

#include "Arduino.h"
#include "HT_SSD1306Wire.h"

struct Switch 
{
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Switch switch1 = {47, 0, false};

void ARDUINO_ISR_ATTR isr(void* arg) 
{
    Switch* s = static_cast<Switch*>(arg);
    //disable interrupt to avoid repeat handling
    detachInterrupt(s->PIN);
    //flag our real interrupt handler
    s->pressed = true;
}

void ARDUINO_ISR_ATTR isr() {}

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
char display_str[30] = "";

void setup()
{
    //Serial.begin(115200);
    pinMode(switch1.PIN, INPUT);
    attachInterruptArg(switch1.PIN, isr, &switch1, FALLING);
    oled.init();
    oled.clear();
}

void loop() 
{
    if (switch1.pressed)
    {
      //clear flag
      switch1.pressed = false;
      switch1.numberKeyPresses += 1;
      snprintf(display_str, 30, "Anemometer rev count: %u\0", (switch1.numberKeyPresses / 2) );
      oled.clear();
      oled.drawString(0, 0, display_str);
      oled.display();
      //re-enable interrupt
      attachInterruptArg(switch1.PIN, isr, &switch1, FALLING);
    }
}
