// Author: Bitrot_alpha
// Pins of Heltec Wifi LoRa 32 (V3) board
// see https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA(F)_V3.1_Schematic_Diagram.pdf

#ifndef _HELTEC_BOARD_PINS_H
#define _HELTEC_BOARD_PINS_H

//lora module pins on Heltec Wifi LoRa V3 board
#define LORA_NSS     8
#define LORA_DIO    14
#define LORA_RST    12
#define LORA_BSY    13
#define LORA_MOSI   10
#define LORA_MISO   11
#define LORA_SCK     9

//OLED display pins
#define VEXT_CTRL   36
#define OLED_PWR    VEXT_CTRL
#define OLED_SDA    17
#define OLED_SCL    18
#define OLED_RST    21

//PRG button ("USER_Key")
#define BTN_ONBOARD     0
//onboard LED is already in the Arduino board def.
//#define LED_BUILTIN   35

//Battery charger circuit
#define ADC_CTRL    37
#define VBAT_PIN     1

#endif
