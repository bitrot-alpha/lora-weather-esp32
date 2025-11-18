// Author: Bitrot_alpha
// stuff to make lora work
// specifies lora packet structure (with a struct)
// used by base_station and receiver

#ifndef _LORA_COMMON_H
#define _LORA_COMMON_H

//lora module pins on Heltec Wifi LoRa V3 board
//see https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32LA(F)_V3.1_Schematic_Diagram.pdf
#define LORA_NSS     8
#define LORA_DIO    14
#define LORA_RST    12
#define LORA_BSY    13
#define LORA_MOSI   10
#define LORA_MISO   11
#define LORA_SCK     9

//lora radio parameters
//see https://jgromes.github.io/RadioLib/class_s_x126x.html
//also see https://meshtastic.org/docs/overview/radio-settings/#data-rates
#define LORA_FREQ       915.0F  //Frequency in MHz
#define LORA_BW         125.0F  //Bandwidth in kHz
#define LORA_SF         7U      //spread factor from 7-12, low SF is higher data rate but less range
#define LORA_CR         5U      //coding rate denominator, from 4 (no error correction) to 8 (robust error correction).
#define LORA_TX_POWER   14      //radio transmit power in dBm.

//stuff in our LoRa packet
typedef struct LoraPacket
{
    //station key
    uint16_t station_key;
    //anemometer stuff
    uint8_t wind_heading;
    float wind_speed;
    //BME280 stuff
    float temperature, humidity, pressure;
    float rainfall;
    //how many hours since reset
    uint16_t hours_up;
} lora_packet_t;

#endif

