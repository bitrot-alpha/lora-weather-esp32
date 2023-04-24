// Author: Bitrot_alpha
// stuff to make lora work
// specifies lora packet structure (with a struct)
// used by lora_send and lora_receive

#ifndef _LORA_COMMON_H
#define _LORA_COMMON_H

typedef struct LoraPacket
{
    //anemometer stuff
    uint8_t wind_heading;
    float wind_speed;
    //BME280 stuff
    float temperature, humidity, pressure;
    float rainfall;
} lora_packet_t;

#endif

