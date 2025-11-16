// Author: Bitrot_alpha
// stuff to make lora work
// specifies lora packet structure (with a struct)
// used by lora_send and lora_receive

#ifndef _LORA_COMMON_H
#define _LORA_COMMON_H

//LoRa modem setup stuff
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

