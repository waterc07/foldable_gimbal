#pragma once

#include "std_typedef.h"
#include <sys/cdefs.h>

#define USB_SOF 0x3A
#define USB_EOF 0xAA

typedef struct
{
    uint8_t SOF;
    float pitch_angle;
    uint8_t is_updated;
    uint8_t CheckSum;
} __attribute__((packed)) vision_rx_t;

extern vision_rx_t vision_rx;
