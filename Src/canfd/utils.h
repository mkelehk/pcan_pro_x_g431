/*
    The MIT License
    Copyright (c) 2025 ElmueSoft / Nakanishi Kiyomaro / Normadotcom
    https://netcult.ch/elmue/CANable Firmware Update
*/

#pragma once

#include <stdint.h>

typedef struct
{
    uint32_t nom_brp_max;
    uint32_t nom_seg1_max;
    uint32_t nom_seg2_max;
    uint32_t nom_sjw_max;
    
    uint32_t fd_brp_max;
    uint32_t fd_seg1_max;
    uint32_t fd_seg2_max;
    uint32_t fd_sjw_max;
} bitlimits;

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

void        utils_init();
bitlimits*  utils_get_bit_limits();
int8_t      utils_dlc_to_byte_count(uint32_t hal_dlc_code);
int8_t      utils_byte_count_to_dlc(uint32_t byte_count);

