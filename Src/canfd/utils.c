#include "utils.h"
#include "stm32g4xx.h"

static bitlimits limits = {0};

void utils_init(void)
{
    // Stupidly the files from ST do not declare any constants for the CAN bitrate ranges.
    // They only give us the macros IS_FDCAN_NOMINAL_PRESCALER(), IS_FDCAN_NOMINAL_TSEG1(), ...
    // But the host application needs these limits to calculate the bitrates (commands 's' and 'y')
    
    for (limits.nom_brp_max  = 2; IS_FDCAN_NOMINAL_PRESCALER(limits.nom_brp_max  +1); limits.nom_brp_max ++) { }
    for (limits.nom_seg1_max = 2; IS_FDCAN_NOMINAL_TSEG1    (limits.nom_seg1_max +1); limits.nom_seg1_max++) { }
    for (limits.nom_seg2_max = 2; IS_FDCAN_NOMINAL_TSEG2    (limits.nom_seg2_max +1); limits.nom_seg2_max++) { }
    for (limits.nom_sjw_max  = 2; IS_FDCAN_NOMINAL_SJW      (limits.nom_sjw_max  +1); limits.nom_sjw_max ++) { }
    
    for (limits.fd_brp_max   = 2; IS_FDCAN_DATA_PRESCALER   (limits.fd_brp_max   +1); limits.fd_brp_max  ++) { }
    for (limits.fd_seg1_max  = 2; IS_FDCAN_DATA_TSEG1       (limits.fd_seg1_max  +1); limits.fd_seg1_max ++) { }
    for (limits.fd_seg2_max  = 2; IS_FDCAN_DATA_TSEG2       (limits.fd_seg2_max  +1); limits.fd_seg2_max ++) { }
    for (limits.fd_sjw_max   = 2; IS_FDCAN_DATA_SJW         (limits.fd_sjw_max   +1); limits.fd_sjw_max  ++) { }
}

bitlimits* utils_get_bit_limits()
{
    return &limits;
}

// Convert a FDCAN DLC into the number of bytes in a message
int8_t utils_dlc_to_byte_count(uint32_t dlc_code)
{
    if (dlc_code <= 8)
        return dlc_code;
    
    switch (dlc_code)
    {
        case 0x9: return 12;
        case 0xA: return 16;
        case 0xB: return 20;
        case 0xC: return 24;
        case 0xD: return 32;
        case 0xE: return 48;
        case 0xF: return 64;
        default:  return -1;
    }
}

int8_t utils_byte_count_to_dlc(uint32_t byte_count)
{
         if (byte_count > 48) return 15;
    else if (byte_count > 32) return 14;
    else if (byte_count > 24) return 13;
    else if (byte_count > 20) return 12;
    else if (byte_count > 16) return 11;
    else if (byte_count > 12) return 10;
    else if (byte_count >  8) return  9;
    return byte_count;
}