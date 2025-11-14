#pragma once

// Two helper macros because the precompiler is not able to conacatenate a string with a constant
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

typedef enum 
{
    false = 0,
    true  = 1,
} bool;

// If command feedback is enabled these error codes are sent to the host.
// This enum is used for Slcan and for Candlelight.
// Slcan sends errors as "#1\r" which means FBK_InvalidCommand.
// Candlelight sends errors with command ELM_ReqGetLastError.
typedef enum // sent as 8 bit
{
    FBK_RetString = 1,            // The reponse has already been sent over USB --> no additional feedback. This is used only internally.
    FBK_Success   = 2,            // Command successfully executed
    // --------------------------    
    FBK_InvalidCommand    = '1',  // The command is invalid
    FBK_InvalidParameter,         // One of the parameters is invalid
    FBK_AdapterMustBeOpen,        // The command cannot be executed before opening the adapter
    FBK_AdapterMustBeClosed,      // The command cannot be executed after  opening the adapter
    FBK_ErrorFromHAL,             // The HAL from ST Microelectronics has reported an error
    FBK_UnsupportedFeature,       // The feature is not implemented or not supported by the board
    FBK_TxBufferFull,             // Sending is not possible because the buffer is full (only Slcan)
    FBK_BusIsOff,                 // Sending is not possible because the processor is blocked in the BusOff state
    FBK_NoTxInSilentMode,         // Sending is not possible because the adapter is in Bus Monitoring mode
    FBK_BaudrateNotSet,           // Opening the adapter is not possible if no baudrate has been set
    FBK_OptBytesProgrFailed,      // Programming the Option Bytes failed
    FBK_ResetRequired,            // The user must disconnect and reconnect the USB cable to enter boot mode
} eFeedback;

// If bus status is BUS_OFF both LED's (green + blue) are permanently ON
// This status is controlled only by hardware
// Slcan sends this in the error report "EXXXXXXXX\r"
typedef enum // sent as 4 bit
{
    BUS_StatusActive     = 0x00, // operational  (must be zero because this is not an error)
    BUS_StatusWarning    = 0x10, // set in can.c (>  96 errors)
    BUS_StatusPassive    = 0x20, // set in can.c (> 128 errors)
    BUS_StatusOff        = 0x30, // set in can.c (> 248 errors)
} eErrorBusStatus;

// If any of these flags is set, both LED's (green + blue) are permanently ON
// These flags are reset after sending them once to the host
// They are set again if the error is still present
// Slcan sends this in the error report "EXXXXXXXX\r"
// Candlelight sends this in a special error packet with a flag (legacy: CAN_ID_Error, Elm黃oft: MSG_Error)
typedef enum // sent as 8 bit 
{
    APP_CanRxFail       = 0x01, // the HAL reports an error receiving a CAN packet.
    APP_CanTxFail       = 0x02, // trying to send while in silent mode, while bus off or adaper not open or HAL error
    APP_CanTxOverflow   = 0x04, // a CAN packet could not be sent because the Tx FIFO + buffer are full (mostly because bus is passive).
    APP_UsbInOverflow   = 0x08, // a USB IN packet could not be sent because CAN traffic is faster than USB transfer.
    APP_CanTxTimeout    = 0x10, // A packet in the transmit FIFO was not acknowledged during 500 ms --> abort Tx and clear Tx buffer.
} eErrorAppFlags;

