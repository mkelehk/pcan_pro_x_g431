#pragma once

#include <stdio.h>

extern void system_disable_irq();
extern void system_enable_irq();

//for debug
//#define DEBUG
#define DEBUG_LEVEL  0 /*0 1 2*/

#if defined(DEBUG)
#define PRINT_FAULT(fmt, args...) \
	do { \
		system_disable_irq(); \
		printf( "[ERROR][%8u] ", HAL_GetTick() ); \
		printf( fmt, ##args ); \
		printf( "\r\n" ); \
		system_enable_irq(); \
	} while (0)

#define PRINT_INFO(fmt, args...) \
	do { \
		system_disable_irq(); \
		printf( "[ INFO][%8u] ", HAL_GetTick() ); \
		printf(fmt, ##args); \
		printf("\r\n"); \
		system_enable_irq(); \
	} while (0)
#else
#define PRINT_FAULT(...)
#define PRINT_INFO(...)
#endif

#if defined(DEBUG) && (DEBUG_LEVEL > 0)
#define PRINT_DEBUG(fmt, args...) \
	do { \
		system_disable_irq(); \
		printf( "[DEBUG][%8u] ", HAL_GetTick() ); \
		printf(fmt, ##args); \
		printf("\r\n"); \
		system_enable_irq(); \
	} while (0)
#else
#define PRINT_DEBUG(...)
#endif

#if defined(DEBUG) && (DEBUG_LEVEL > 1)
#define PRINT_TRACE(fmt, args...) \
	do { \
		system_disable_irq(); \
		printf( "[TRACE][%8u] ", HAL_GetTick() ); \
		printf(fmt, ##args); \
		printf("\r\n"); \
		system_enable_irq(); \
	} while (0)
#else
#define PRINT_TRACE(...)
#endif

