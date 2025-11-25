#pragma once

#include <stdio.h>

extern void system_disable_irq();
extern void system_enable_irq();

//for debug
//#define DEBUG
#define DEBUG_LEVEL  2 /*0 1 2*/

#define SHELL_COLOR_DEFAULT         "\033[0m"    //打印完关闭所有颜色属性
#define SHELL_COLOR_ERROR           "\033[33m"   //黄色
#define SHELL_COLOR_DEBUG           "\033[36m"   //蓝色
#define SHELL_COLOR_TRACE           "\033[32m"   //绿色

#if defined(DEBUG)
#define PRINT_FAULT(fmt, args...) \
		do { \
			system_disable_irq(); \
			printf( "%s%08lu| FAULT | ", SHELL_COLOR_ERROR, HAL_GetTick() ); \
			printf( fmt, ##args ); \
			printf( "%s\r\n", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)
#else
#define PRINT_FAULT(...)
#endif

#if defined(DEBUG) && (DEBUG_LEVEL > 0)
#define PRINT_DEBUG(fmt, args...) \
	do { \
			system_disable_irq(); \
			printf( "%s%08lu| DEBUG | ", SHELL_COLOR_DEBUG, HAL_GetTick() ); \
			printf( fmt, ##args ); \
			printf( "%s\r\n", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)

#else
#define PRINT_DEBUG(...)
#endif

#if defined(DEBUG) && (DEBUG_LEVEL > 1)
#define PRINT_TRACE(fmt, args...) \
	do { \
			system_disable_irq(); \
			printf( "%s%08lu| TRACE | ", SHELL_COLOR_TRACE, HAL_GetTick() ); \
			printf( fmt, ##args ); \
			printf( "%s\r\n", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)

#else
#define PRINT_TRACE(...)
#endif

