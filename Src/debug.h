#pragma once

#include <stdio.h>

extern void system_disable_irq();
extern void system_enable_irq();
extern uint32_t HAL_GetTick(void);

// 定义打印级别
#define PRINT_LEVEL_NONE    -1      // 不输出打印信息
#define PRINT_LEVEL_FAULT   0       // 错误信息, 异常返回时添加
#define PRINT_LEVEL_DEBUG   1       // 调试信息, 普通调试时添加
#define PRINT_LEVEL_TRACE   2       // 堆栈跟踪, 频繁打印时添加

//设置调试打印等级， 值为PRINT_LEVEL_NONE时关闭调试打印
#define PRINT_LEVEL  PRINT_LEVEL_NONE /*PRINT_LEVEL_NONE/PRINT_LEVEL_FAULT/PRINT_LEVEL_DEBUG/PRINT_LEVEL_TRACE*/

#define SHELL_COLOR_DEFAULT         "\033[0m"    //打印完关闭所有颜色属性
#define SHELL_COLOR_ERROR           "\033[33m"   //黄色
#define SHELL_COLOR_DEBUG           "\033[36m"   //蓝色
#define SHELL_COLOR_TRACE           "\033[32m"   //绿色

#if (PRINT_LEVEL >= PRINT_LEVEL_FAULT)
#define PRINT_FAULT(fmt, args...) \
		do { \
			system_disable_irq(); \
			printf( "%s%08lu| FAULT | ", SHELL_COLOR_ERROR, (unsigned long)HAL_GetTick() ); \
			printf( fmt, ##args ); \
			printf( "%s\r\n", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)
#define PRINT_FAULT_BUFFER(buf, len) \
		do { \
				int i = 0; \
				system_disable_irq(); \
				printf( "%s%08lu| FAULT | (size: %u):\r\n", SHELL_COLOR_ERROR, (unsigned long)HAL_GetTick(), len ); \
				for(i = 0; i < len; ++i) \
				{ \
					printf( "%02x ", ((uint8_t *)buf)[i] ); \
					if(i % 16 == 15) \
					{ \
						printf( "\r\n" ); \
					} \
				} \
				printf( "%s", SHELL_COLOR_DEFAULT ); \
				system_enable_irq(); \
			} while (0)

#else
#define PRINT_FAULT(...)
#define PRINT_FAULT_BUFFER(buf, len)
#endif

#if (PRINT_LEVEL >= PRINT_LEVEL_DEBUG)
#define PRINT_DEBUG(fmt, args...) \
	do { \
			system_disable_irq(); \
			printf( "%s%08lu| DEBUG | ", SHELL_COLOR_DEBUG, (unsigned long)HAL_GetTick() ); \
			printf( fmt, ##args ); \
			printf( "%s\r\n", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)
#define PRINT_DEBUG_BUFFER(buf, len) \
	do { \
			int i = 0; \
			system_disable_irq(); \
			printf( "%s%08lu| DEBUG | (size: %u):\r\n", SHELL_COLOR_DEBUG, (unsigned long)HAL_GetTick(), len ); \
			for(i = 0; i < len; ++i) \
			{ \
				printf( "%02x ", ((uint8_t *)buf)[i] ); \
				if(i % 16 == 15) \
				{ \
					printf( "\r\n" ); \
				} \
			} \
			printf( "%s", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)
#else
#define PRINT_DEBUG(...)
#define PRINT_DEBUG_BUFFER(buf, len)
#endif

#if (PRINT_LEVEL >= PRINT_LEVEL_TRACE)
#define PRINT_TRACE(fmt, args...) \
	do { \
			system_disable_irq(); \
			printf( "%s%08lu| TRACE | ", SHELL_COLOR_TRACE, (unsigned long)HAL_GetTick() ); \
			printf( fmt, ##args ); \
			printf( "%s\r\n", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)
#define PRINT_TRACE_BUFFER(buf, len) \
	do { \
			int i = 0; \
			system_disable_irq(); \
			printf( "%s%08lu| TRACE | (size: %u):\r\n", SHELL_COLOR_TRACE, (unsigned long)HAL_GetTick(), len ); \
			for(i = 0; i < len; ++i) \
			{ \
				printf( "%02x ", ((uint8_t *)buf)[i] ); \
				if(i % 16 == 15) \
				{ \
					printf( "\r\n" ); \
				} \
			} \
			printf( "%s", SHELL_COLOR_DEFAULT ); \
			system_enable_irq(); \
		} while (0)
#else
#define PRINT_TRACE(...)
#define PRINT_TRACE_BUFFER(buf, len)
#endif

