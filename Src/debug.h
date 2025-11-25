#pragma once

//for debug
//#define DEBUG
#define PRINT_LEVEL  2 /*0 1 2 3*/

// 定义打印级别
#define PRINT_LEVEL_NONE    0       // 不输出打印信息
#define PRINT_LEVEL_FAULT   1       // 错误信息, 返回异常时使用
#define PRINT_LEVEL_DEBUG   2       // 调试信息, 普通调试时使用
#define PRINT_LEVEL_TRACE   3       // 跟踪信息, 频繁打印时使用

#if defined(DEBUG)
#define PRINT_FAULT(fmt, args...)  PrintDebug( PRINT_LEVEL_FAULT, fmt, ##args)
#define PRINT_DEBUG(fmt, args...)  PrintDebug( PRINT_LEVEL_DEBUG, fmt, ##args)
#define PRINT_TRACE(fmt, args...)  PrintDebug( PRINT_LEVEL_TRACE, fmt, ##args)

#define PRINT_FAULT_BUFFER(buffer, length) PrintDebugBuffer(PRINT_LEVEL_FAULT, buffer, length)
#define PRINT_DEBUG_BUFFER(buffer, length) PrintDebugBuffer(PRINT_LEVEL_DEBUG, buffer, length)
#define PRINT_TRACE_BUFFER(buffer, length) PrintDebugBuffer(PRINT_LEVEL_TRACE, buffer, length)
#else
#define PRINT_FAULT(...)
#define PRINT_DEBUG(...)
#define PRINT_TRACE(...)

#define PRINT_FAULT_BUFFER(buffer, length)
#define PRINT_DEBUG_BUFFER(buffer, length)
#define PRINT_TRACE_BUFFER(buffer, length)
#endif

