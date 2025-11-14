#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>

#include "stm32g4xx.h"
#include "uart.h"

#include "debug.h"

#ifdef DEBUG
extern void Error_Handler(void);
static UART_HandleTypeDef huart1;

int uart_init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef  PeriphClkInit = {0};

	// 配置UART控制器时钟
	HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInit);
	PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }
	
	__HAL_RCC_USART1_CLK_ENABLE(); // 使能USART1时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();  // 启用GPIOA时钟

	// 配置UART1参数
	huart1.Instance = USART1;                        // 选择使用的USART端口，例如USART1
	huart1.Init.BaudRate = 115200;                   // 设置波特率
	huart1.Init.WordLength = UART_WORDLENGTH_8B;     // 设置数据位长度为8位
	huart1.Init.StopBits = UART_STOPBITS_1;          // 设置停止位为1位
	huart1.Init.Parity = UART_PARITY_NONE;           // 设置无校验位
	huart1.Init.Mode = UART_MODE_TX_RX;              // 设置模式为发送和接收
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // 设置无硬件流控制
	huart1.Init.OverSampling = UART_OVERSAMPLING_16; // 设置过采样为16

	// 配置TX/RX引脚(PA9/PA10)
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}

	return 0;
}

//重定向printf
ssize_t _write(int file, const void *ptr, size_t len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

#else

int uart_init(void)
{
	return 0;
}

#endif //#ifdef DEBUG
