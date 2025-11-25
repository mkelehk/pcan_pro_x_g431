#include <stm32g4xx_hal.h>
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "usb_device.h"
#include "uart.h"

#include "io_macro.h"
#include "pcanpro_variant.h"
#include "debug.h"

static uint32_t canfd_clock = 0;

void Error_Handler(void)
{
  /* reboot */
  HAL_Delay( 250 );
  HAL_NVIC_SystemReset();
  for(;;);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

static void pcan_io_config(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

//可参考STM32CubeMX(官方图形化配置工具)的时钟树图示,代码命名与时钟树图示高度对应,很好理解
void pcan_clock_config( void )
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	RCC_CRSInitTypeDef pInit = {0};

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	if (HAL_Init() != HAL_OK)
	{
		Error_Handler();
	}
	
	/** Configure the main internal regulator output voltage
	*/
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
#ifdef CANABLE2 //使用内部16MHz晶振,如标准版的CANable2.0等
	RCC_OscInitStruct.OscillatorType	  = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState			  = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State		  = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState		  = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource 	  = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM			  = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN			  = 20;
	RCC_OscInitStruct.PLL.PLLP			  = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ			  = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR			  = RCC_PLLR_DIV2;
#else //使用外部晶振,譬如预留有8MHz晶振的非标版CANable2.0等
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
#endif
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}

	// Initializes the peripherals clocks
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1; //使用APB1外设时钟作为CANFD控制器的时钟源
    PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
		Error_Handler();
	}

	/** Enable the SYSCFG APB clock
	*/
	__HAL_RCC_CRS_CLK_ENABLE();

	/** Configures CRS
	*/
	pInit.Prescaler = RCC_CRS_SYNC_DIV1;
	pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
	pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
	pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
	pInit.ErrorLimitValue = 34;
	pInit.HSI48CalibrationValue = 32;
	HAL_RCCEx_CRSConfig(&pInit);

    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    
    canfd_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN); // 80 MHz
}

uint32_t system_get_can_clock(void)
{
    return canfd_clock;
}

//动态调整CAN FD频率(PcanView.exe上允许设置的频率为80MHz 60MHz 40MHz 30MHz 24MHz 20MHz)
//调整方案是：系统时钟频率跟CAN FD时钟频率保持一致;
//也可以保持MCU系统时钟为高频率,没有测试过哪种方案比较优;
//选择这种方案只是觉得频率保持一致比较简单,缺点是设置CAN频率低，CPU运行代码的频率也跟着低,即CPU效率低不能发挥它应有的性能
//另外分频系数组合太多了,未考虑哪种约束条件下更优,这里只考虑只要能分频到想要的频率就行了
int pcan_can_set_canfdclock(uint32_t can_clock)
{
	uint32_t tickstart = 0;
	uint32_t PLLM = 0;
	uint32_t PLLN = 0;
	uint32_t PLLR = 0;
	uint32_t PLLQ = 2; //本工程未使用该时钟源,固定
	uint32_t PLLP = 2; //固定不可配置

	//跟当前频率一致时无需大动干戈
	if( can_clock == system_get_can_clock() )
	{
		return 0;
	}

	PRINT_DEBUG("Set CANFD Clk %lu", can_clock);
	
#ifdef CANABLE2
	//计算分频系数 SYSCLK = ( ( 16MHz / PLLM ) * PLLN ) / PLLR
	switch( can_clock )
	{
		case 80000000u:
		{
			PLLM = 2;
			PLLN = 20;
			PLLR = 2;
			break;
		}
		case 60000000u:
		{
			PLLM = 2;
			PLLN = 15;
			PLLR = 2;
			break;
		}
		case 40000000u:
		{
			PLLM = 2;
			PLLN = 20;
			PLLR = 4;
			break;
		}	
		case 30000000u:
		{
			PLLM = 2;
			PLLN = 15;
			PLLR = 4;
			break;
		}	
		case 24000000u:
		{
			PLLM = 2;
			PLLN = 12;
			PLLR = 4;
			break;
		}	
		case 20000000u:
		{
			PLLM = 2;
			PLLN = 15;
			PLLR = 6;
			break;
		}	
		default:
		{
			PRINT_FAULT("Not Support!!");
			return -1;
		}
	}
	
	//先使能HSI内部时钟源作为系统时钟,以免死机
	//__HAL_RCC_HSI48_ENABLE();
	//__HAL_RCC_HSI_ENABLE();
	__HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
	
	//关闭PLL
	__HAL_RCC_PLL_DISABLE();

	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI); //选择PLL的时钟源

	/* Wait till PLL is disabled */
	tickstart = HAL_GetTick();
	while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET)
	{
		if((HAL_GetTick() - tickstart ) > 2/*PLL_TIMEOUT_VALUE*/)
		{
			PRINT_FAULT("Disable PLL err");
			Error_Handler();
		}
	}

	//设置PLL分频系数
	__HAL_RCC_PLL_CONFIG( RCC_PLLSOURCE_HSI,
                             PLLM,
                             PLLN,
                             PLLP,
                             PLLQ,
                             PLLR );
#else
	//计算分频系数 SYSCLK = ( ( 8MHz / PLLM ) * PLLN ) / PLLR
	switch( can_clock )
	{
		case 80000000u:
		{
			PLLM = 1;
			PLLN = 20;
			PLLR = 2;
			break;
		}
		case 60000000u:
		{
			PLLM = 1;
			PLLN = 15;
			PLLR = 2;
			break;
		}
		case 40000000u:
		{
			PLLM = 1;
			PLLN = 20;
			PLLR = 4;
			break;
		}	
		case 30000000u:
		{
			PLLM = 1;
			PLLN = 15;
			PLLR = 4;
			break;
		}	
		case 24000000u:
		{
			PLLM = 1;
			PLLN = 12;
			PLLR = 4;
			break;
		}	
		case 20000000u:
		{
			PLLM = 1;
			PLLN = 15;
			PLLR = 6;
			break;
		}	
		default:
		{
			PRINT_FAULT("Not Support!!");
			return -1;
		}
	}
	
	//先使能HSI内部时钟源作为系统时钟,以免死机
	//__HAL_RCC_HSI48_ENABLE();
	__HAL_RCC_HSI_ENABLE();
	__HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

	//关闭PLL
	__HAL_RCC_PLL_DISABLE();

	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE); //选择PLL的时钟源
	__HAL_RCC_HSE_CONFIG(RCC_HSE_ON);                  //

	/* Wait till PLL is disabled */
	tickstart = HAL_GetTick();
	while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET)
	{
		if((HAL_GetTick() - tickstart ) > 2/*PLL_TIMEOUT_VALUE*/)
		{
			PRINT_FAULT("Disable PLL err");
			Error_Handler();
		}
	}
	
	//设置PLL分频系数
	__HAL_RCC_PLL_CONFIG( RCC_PLLSOURCE_HSE,
                             PLLM,
                             PLLN,
                             PLLP,
                             PLLQ,
                             PLLR );
#endif

	//重新使能PLL
	__HAL_RCC_PLL_ENABLE();

	/* Wait till PLL is ready */
	tickstart = HAL_GetTick();
	while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET)
	{
		if((HAL_GetTick() - tickstart ) > 2/*PLL_TIMEOUT_VALUE*/)
		{
			PRINT_FAULT("Enable PLL err");
			Error_Handler();
		}
	}

	//切换回PLL时钟源作为系统时钟
	__HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);

	//由于没有使用HAL_RCC_OscConfig/HAL_RCC_ClockConfig等上层接口,
	//需要显式地调用同步更新HAL库里RCC相关的全局变量(如SystemCoreClock等)
	SystemCoreClockUpdate();
	
	//根据新的系统时钟更新PCAN时间戳计数器
	TIM2->PSC   = (SystemCoreClock / 1000000) - 1;

	//更新CANFD时钟值
	canfd_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
	if(canfd_clock != can_clock)
	{
		PRINT_FAULT("Get CANFD Clk %lu != %lu", canfd_clock, can_clock);
		Error_Handler();
	}

	PRINT_DEBUG("SystemCoreClock = %lu, canfd_clock = %lu", SystemCoreClock, canfd_clock);
	
	return 0;
}

// ARM's
// "Application Note 321 ARM Cortex-M Programming Guide to Memory Barrier Instructions"
// (from https://developer.arm.com/documentation/dai0321/latest) says that
// the ISBs are actually necessary on Cortex-M0 to avoid a 2-instruction
// delay in the effect of enabling and disabling interrupts.
// That probably doesn't matter here, but it's hard to say what the compiler
// will put in those 2 instructions so it's safer to leave it. The DSB isn't
// necessary on Cortex-M0, but it's architecturally required so we'll
// include it to be safe.
//
// The "memory" and "cc" clobbers tell GCC to avoid moving memory loads or
// stores across the instructions. This is important when an interrupt and the
// code calling disable_irq/enable_irq share memory. The fact that these are
// non-inlined functions probably forces GCC to flush everything to memory
// anyways, but trying to outsmart the compiler is a bad strategy (you never
// know when somebody will turn on LTO or something).

void system_disable_irq()
{
	__disable_irq();
    __DSB(); // Data Synchronization Barrier
	__ISB(); // Instruction Synchronization Barrier
}

void system_enable_irq()
{
	__enable_irq();
	__ISB(); // Instruction Synchronization Barrier
}


int main(void)
{
  pcan_clock_config();
  pcan_io_config();
  pcan_timestamp_init();
  uart_init();
  
  PRINT_DEBUG("Firmware compile time:%s %s", __DATE__, __TIME__);
  
  pcan_led_init();
  pcan_led_set_mode( LED_STAT, LED_MODE_BLINK_FAST, 0xFFFFFFFF );
  pcan_protocol_init();
  pcan_usb_device_init();

  for(;;)
  {
    pcan_usb_device_poll();
    pcan_can_poll();
    pcan_protocol_poll();
    pcan_led_poll();
  }
}
