/**
 ******************************************************************************
 * @file    GPIO/GPIO_IOToggle/Src/main.c
 * @author  MCD Application Team
 * @version V1.1.3
 * @date    17-March-2017
 * @brief   This example describes how to configure and use GPIOs through
 *          the STM32F2xx HAL API.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f2xx_hal_gpio.h"
/** @addtogroup STM32F2xx_HAL_Examples
 * @{
 */

/** @addtogroup GPIO_IOToggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;
UART_HandleTypeDef UartHandle;
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC Compilers, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIMER1_INIT(void);
void pwm_start(char Dutycycle);
TIM_HandleTypeDef TIM1HANDLE;
TIM_OC_InitTypeDef sConfig;


/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	char Data[10]={0};
	unsigned char ch=0,i=0;
	int val=0;
	/* STM32F2xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();
	/* Configure the system clock to 120 MHz */
	SystemClock_Config();
	clock_init();
	gpio_init();
	uart_init();
	TIMER1_INIT();
	/* -1- Enable GPIOG, GPIOC and GPIOI Clock (to be able to program the configuration registers) */
	/* -3- Toggle PG.6, PG.8, PI.9 and PC.7 IOs in an infinite loop */
	printf("init\r\n");
	while (1)
	{	
}

void clock_init (void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

}


void pwm_start(char Dutycycle)
{
	sConfig.Pulse =  Dutycycle*60;
	HAL_TIM_PWM_ConfigChannel(&TIM1HANDLE, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TIM1HANDLE, &sConfig, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&TIM1HANDLE, &sConfig, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM1HANDLE, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM1HANDLE, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM1HANDLE, TIM_CHANNEL_1);
	printf("\r\nPWM STARTED WITH %d %% DUTYCYCLE\r\n",Dutycycle);
}


void gpio_init (void)
{
	/* -2- Configure PG.6, PG.8, PI.9 and PC.7 IOs in output push-pull mode to
	         drive external LEDs */
	GPIO_InitStruct.Pin = (GPIO_PIN_0 | GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void led_init ()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	/* Insert delay 100 ms */
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	/* Insert delay 100 ms */
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	/* Insert delay 100 ms */
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	/* Insert delay 100 ms */
	HAL_Delay(100);
}



void uart_init ()
{
	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
	      - Word Length = 8 Bits
	      - Stop Bit = One Stop bit
	      - Parity = ODD parity
	      - BaudRate = 115200 baud
	      - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance          = USARTx;
	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
}

void TIMER1_INIT(void)
{
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	TIM1HANDLE.Instance = TIM1;
	TIM1HANDLE.Init.Prescaler = 0;
	TIM1HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM1HANDLE.Init.Period = 5999;
	TIM1HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM1HANDLE.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&TIM1HANDLE);

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 1;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&TIM1HANDLE, &sBreakDeadTimeConfig);

	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.Pulse =  4200;//2400;//800;//5900;//2400;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&TIM1HANDLE, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TIM1HANDLE, &sConfig, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&TIM1HANDLE, &sConfig, TIM_CHANNEL_3);
}



/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 120000000
 *            HCLK(Hz)                       = 120000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 25
 *            PLL_N                          = 240
 *            PLL_P                          = 2
 *            PLL_Q                          = 5
 *            VDD(V)                         = 3.3
 *            Flash Latency(WS)              = 3
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 240;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
	/* User may add here some code to deal with this error */
	while(1)
	{
	}
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
