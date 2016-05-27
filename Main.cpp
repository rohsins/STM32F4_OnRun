/*

**/

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"       
#include "RTE_Components.h" 
#include "stm32f4xx_hal_gpio.h"
#include "Driver_USART.h"


extern  ARM_DRIVER_USART Driver_USART1;

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void LEDInitialize(void) {
	 GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void LEDUninitialize(void) {
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
}

void USART_Handler(uint32_t event) {
	
}

void Usart1Initialize(void) {
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE | ARM_USART_PARITY_NONE , 115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);
	
	Driver_USART1.Initialize(USART_Handler);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	
	Driver_USART1.Send("hey\n",4);
}

void Blinky(void const *argument) {
	while (1) {
	//	HAL_GPIO_WritePin(GPIOD, 12, 1);
		GPIOD->BSRR |= 1 << 12 | 1 << 13 | 1 << 14 | 1 << 15;
		osDelay(10);
	//	HAL_GPIO_WitePin(GPIOD, 12, 0);
		GPIOD->BSRR |= 1 << 28| 1 << 29 | 1 << 30 | 1 << 31;
		osDelay(10000);
	}
}
osThreadDef(Blinky, osPriorityNormal, 1, 0);

int main (void) {

  osKernelInitialize();
  SystemClock_Config();
	LEDInitialize();
	Usart1Initialize();
	
  osKernelStart();
	osThreadCreate(osThread(Blinky), NULL);
	while (1) ;

}
