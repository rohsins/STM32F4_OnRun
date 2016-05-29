/*

**/

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"       
#include "RTE_Components.h" 
#include "stm32f4xx_hal_gpio.h"
#include "Driver_USART.h"
#include "Driver_SPI.h"


extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_SPI Driver_SPI1;
osThreadId tid_mySPI_Thread;

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

void USART1Initialize(void) {
	
	__USART1_CLK_ENABLE();
	
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE | ARM_USART_PARITY_NONE , 115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);
	
	Driver_USART1.Initialize(USART_Handler);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	
	Driver_USART1.Send("hey\n",4);
}

void mySPI_callback(uint32_t event)
{
    switch (event)
    {
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
        /* Success: Wakeup Thread */
        osSignalSet(tid_mySPI_Thread, 0x01);
        break;
    case ARM_SPI_EVENT_DATA_LOST:
        /*  Occurs in slave mode when data is requested/sent by master
            but send/receive/transfer operation has not been started
            and indicates that data is lost. Occurs also in master mode
            when driver cannot transfer data fast enough. */
        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
        break;
    case ARM_SPI_EVENT_MODE_FAULT:
        /*  Occurs in master mode when Slave Select is deactivated and
            indicates Master Mode Fault. */
        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
        break;
    }
}


void SPI1Initialize(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__GPIOE_CLK_ENABLE();

  GPIO_InitStruct.Pin   = GPIO_PIN_3;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	__SPI1_CLK_ENABLE();
	
	Driver_SPI1.Initialize(mySPI_callback);
	Driver_SPI1.Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8), 10000000);
//	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	Driver_SPI1.PowerControl(ARM_POWER_FULL);
}

void SPIThread(void const *argument) {
	osEvent evt;
	const uint8_t testdata_out[1] = {0x8F};
	uint8_t testdata_in[1];
	
	while (1)
    {
        /* SS line = ACTIVE = LOW */
//        Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
			  GPIOE->BSRR |= 1 << 19;
        /* Transmit some data */
        Driver_SPI1.Send(testdata_out, sizeof(testdata_out));
        /* Wait for completion */
        evt = osSignalWait(0x01, 100);
        if (evt.status == osEventTimeout) {
            __breakpoint(0); /* Timeout error: Call debugger */
        }
        /* SS line = INACTIVE = HIGH */
//        Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
				GPIOE->BSRR |= 1 << 3;
        /* SS line = ACTIVE = LOW */
//        Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
				GPIOE->BSRR |= 1 << 19;
        /* Receive 8 bytes of reply */
        Driver_SPI1.Receive(testdata_in, 1);
        evt = osSignalWait(0x01, 100);
        if (evt.status == osEventTimeout) {
            __breakpoint(0); /* Timeout error: Call debugger */
        }
        /* SS line = INACTIVE = HIGH */
//        Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
				GPIOE->BSRR |= 1 << 3;
    }
}
osThreadDef(SPIThread, osPriorityNormal, 1, 0);

void Blinky(void const *argument) {
	while (1) {
	//	HAL_GPIO_WritePin(GPIOD, 12, 1);
		GPIOD->BSRR |= 1 << 12 | 1 << 13 | 1 << 14 | 1 << 15;
		osDelay(10);
	//	HAL_GPIO_WitePin(GPIOD, 12, 0);
		GPIOD->BSRR |= 1 << 28| 1 << 29 | 1 << 30 | 1 << 31;
		osDelay(1000);
	}
}
osThreadDef(Blinky, osPriorityNormal, 1, 0);

int main (void) {

  osKernelInitialize();
  SystemClock_Config();
	LEDInitialize();
	USART1Initialize();
	SPI1Initialize();
	
  osKernelStart();
	osThreadCreate(osThread(Blinky), NULL);
	osThreadCreate(osThread(SPIThread), NULL);
	while (1) ;

}
