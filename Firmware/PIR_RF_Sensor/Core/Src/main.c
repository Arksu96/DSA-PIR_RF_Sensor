/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RFM69.h"
#include "PIR.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//RFM69
uint8_t deviceType = 0;
uint8_t RFMinit = 0;
uint8_t RFMMessageOK = 0;
RFM69Stats_t RFStats;

//PIR
PIR_Event PIR[MAX_NUM_OF_EVENTS];
PIR_Occurance PIR_instance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  SPIInit(&hspi1);
  if(RFM69_initialize(RF69_868MHZ, 2, 1, &RFStats)){
	  RFMinit = 1;
	  RFM69_initMsg();
  }
  deviceType = RFM69_readReg(0x10);
  PIR_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Żeby nie było ujemnego miejsca w tabeli poniżej
	  if(PIR_instance.PIR_numOfEvents>0){
		  //Check if timeout on PIR
		  if(PIR_endOfMovement(&PIR[PIR_instance.PIR_numOfEvents-1])
			  || PIR_counterLimit(&PIR_instance)){
			  PIR_IRQstate(0);
			  //Calc movement duration
			  PIR_instance.PIR_movementDuration = HAL_GetTick() - PIR[0].PIR_start;
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			  //Send info to ESP
			  if(!PIR_sendRF(&PIR_instance, PIR)){
				  RFM69_send(RF_MASTER_ID, "Sent failed", sizeof(char)*11, false);
			  }
			  PIR_reset(&PIR_instance);
			  for(int i=0; i<20; i++){
				  memset(&PIR[i], 0, sizeof(PIR_Event));
			  }
			  PIR_IRQstate(1);
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == RF_Payload_Pin && checkInterruptStatus())
	{
		RFM69_ISRRx();
	}
	if((GPIO_Pin == PIR_H_Pin || GPIO_Pin == PIR_L_Pin) && PIR_IRQEnabled())
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)){
			PIR_DetectionCallback(GPIO_Pin, PIR_RISING, &PIR[PIR_instance.PIR_numOfEvents], &PIR_instance, HAL_GetTick());
		} else {
			PIR_DetectionCallback(GPIO_Pin, PIR_FALLING, &PIR[PIR_instance.PIR_numOfEvents], &PIR_instance, HAL_GetTick());
		}
	}
//	if(GPIO_Pin == PIR_H_Pin)
//	{
//		if(HAL_GPIO_ReadPin(PIR_H_GPIO_Port, PIR_H_Pin) == GPIO_PIN_SET){
//			PIR_DetectionCallback(PIR_H_Pin, PIR_RISING, &PIR[PIR_instance.PIR_numOfEvents], &PIR_instance);
//		} else {
//			PIR_DetectionCallback(PIR_H_Pin, PIR_FALLING, &PIR[PIR_instance.PIR_numOfEvents], &PIR_instance);
//		}
//	}
//	if(GPIO_Pin == PIR_L_Pin)
//	{
//		if(HAL_GPIO_ReadPin(PIR_L_GPIO_Port, PIR_L_Pin) == GPIO_PIN_SET){
//			PIR_DetectionCallback(PIR_L_Pin, PIR_RISING, &PIR[PIR_instance.PIR_numOfEvents], &PIR_instance);
//		} else {
//			PIR_DetectionCallback(PIR_L_Pin, PIR_FALLING, &PIR[PIR_instance.PIR_numOfEvents], &PIR_instance);
//		}
//	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
