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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_16x2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t quadrant_I[8] 		= {0x00,0x00,0x00,0x00,0x03,0x07,0x0F,0x1F};
uint8_t quadrant_II[8] 		= {0x00,0x00,0x00,0x00,0x18,0x1C,0x1E,0x1F};
uint8_t quadrant_III[8] 	= {0x1F,0x0F,0x07,0x03,0x00,0x00,0x00,0x00};
uint8_t quadrant_IV[8] 		= {0x1F,0x1E,0x1C,0x18,0x00,0x00,0x00,0x00};
uint8_t halfSmileLeft1[8] 	= {0x04,0x08,0x14,0x03,0x00,0x00,0x00,0x00};
uint8_t halfSmileLeft2[8] 	= {0x00,0x00,0x00,0x00,0x18,0x07,0x03,0x03};
uint8_t halfSmileRight1[8] = {0x00,0x00,0x00,0x00,0x03,0x1C,0x18,0x18};
uint8_t halfSmileRight2[8] = {0x04,0x02,0x05,0x18,0x00,0x00,0x00,0x00};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// config for HCSR04
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
// config for Module drive car
#define IN1_PIN GPIO_PIN_4
#define IN1_PORT GPIOA
#define IN2_PIN GPIO_PIN_5
#define IN2_PORT GPIOA
#define IN3_PIN GPIO_PIN_6
#define IN3_PORT GPIOA
#define IN4_PIN GPIO_PIN_7
#define IN4_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t 	pMillis;
uint32_t 	Value_1 = 0;
uint32_t 	Value_2 = 0;
uint16_t 	Distance = 0;
uint8_t		DistLimit = 20;

char Rx_data[1];
uint8_t state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HCSR04();
void GroupInfoDisplay();
void SmileFace();
void CautionDisplay();
void DungXe();
void DiTien();
void DiLui();
void QuayTrai();
void QuayPhai();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
Note : 	+ while state = 0 => Car stop
				+ while state = 1 => Car Go Straight
				+ while state = 2 => Car Go Backward
				+ while state = 3 => Car Go Turn Right
				+ while state = 4 => Car Go Turn Left
				and GPIO Logic 0 is ON, Logic 1 is OFF
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		
{
  if(huart->Instance==USART2)
	{
		if(Rx_data[0]=='0') state = 0;
		if(Rx_data[0]=='1') state = 1;
		if(Rx_data[0]=='2') state = 2;
		if(Rx_data[0]=='3') state = 3;
		if(Rx_data[0]=='4') state = 4;
		HAL_UART_Receive_IT(&huart2, (uint8_t*)Rx_data,1);
	}
}

void DieuKhienXe()
{
	if(state==0)
	{
		DungXe();
	}else if(state==1)
	{
		DiTien();
	}else if(state==2)
	{
		DiLui();
	}else if(state==3)
	{
		QuayPhai();
	}else if(state==4)
	{
		QuayTrai();
	}else {
		DungXe();
	}
}
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
	Lcd_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//-----------------------------Create char display LCD--------------------------------
	Lcd_create_custom_char(0,quadrant_I);
	Lcd_create_custom_char(1,quadrant_II);
	Lcd_create_custom_char(2,quadrant_III);
	Lcd_create_custom_char(3,quadrant_IV);
	Lcd_create_custom_char(4,halfSmileLeft1);
	Lcd_create_custom_char(5,halfSmileLeft2);
	Lcd_create_custom_char(6,halfSmileRight1);
	Lcd_create_custom_char(7,halfSmileRight2);
	//------------------------------------------------------------------------------------
	HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(TRIG_PORT,TRIG_PIN, GPIO_PIN_RESET); // Pull the TRIG pin Low
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_UART_Receive_IT(&huart2, (uint8_t*)Rx_data,1);
	GroupInfoDisplay();
	SmileFace();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		DieuKhienXe();
		SmileFace();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(state == 2)
		{
			for(int a=250;a<551;a=a+100)
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, a);
				HAL_Delay(200);
				HCSR04();
				if(Distance < DistLimit)
				{
					DungXe();
					CautionDisplay();
					if(state==2)state=0;
				}
				DieuKhienXe();
				if(state==0)break;
			}
//----------------------------------------------------------------
			for(int a=550;a>249;a=a-100)
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, a);
				HAL_Delay(200);
				HCSR04();
				if(Distance < DistLimit)
				{
					DungXe();
					CautionDisplay();
					if(state==2)state=0;
				}
				DieuKhienXe();
				if(state==0)break;
			}
		}
			
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HCSR04()
{
		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
    // wait for the echo pin to go high
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
    Value_1 = __HAL_TIM_GET_COUNTER (&htim1);

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    // wait for the echo pin to go low
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    Value_2 = __HAL_TIM_GET_COUNTER (&htim1);

    Distance = (Value_2-Value_1)* 0.034/2; // Vas = 340m/s = 0.034cm/us
    HAL_Delay(50);
}
// Hien thi thong tin nhom
void GroupInfoDisplay()
{
	// Name Topic
	Lcd_gotoxy(0,0);
	Lcd_write_string("CANH BAO LUI XE");
	// Name Group
	Lcd_gotoxy(0,1);
	Lcd_write_string("NHOM 15");
	HAL_Delay(3000);
	Lcd_clear_display();
	// Name Member
	Lcd_gotoxy(0,0);
	Lcd_write_string("   Trang-Hang  ");
	Lcd_gotoxy(0,1);
	Lcd_write_string("Thai-Quang-Tung");
	HAL_Delay(3000);
	Lcd_clear_display();
}

// Hien thi mat cuoi

void SmileFace()
{
	Lcd_write_custom_char(3,0,0);
	Lcd_write_custom_char(4,0,1);
	Lcd_write_custom_char(3,1,2);
	Lcd_write_custom_char(4,1,3);
	Lcd_write_custom_char(6,1,4);
	Lcd_write_custom_char(7,1,5);
	Lcd_write_custom_char(8,1,6);
	Lcd_write_custom_char(9,1,7);
	Lcd_write_custom_char(11,0,0);
	Lcd_write_custom_char(12,0,1);
	Lcd_write_custom_char(11,1,2);
	Lcd_write_custom_char(12,1,3);
}
// Hien thi khoang cach voi vat can

void CautionDisplay()
{
	Lcd_clear_display();
	Lcd_gotoxy(0,0);
	Lcd_write_string("Dist: ");
	Lcd_write_int(Distance);//Value_1
	Lcd_write_string(" cm");
	Lcd_gotoxy(0,1);
	Lcd_gotoxy(0,1);
	Lcd_write_string("Caution !");
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10, 1);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10, 0);
	Lcd_clear_display();
}

void DungXe()
{
	HAL_GPIO_WritePin(IN1_PORT,IN1_PIN,1);
	HAL_GPIO_WritePin(IN2_PORT,IN2_PIN,1);
	HAL_GPIO_WritePin(IN3_PORT,IN3_PIN,1);
	HAL_GPIO_WritePin(IN4_PORT,IN4_PIN,1);
}

void DiTien()
{
	HAL_GPIO_WritePin(IN1_PORT,IN1_PIN,0);
	HAL_GPIO_WritePin(IN2_PORT,IN2_PIN,1);
	HAL_GPIO_WritePin(IN3_PORT,IN3_PIN,1);
	HAL_GPIO_WritePin(IN4_PORT,IN4_PIN,0);
}

void DiLui()
{
	HAL_GPIO_WritePin(IN1_PORT,IN1_PIN,1);
	HAL_GPIO_WritePin(IN2_PORT,IN2_PIN,0);
	HAL_GPIO_WritePin(IN3_PORT,IN3_PIN,0);
	HAL_GPIO_WritePin(IN4_PORT,IN4_PIN,1);
}

void QuayTrai()
{
	HAL_GPIO_WritePin(IN1_PORT,IN1_PIN,1);
	HAL_GPIO_WritePin(IN2_PORT,IN2_PIN,0);
	HAL_GPIO_WritePin(IN3_PORT,IN3_PIN,1);
	HAL_GPIO_WritePin(IN4_PORT,IN4_PIN,0);
}

void QuayPhai()
{
	HAL_GPIO_WritePin(IN1_PORT,IN1_PIN,0);
	HAL_GPIO_WritePin(IN2_PORT,IN2_PIN,1);
	HAL_GPIO_WritePin(IN3_PORT,IN3_PIN,0);
	HAL_GPIO_WritePin(IN4_PORT,IN4_PIN,1);
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
