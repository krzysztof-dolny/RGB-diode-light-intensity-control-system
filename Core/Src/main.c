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
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bh1750.h"
#include "regulator.h"
#include "led.h"
#include<stdio.h>
#include<math.h>
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

BH1750_HandleTypeDef bh1750_1 = {
.I2C = &hi2c1, .Address_r = BH1750_GROUND_ADDR_READ, .Address_w = BH1750_GROUND_ADDR_WRITE,
.Timeout = 1000};

regulator_Handle_TypeDef reg_I = {
.Kp = 0.0f, .Ki = 0.035f, .Kd = 0.0f, .u_p = 0.0f, .u_i = 0.0f,
.u_d = 0.0f, .limitdown = 0.0f, .limitup = 100.0f};

LED_HandleTypeDef led_rgb = {
.R = 1.0f, .G = 0.0f, .B = 1.0f, .duty_R = 0.0f, .duty_G = 0.0f, .duty_B = 0.0f};

int akcja = 0;
float wartosc_zadana = 0.0f;
float natezenie_swiatla = -0.1;
char wiadomosc[23];
uint8_t komunikat1[] = "Yr: 00000 \r\n Y: 00000 \r\n RED: 00 %, Green: 00 %, Blue: 00 % \r\n";
uint16_t dl_kom;
float sygnal_sterujacy = 0.0f;
float led_R, led_G, led_B;
int pulseR = 0, pulseG = 0, pulseB = 0;
int red_percent = 0, green_percent = 0, blue_percent = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  uint8_t TrybPracy = BH1750_CONTINOUS_H_RES_MODE ;
  BH1750_Init(&bh1750_1, TrybPracy);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_UART_Receive_IT(&huart3, (uint8_t*)wiadomosc, 23);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart->Instance == USART3)
	{
		if(wiadomosc[0] == 'Y' && wiadomosc[1] == ':' && wiadomosc[5] == ',' &&
				wiadomosc[6] == 'R' && wiadomosc[7] == ':' && wiadomosc[11] == ',' &&
				wiadomosc[12] == 'G' && wiadomosc[13] == ':' && wiadomosc[17] == ',' &&
				wiadomosc[18] == 'B' && wiadomosc[19] == ':')
		{
			akcja = 1;
			sscanf (wiadomosc,"Y:%f,R:%d,G:%d,B:%d", &wartosc_zadana, &pulseR, &pulseG, &pulseB);

			if(pulseR >=0 && pulseR <= 100)
			{
				led_rgb.R = (float)(pulseR / 100.0f);
			}
			if(pulseG >=0 && pulseG <= 100)
			{
				led_rgb.G = (float)(pulseG / 100.0f);
			}
			if(pulseB >=0 && pulseB <= 100)
			{
				led_rgb.B = (float)(pulseB / 100.0f);
			}
			float sum = (float)pulseR + (float)pulseG + (float)pulseB;
			red_percent = round(100 * (float)pulseR / sum);
			green_percent = round(100 * (float)pulseG / sum);
			blue_percent = round(100 * (float)pulseB / sum);
		}
		HAL_UART_Receive_IT(&huart3, (uint8_t*)wiadomosc, 23);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM2)
	{
		natezenie_swiatla = BH1750_ReadLux(&bh1750_1);
		if(akcja == 1)
		{
			sygnal_sterujacy = regulator_signal(&reg_I, wartosc_zadana, natezenie_swiatla);
			ColorsGenerator(&led_rgb, sygnal_sterujacy);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)((led_rgb.duty_R) * 10));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)((led_rgb.duty_G) * 10));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)((led_rgb.duty_B) * 10));
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == P1_Pin)
	{
		dl_kom = sprintf((char *)komunikat1, "Yr: %f lx \r\n Y: %f lx \r\n RED: %d , Green: %d , Blue: %d \r\n Sygnal sterujacy: %f \r\n \r\n",
				wartosc_zadana, natezenie_swiatla, red_percent, green_percent, blue_percent, sygnal_sterujacy);
	    HAL_UART_Transmit(&huart3, komunikat1, dl_kom, 100);
	}
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
