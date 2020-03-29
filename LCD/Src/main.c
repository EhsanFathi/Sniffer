/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD_Character.h"
#include "tm_stm32f4_hd44780.h"
#include "stdio.h"
#include "stdlib.h"
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

char		i,j;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void ShowNumberOnLCD1(char n);
void ShowNumberOnLCD2(char n);
char ReadButton(void);
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
  /* USER CODE BEGIN 2 */
	TM_HD44780_Init( 16 , 2 );
  //TM_HD44780_Puts( 0 , 0 , "1 2 3 4 5 6 7 8 9 10 11 12 13 14" );
	//HAL_Delay(1000);
	HAL_Delay(1000);
	TM_HD44780_Puts( 3 , 0 , "Starting..." );
	TM_HD44780_Puts( 4 , 1 , "ARC1400" );
	HAL_Delay(2000);
	TM_HD44780_DisplayOff();
	TM_HD44780_Clear();
	TM_HD44780_DisplayOn();
	
	TM_HD44780_Puts( 0 , 0 ,"FREQ : ");
	TM_HD44780_Puts( 0 , 1 , "Channel : " );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		/*********************Frequency***********************/
		
		
		if(ReadButton()==1)
		{
			while(ReadButton()==1);
			i++;
      if(i==100)	i=0;
      ShowNumberOnLCD1(i);
		}
		else if(ReadButton()==2)
    {
       while(ReadButton()==2);
       i--;
       if(i==255)	i=99;
       ShowNumberOnLCD1(i);     
     }   
		else if(ReadButton()==3)
		{
			while(ReadButton()==3);
			j++;
      if(j==10)	j=0;
      ShowNumberOnLCD2(j);
		}
		else if(ReadButton()==4)
    {
       while(ReadButton()==4);
       j--;
       if(j==255)	j=9;
       ShowNumberOnLCD2(j);     
    }   
		
		
		/*********************Channels************************/
		if((HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)) == 0)
		{
			TM_HD44780_Puts( 10 , 1 , "1" );
    }
		if((HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin)) == 0)
		{
			TM_HD44780_Puts( 10 , 1 , "2");
		}
		if((HAL_GPIO_ReadPin(key3_GPIO_Port,key3_Pin)) == 0)
		{
			TM_HD44780_Puts( 10 , 1 , "3");
		}
		if((HAL_GPIO_ReadPin(key4_GPIO_Port,key4_Pin)) == 0)
		{
			TM_HD44780_Puts( 10 , 1 , "4");
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin|LCD_D7_Pin 
                          |LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_E_Pin LCD_D7_Pin 
                           LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin|LCD_D7_Pin 
                          |LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : key1_Pin key2_Pin key3_Pin key4_Pin 
                           key5_Pin key6_Pin key7_Pin key8_Pin */
  GPIO_InitStruct.Pin = key1_Pin|key2_Pin|key3_Pin|key4_Pin 
                          |key5_Pin|key6_Pin|key7_Pin|key8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/***************************************************************/
char ReadButton(void)
{
    if((HAL_GPIO_ReadPin(key5_GPIO_Port,key5_Pin)) == 0)
        return 1;
    else if((HAL_GPIO_ReadPin(key6_GPIO_Port,key6_Pin)) == 0)
        return 2;
		else if((HAL_GPIO_ReadPin(key7_GPIO_Port,key7_Pin)) == 0)
        return 3;
    else if((HAL_GPIO_ReadPin(key8_GPIO_Port,key8_Pin)) == 0)
        return 4;
		else
        return 0;
}

void ShowNumberOnLCD1(char n)
{
    char Buffer1[3];

    sprintf(Buffer1,"%d ", n);
    TM_HD44780_Puts(8,0,Buffer1);
}

void ShowNumberOnLCD2(char n)
{
    char Buffer2[2];

    sprintf(Buffer2,"%d", n);
    TM_HD44780_Puts(10,0,Buffer2);
}
/*****************************************************************/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
