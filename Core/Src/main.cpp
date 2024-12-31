/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "gpio.h"
#include "utils.h"
#include <cstdio>
#include <string.h>
#include "lcd_display.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LCD_Display *display_pointer;


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
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // ユージング
  using namespace utils;

  // 基底
  const int control_ms = 2;
  const int debug_ms = 100;
  uint32_t time = 0;
  uint32_t time_debug = 0;

  uint16_t board_id = 0;
  int error = 0;

  // CAN通信
  CAN_FilterTypeDef filter;
  uint32_t fId1   =  0x000 << 5;
  uint32_t fMask1 = (0x7F0 << 5) | 0x8; // 0x000~0x00F
  uint32_t fId2   =  0x500 << 5;
  uint32_t fMask2 = (0x700 << 5) | 0x8; // 0x500~0x5FF
  filter.FilterIdHigh         = fId1;
  filter.FilterIdLow          = fId2;
  filter.FilterMaskIdHigh     = fMask1;
  filter.FilterMaskIdLow      = fMask2;
  filter.FilterScale          = CAN_FILTERSCALE_16BIT; // 16モード
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.SlaveStartFilterBank = 14;
  filter.FilterActivation     = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &filter);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  printf("CAN INIT\n");

  LCD_Display display(hi2c1);
  printf("LCD DISPLAY setup : %d\n", display.setup());

  display_pointer = &display;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(HAL_GetTick() - time > control_ms){
        time = HAL_GetTick();

        if(HAL_GetTick() - time_debug > debug_ms){
            time_debug = HAL_GetTick();
            HAL_GPIO_TogglePin(CYCLE_LED_GPIO_Port, CYCLE_LED_Pin);
            display.cycle();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
// printf関数
extern "C" {
int _write(int file, char *ptr, int len){
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++){
    ITM_SendChar(*ptr++);
  }
  return len;
}
}

// CANコールバック
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rxdata[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxdata) == HAL_OK){
        const uint32_t id = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;     // ID
        const uint32_t dlc = RxHeader.DLC;

        // 緊急停止情報
        if(id == 0x00F){
            if(rxdata[0]){
                HAL_GPIO_WritePin(RED_OUT_GPIO_Port, RED_OUT_Pin, GPIO_PIN_SET);
                display_pointer->write(0x000);
            }
            else{
                HAL_GPIO_WritePin(RED_OUT_GPIO_Port, RED_OUT_Pin, GPIO_PIN_RESET);
                display_pointer->write(0x001);
            }
        }
        else if(id == 0x000){
            display_pointer->write(0x002);
        }
        else if(id == 0x001){
            display_pointer->write(0x003);
        }

        // 基幹情報
        else if(id == 0x501){
            HAL_GPIO_WritePin(YELLOW_OUT_GPIO_Port, YELLOW_OUT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GREEN_OUT_GPIO_Port, GREEN_OUT_Pin, GPIO_PIN_RESET);
        }
        else if(id == 0x502){
            HAL_GPIO_WritePin(GREEN_OUT_GPIO_Port, GREEN_OUT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(YELLOW_OUT_GPIO_Port, YELLOW_OUT_Pin, GPIO_PIN_RESET);
        }

        // 表示機能のIDなら直接ディスプレイ表記する
        if(((id >> 8) & 0xF) == 0x5){
            display_pointer->write(id);
        }

        // printf("ID %03X\n", id);
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
