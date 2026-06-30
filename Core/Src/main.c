/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdmmc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "libs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern const uint8_t bmi270_config_file[];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile bool fifo_wm_flag = false;
DMA_HandleTypeDef hdma_sdmmc1;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    printf("UART Error: %lu\r\n", huart->ErrorCode);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_sec = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ACC2_INT_Pin)
    {
        //printf("Interrupt!\n");
        fifo_wm_flag = true;
    }
}

void delay_us(uint32_t us, void *intf_ptr)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}

void update_LED(uint32_t elapsed, uint32_t interval) {
  if ((elapsed / interval) % 2 == 0) {
    //HAL_GPIO_WritePin(Error_LED_GPIO_Port, Error_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Battery_LED_GPIO_Port, Battery_LED_Pin, GPIO_PIN_SET);
  } else {
    //HAL_GPIO_WritePin(Error_LED_GPIO_Port, Error_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Battery_LED_GPIO_Port, Battery_LED_Pin, GPIO_PIN_RESET);
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd);
extern HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd);

uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
	uint8_t sd_state = MSD_OK;
  /* Invalidate the dma tx handle*/
  hsd1.hdmatx = NULL;

  /* Prepare the dma channel for a read operation */
  sd_state = SD_DMAConfigRx(&hsd1);

  if(sd_state == HAL_OK)
  {
	   /* Read block(s) in DMA transfer mode */
		sd_state = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  }

  if( sd_state == HAL_OK)
  {
	return MSD_OK;
  }
  else
  {
	return MSD_ERROR;
  }
}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
	uint8_t sd_state = MSD_OK;

	  // Invalidate the dma rx handle
	  hsd1.hdmarx = NULL;

	  // Prepare the dma channel for a read operation
	  sd_state = SD_DMAConfigTx(&hsd1);

	  if(sd_state == HAL_OK)
	  {
		/* Write block(s) in DMA transfer mode */
		sd_state = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks);
	  }

	  if( sd_state == HAL_OK)
	  {
		return MSD_OK;
	  }
	  else
	  {
		return MSD_ERROR;
	  }

	  return sd_state;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  //enable cycle counting for precise us sleep
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  fifo.data = fifo_buffer;
  fifo.length = sizeof(fifo_buffer);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("test\n");
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Deinitialize USART1
  HAL_UART_DeInit(&huart1);

  // Optional: Disable USART1 peripheral clock if no longer needed
  __HAL_RCC_USART1_CLK_DISABLE();

  // Make sure GPIOA clock is enabled
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure PA9 as GPIO input
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;    // Or GPIO_PULLUP / GPIO_PULLDOWN as needed

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(Power_Disable_GPIO_Port,Power_Disable_Pin,GPIO_PIN_SET);
  
  int res = initBMI270();

  while (res == -2) { // comm error, likely due to resetting
    HAL_Delay(2000);
    res = initBMI270();
  }

  if (res != 0) {
    Error_Handler();
  }

  initSD();
  initSDFiles();
  printf("SD Initialization successful!\n");

  initTMP();
  printf("TMP/PRS initialization successful!\n");

  initBMIC();

  GPS_Start();
  setupAccSleep();
  //set start time to post-initialization start
  startTime = HAL_GetTick();
  //HAL_GPIO_WritePin(Power_Disable_GPIO_Port,Power_Disable_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (fifo_wm_flag) {

      readBMI270();
      uint16_t fifo_len;
      bmi2_get_fifo_length(&fifo_len, &bmi270);
      if (fifo_len < FIFO_WM_THRESH) {
        fifo_wm_flag = false;
      }
    }
    if ((HAL_GetTick() - startTime)/1000 > last_sec) {
      last_sec = (HAL_GetTick() - startTime)/1000;
      readTMP();
      readGPS();
      readBMIC();
    }
    //printf("successful tmp sensor read!\n");
    update_LED(HAL_GetTick() - startTime, 1000);
    //HAL_Delay(1000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
}

/* USER CODE BEGIN 4 */

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
  HAL_GPIO_WritePin(Error_LED_GPIO_Port, Error_LED_Pin, GPIO_PIN_SET);
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
