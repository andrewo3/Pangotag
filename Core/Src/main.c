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
#include "fatfs.h"
#include "i2c.h"
#include "sdmmc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bmi270/bmi270.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern const uint8_t bmi270_config_file[];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile bool fifo_wm_flag = false;
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
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

int8_t bmi270_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    int res = HAL_I2C_Mem_Write(&hi2c2,
                              BMI2_I2C_PRIM_ADDR << 1,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t*)data,
                              len,
                              HAL_MAX_DELAY);
    if (res != HAL_OK) {
      return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

int8_t bmi270_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
    int res = HAL_I2C_Mem_Read(&hi2c2,
                             BMI2_I2C_PRIM_ADDR << 1,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             len,
                             HAL_MAX_DELAY);
    if (res != HAL_OK) {
      return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ACC2_INT_Pin)
    {
        printf("Interrupt!\n");
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
    HAL_GPIO_WritePin(Error_LED_GPIO_Port, Error_LED_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(Error_LED_GPIO_Port, Error_LED_Pin, GPIO_PIN_RESET);
  }
}
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
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint32_t start = HAL_GetTick();

  uint8_t fifo_buffer[1024];
  struct bmi2_fifo_frame fifo = {0};

  fifo.data = fifo_buffer;
  fifo.length = sizeof(fifo_buffer);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_DeInit(&huart1);
  //MX_USART1_UART_Init();
  printf("test\n");
  HAL_GPIO_WritePin(Power_Disable_GPIO_Port,Power_Disable_Pin,GPIO_PIN_RESET);
  
  struct bmi2_dev dev = {0};
  dev.read = bmi270_read;
  dev.write = bmi270_write;
  dev.delay_us = delay_us;
  dev.intf = BMI2_I2C_INTF;
  dev.config_file_ptr = nullptr;
  printf("bmi270 cfg @ %p: 0x%02x 0x%02x\n",bmi270_config_file, bmi270_config_file[0], bmi270_config_file[1]);
  uint8_t chip_id = 0x00;
  HAL_I2C_Mem_Read(&hi2c2,
                 BMI2_I2C_PRIM_ADDR << 1,
                 BMI2_CHIP_ID_ADDR,
                 I2C_MEMADD_SIZE_8BIT,
                 &chip_id,
                 1,
                 HAL_MAX_DELAY);
  printf("Chip ID: 0x%02x\n",chip_id);

  int8_t rslt = bmi270_init(&dev);
  if (rslt != BMI2_OK) {
      // communication failed
      printf("Failed to initialize BMI270: err %i\n",rslt);
      Error_Handler();
  }

  rslt = bmi2_soft_reset(&dev);
  dev.delay_us(2000,&dev);
  if (rslt != BMI2_OK) {
      // communication failed
      printf("Failed to soft reset BMI270: err %i\n",rslt);
      Error_Handler();
  } 

  struct bmi2_sens_config config;
  config.type = BMI2_ACCEL;
  bmi2_get_sensor_config(&config,1,&dev);
  config.cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  config.cfg.acc.range = BMI2_ACC_RANGE_4G;
  config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

  bmi2_set_sensor_config(&config, 1, &dev);

  config.type = BMI2_GYRO;
  bmi2_get_sensor_config(&config, 1, &dev);

  config.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  config.cfg.gyr.range = BMI2_GYR_RANGE_250;
  config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

  bmi2_set_sensor_config(&config, 1, &dev);

  uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  rslt = bmi2_sensor_enable(sensor_list, 2, &dev);
  if (rslt != BMI2_OK) {
      // communication failed
      printf("Failed to enable BMI270: err %i\n",rslt);
      Error_Handler();
  } 

  uint16_t fifo_config =
    BMI2_FIFO_ACC_EN |
    BMI2_FIFO_GYR_EN |
    BMI2_FIFO_HEADER_EN |
    BMI2_FIFO_TIME_EN;

  rslt = bmi2_set_fifo_config(fifo_config, BMI2_ENABLE, &dev);
  if (rslt != BMI2_OK) {
      // communication failed
      printf("BMI270: Failed to set FIFO config: err %i\n",rslt);
      Error_Handler();
  } 

  rslt = bmi2_set_fifo_wm(256, &dev);
  if (rslt != BMI2_OK) {
      // communication failed
      printf("BMI270: Failed to enable FIFO watermark: err %i\n",rslt);
      Error_Handler();
  }

  struct bmi2_int_pin_config int_cfg;
  struct bmi2_int_pin_cfg pin_cfg;

  int_cfg.pin_type = BMI2_INT1;

  pin_cfg.output_en = BMI2_ENABLE;
  pin_cfg.od        = BMI2_INT_PUSH_PULL;
  pin_cfg.lvl       = BMI2_INT_ACTIVE_HIGH;
  pin_cfg.input_en  = BMI2_DISABLE;
  int_cfg.pin_cfg[0] = pin_cfg;

  rslt = bmi2_set_int_pin_config(&int_cfg, &dev);
  if (rslt != BMI2_OK) {
      // communication failed
      printf("BMI270: Failed to enable INT1: err %i\n",rslt);
      Error_Handler();
  }

  rslt = bmi2_map_data_int(BMI2_FWM_INT, BMI2_INT1, &dev);
  if (rslt != BMI2_OK) {
      // communication failed
      printf("BMI270: Failed to enable WM interrupts: err %i\n",rslt);
      Error_Handler();
  } 

  printf("BMI270 Initialization successful!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (fifo_wm_flag) {
      fifo_wm_flag = false;

      uint16_t int_status;
      bmi2_get_int_status(&int_status, &dev);
      printf("FIFO Status: %i\n",int_status);

      uint16_t fifo_len;
      bmi2_get_fifo_length(&fifo_len, &dev);
      printf("fifo length before = %u\n", fifo_len);

      rslt = bmi2_read_fifo_data(&fifo, &dev);
      if (rslt != BMI2_OK) {
        printf("Failed to read FIFO Frame: err %i\n",rslt);
      } else {
        printf("Successful fifo frame read: %i frames\n",fifo.length);
        bmi2_get_fifo_length(&fifo_len, &dev);
        printf("fifo length after = %u\n", fifo_len);

        struct bmi2_sens_axes_data accel_data[128];
        struct bmi2_sens_axes_data gyro_data[128];

        uint16_t acc_frames = 128;
        uint16_t gyr_frames = 128;

        rslt = bmi2_extract_accel(accel_data, &acc_frames, &fifo, &dev);
        if (rslt < BMI2_OK) {
          printf("Failed to extract acceleration frame: err %i\n",rslt);
        }
        rslt = bmi2_extract_gyro(gyro_data, &gyr_frames, &fifo, &dev);
        if (rslt < BMI2_OK) {
          printf("Failed to extract gyroscope frame: err %i\n",rslt);
        }
        printf("Frame 1: (%i,%i,%i)\n",accel_data[0].x,accel_data[0].y,accel_data[0].z);

        printf("Decoded %i acc frames and %i gyro frames.\n",acc_frames, gyr_frames);

        bmi2_get_int_status(&int_status, &dev);
        printf("FIFO Status: %i\n",int_status);
      }
    }
    update_LED(HAL_GetTick() - start, 1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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
