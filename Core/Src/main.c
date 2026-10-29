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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h" /* xQueueOverwrite (no CMSIS-RTOS2 equivalent) */
#include "sht31.h"
#include "ssd1306.h"
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* One measurement, passed by copy from the sensor task to the output task. */
typedef struct {
  uint32_t counter; /* attempt number, including failures */
  SHT31_Status status;
  SHT31_Reading reading;   /* valid only when status == SHT31_OK */
  int i2c_recovery_failed; /* sensor task tried an I2C reset and it failed */
} SensorSample;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_PERIOD_MS 1000U
#define UART_TIMEOUT_MS 100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* defaultTask (above, CubeMX-generated) is the sensor task. */

/* Output task: prints and displays samples. Lower priority than the sensor,
 * so a slow OLED or UART can never delay a measurement from starting.
 */
osThreadId_t outputTaskHandle;
const osThreadAttr_t outputTask_attributes = {
    .name = "outputTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

/* Mailbox: a queue of length 1 that the sensor overwrites. The output task
 * always gets the newest sample; if it falls behind, old samples are
 * replaced rather than piling up.
 */
osMessageQueueId_t sampleQueue;

/* The SHT31 and the OLED share I2C1. Whoever holds this mutex owns the bus.
 * Priority inheritance: if the low-priority output task holds it while the
 * sensor task waits, the holder is temporarily raised so it finishes fast.
 */
osMutexId_t i2cMutex;
const osMutexAttr_t i2cMutex_attributes = {
    .name = "i2cMutex",
    .attr_bits = osMutexPrioInherit,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
#ifdef SENSOR_TEST
void sensor_test_run(void); /* Optional bring-up diagnostics. */
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_send(const char *message, size_t length) {
  if (HAL_UART_Transmit(&huart2, (const uint8_t *)message, (uint16_t)length,
                        UART_TIMEOUT_MS) != HAL_OK) {
    Error_Handler();
  }
}

/* Preserve the peripheral-reset recovery used during bring-up.
 * This cannot fix missing power, loose wires or a device holding SDA low.
 * Returns the result instead of printing: only the output task uses UART.
 * Caller must hold i2cMutex once the scheduler is running.
 */
static HAL_StatusTypeDef recover_i2c1(void) {
  HAL_StatusTypeDef status = HAL_I2C_DeInit(&hi2c1);
  if (status == HAL_OK) {
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    status = HAL_I2C_Init(&hi2c1);
  }
  return status;
}

static void i2c_lock(void) { osMutexAcquire(i2cMutex, osWaitForever); }
static void i2c_unlock(void) { osMutexRelease(i2cMutex); }

static void report_sample(uint32_t counter, SHT31_Status status,
                          const SHT31_Reading *reading) {
  char message[96];
  int length;

  if (status == SHT31_OK) {
    /* The sign is separate so -50 hundredths prints as -0.50, not 0.50. */
    long temperature = (long)reading->temperature_f100;
    long magnitude = labs(temperature);
    long humidity = (long)reading->humidity_rh100;

    length = snprintf(message, sizeof(message),
                      "Sample %lu: T=%s%ld.%02ld F RH=%ld.%02ld %% CRC=OK\r\n",
                      (unsigned long)counter, temperature < 0 ? "-" : "",
                      magnitude / 100L, magnitude % 100L, humidity / 100L,
                      humidity % 100L);
  } else {
    length = snprintf(message, sizeof(message), "Sample %lu: SHT31 %s\r\n",
                      (unsigned long)counter, SHT31_StatusText(status));
  }

  if (length > 0 && (size_t)length < sizeof(message)) {
    uart_send(message, (size_t)length);
  }
}

/* Set once the OLED has answered and initialised; cleared if an update fails,
 * so a missing display never slows down or stops the sensor loop.
 */
static int oled_ready = 0;

/* Draw the latest sample on the OLED. Drawing only fills a RAM buffer;
 * SSD1306_Update() is what sends it to the panel over I2C.
 */
static void oled_show_sample(uint32_t counter, SHT31_Status status,
                             const SHT31_Reading *reading) {
  char line[24];

  SSD1306_Clear();
  SSD1306_DrawText(0, 0, "SHT31 MONITOR");

  if (status == SHT31_OK) {
    long temperature = (long)reading->temperature_f100;
    long magnitude = labs(temperature);
    long humidity = (long)reading->humidity_rh100;

    snprintf(line, sizeof(line), "TEMP: %s%ld.%02ld F",
             temperature < 0 ? "-" : "", magnitude / 100L, magnitude % 100L);
    SSD1306_DrawText(0, 2, line);
    snprintf(line, sizeof(line), "HUMID: %ld.%02ld %%", humidity / 100L,
             humidity % 100L);
    SSD1306_DrawText(0, 3, line);
  } else {
    SSD1306_DrawText(0, 2, "SENSOR ERROR:");
    SSD1306_DrawText(0, 3, SHT31_StatusText(status));
  }

  snprintf(line, sizeof(line), "SAMPLE: %lu", (unsigned long)counter);
  SSD1306_DrawText(0, 5, line);

  /* Drawing above only touched RAM; this is the only part that uses I2C. */
  i2c_lock();
  HAL_StatusTypeDef update_status = SSD1306_Update(&hi2c1);
  i2c_unlock();

  if (update_status != HAL_OK) {
    oled_ready = 0;
    static const char message[] = "OLED update failed; will retry\r\n";
    uart_send(message, sizeof(message) - 1U);
  }
}

/* Called from the loop while the OLED is marked not ready: reset the I2C
 * peripheral and run the init sequence again. A display that is still
 * missing costs one short timeout per attempt, not a stalled loop.
 */
#define OLED_RETRY_SAMPLES 5U

static void oled_try_init(void) {
  i2c_lock();
  HAL_StatusTypeDef status = recover_i2c1();
  if (status == HAL_OK) {
    status = SSD1306_Init(&hi2c1);
  }
  i2c_unlock();

  if (status == HAL_OK) {
    oled_ready = 1;
    static const char message[] = "OLED re-initialised\r\n";
    uart_send(message, sizeof(message) - 1U);
  }
}

/* Output task: sleeps until a sample arrives, then prints and draws it.
 * It is the only task that uses the UART and the OLED framebuffer, so
 * neither needs a lock; only the shared I2C bus does.
 */
static void StartOutputTask(void *argument) {
  (void)argument;
  SensorSample sample;

  for (;;) {
    if (osMessageQueueGet(sampleQueue, &sample, NULL, osWaitForever) != osOK) {
      continue;
    }

    report_sample(sample.counter, sample.status, &sample.reading);
    if (sample.i2c_recovery_failed) {
      static const char message[] = "I2C1 recovery failed; retrying\r\n";
      uart_send(message, sizeof(message) - 1U);
    }

    if (oled_ready) {
      oled_show_sample(sample.counter, sample.status, &sample.reading);
    } else if (sample.counter % OLED_RETRY_SAMPLES == 0U) {
      oled_try_init();
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
#ifdef SENSOR_TEST
  sensor_test_run(); // never returns; see sensor_test.c
#endif

  HAL_Delay(2U); /* Allow sensor power-up before the first command. */
  if (recover_i2c1() != HAL_OK) {
    const char message[] = "I2C1 recovery failed at startup\r\n";
    uart_send(message, sizeof(message) - 1U);
  }

  // LCD INIT
  HAL_Delay(100U);

  HAL_StatusTypeDef oled_status =
      HAL_I2C_IsDeviceReady(&hi2c1, (0x3CU << 1), 2U, 100U);

  if (oled_status == HAL_OK) {
    const char message[] = "OLED ACK at 0x3C\r\n";
    uart_send(message, sizeof(message) - 1U);
  } else {
    const char message[] = "OLED no ACK at 0x3C\r\n";
    uart_send(message, sizeof(message) - 1U);
  }

  if (oled_status == HAL_OK && SSD1306_Init(&hi2c1) == HAL_OK) {
    oled_ready = 1;
    const char message[] = "OLED init OK\r\n";
    uart_send(message, sizeof(message) - 1U);
  } else {
    const char message[] = "OLED init failed\r\n";
    uart_send(message, sizeof(message) - 1U);
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  i2cMutex = osMutexNew(&i2cMutex_attributes);
  if (i2cMutex == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  sampleQueue = osMessageQueueNew(1U, sizeof(SensorSample), NULL);
  if (sampleQueue == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  outputTaskHandle = osThreadNew(StartOutputTask, NULL, &outputTask_attributes);
  if (outputTaskHandle == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Never reached: osKernelStart() hands the CPU to the scheduler.
     * The application loop lives in StartDefaultTask below.
     */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  /* Sensor task: measure on a fixed schedule and publish each result.
   * It never prints or draws; the output task does that from the queue.
   */
  (void)argument;
  uint32_t counter = 0;
  uint32_t next_wake = osKernelGetTickCount();

  for (;;) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    SensorSample sample = {0};
    sample.counter = counter;

    i2c_lock();
    sample.status = SHT31_Read(&hi2c1, &sample.reading);
    if (sample.status == SHT31_TX_ERROR || sample.status == SHT31_RX_ERROR) {
      sample.i2c_recovery_failed = (recover_i2c1() != HAL_OK);
    }
    i2c_unlock();

    /* Overwrite, never block: the sensor must not wait on a slow output. */
    xQueueOverwrite((QueueHandle_t)sampleQueue, &sample);

    counter++; /* Counts attempts, including failed measurements. */

    /* osDelayUntil wakes at fixed 1 s marks measured from the previous wake,
     * so time spent reading the sensor does not stretch the period.
     */
    next_wake += HEARTBEAT_PERIOD_MS;
    osDelayUntil(next_wake);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
