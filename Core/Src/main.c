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
#include "task.h"  /* taskENTER_CRITICAL */
#include "sht31.h"
#include "ssd1306.h"
#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#define ALERT_OFF INT32_MAX  /* threshold value meaning "no alert set" */
#define CMD_LINE_MAX 32U     /* longest command line, including the '\0' */
#define RX_QUEUE_LENGTH 64U  /* typed bytes buffered between ISR and task */

/* Watchdog: each task sets its bit at least this often... */
#define WDG_SENSOR_ALIVE (1U << 0)
#define WDG_OUTPUT_ALIVE (1U << 1)
#define WDG_COMMAND_ALIVE (1U << 2)
#define WDG_ALL_TASKS (WDG_SENSOR_ALIVE | WDG_OUTPUT_ALIVE | WDG_COMMAND_ALIVE)
#define WDG_CHECKIN_MS 500U  /* how often idle tasks wake just to check in */
/* ...and the watchdog task allows this long for all bits to appear. Must stay
 * well under the hardware timeout (~5 s nominal, ~3.4 s worst case).
 */
#define WDG_WINDOW_MS 2000U
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

/* Command task: reads typed lines from UART and acts on them. It sleeps on
 * rxQueue, so it costs nothing until a key is pressed.
 */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
    .name = "commandTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

/* Bytes from the UART receive interrupt to the command task. The ISR must
 * not do real work, so it only drops each byte in here.
 */
osMessageQueueId_t rxQueue;
static uint8_t rx_byte; /* HAL writes each received byte here */

/* Output and command tasks both print, so UART now needs a lock too. */
osMutexId_t uartMutex;
const osMutexAttr_t uartMutex_attributes = {
    .name = "uartMutex",
    .attr_bits = osMutexPrioInherit,
};

/* Set by the command task, read by the output task. A single aligned 32-bit
 * read or write is atomic on Cortex-M4, so no lock is needed.
 */
static volatile int32_t alert_threshold_f100 = ALERT_OFF;

/* Set by the command task while a command is half-typed; the output task
 * skips its UART sample line meanwhile so it doesn't land in the middle of
 * your typing. Measurement and the OLED are unaffected.
 */
static volatile int user_typing = 0;

/* Watchdog task: feeds the hardware watchdog only while every other task
 * keeps checking in. Highest priority, so it is never starved by them.
 */
osThreadId_t watchdogTaskHandle;
const osThreadAttr_t watchdogTask_attributes = {
    .name = "watchdogTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

/* One bit per task (WDG_*_ALIVE). Tasks set their bit; the watchdog task
 * waits for all of them and clears them each round.
 */
osEventFlagsId_t wdgFlags;

/* Copy of the newest sample for the "status" command. The mailbox can't be
 * used for this: reading it would take the sample away from the output task.
 * Guarded by a critical section because it is larger than one word.
 */
static SensorSample latest_sample;
static int latest_valid = 0;

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
/* Before the scheduler starts there is only one thread of execution, so the
 * lock is skipped (and the mutex may not exist yet).
 */
static void uart_send(const char *message, size_t length) {
  int lock = (osKernelGetState() == osKernelRunning);
  if (lock) {
    osMutexAcquire(uartMutex, osWaitForever);
  }
  HAL_StatusTypeDef status = HAL_UART_Transmit(
      &huart2, (const uint8_t *)message, (uint16_t)length, UART_TIMEOUT_MS);
  if (lock) {
    osMutexRelease(uartMutex);
  }
  if (status != HAL_OK) {
    Error_Handler();
  }
}

static void uart_send_text(const char *text) { uart_send(text, strlen(text)); }

/* Preserve the peripheral-reset recovery used during bring-up.
 * This cannot fix missing power, loose wires or a device holding SDA low.
 * Returns the result; the caller decides what to report.
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

static void wdg_checkin(uint32_t task_bit) { osEventFlagsSet(wdgFlags, task_bit); }

static void report_sample(uint32_t counter, SHT31_Status status,
                          const SHT31_Reading *reading, int alert) {
  char message[96];
  int length;

  if (status == SHT31_OK) {
    /* The sign is separate so -50 hundredths prints as -0.50, not 0.50. */
    long temperature = (long)reading->temperature_f100;
    long magnitude = labs(temperature);
    long humidity = (long)reading->humidity_rh100;

    length = snprintf(message, sizeof(message),
                      "Sample %lu: T=%s%ld.%02ld F RH=%ld.%02ld %% CRC=OK%s\r\n",
                      (unsigned long)counter, temperature < 0 ? "-" : "",
                      magnitude / 100L, magnitude % 100L, humidity / 100L,
                      humidity % 100L, alert ? " ALERT" : "");
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
                             const SHT31_Reading *reading, int32_t threshold,
                             int alert) {
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

  if (threshold != ALERT_OFF) {
    snprintf(line, sizeof(line), alert ? "ALERT! > %ld F" : "LIMIT: %ld F",
             (long)(threshold / 100));
    SSD1306_DrawText(0, 7, line);
  }

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
 * It is the only task that uses the OLED framebuffer, so that needs no lock.
 * I2C and UART are shared, so those go through their mutexes.
 */
static void StartOutputTask(void *argument) {
  (void)argument;
  SensorSample sample;

  for (;;) {
    /* Checking in here, with a timeout on the wait, means a stuck sensor
     * task shows up as "sensor" in the watchdog report, not as both tasks.
     */
    wdg_checkin(WDG_OUTPUT_ALIVE);
    if (osMessageQueueGet(sampleQueue, &sample, NULL, WDG_WINDOW_MS / 2U) !=
        osOK) {
      continue;
    }

    /* Read the threshold once so the UART line and the OLED agree. */
    int32_t threshold = alert_threshold_f100;
    int alert = (sample.status == SHT31_OK && threshold != ALERT_OFF &&
                 sample.reading.temperature_f100 > threshold);

    if (!user_typing) {
      report_sample(sample.counter, sample.status, &sample.reading, alert);
    }
    if (sample.i2c_recovery_failed) {
      static const char message[] = "I2C1 recovery failed; retrying\r\n";
      uart_send(message, sizeof(message) - 1U);
    }

    if (oled_ready) {
      oled_show_sample(sample.counter, sample.status, &sample.reading,
                       threshold, alert);
    } else if (sample.counter % OLED_RETRY_SAMPLES == 0U) {
      oled_try_init();
    }
  }
}

/* ---- Command task --------------------------------------------------------*/

/* Runs in interrupt context, once per received byte. Keep it tiny: hand the
 * byte to the command task and re-arm reception for the next one.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    osMessageQueuePut(rxQueue, &rx_byte, 0U, 0U); /* drops the byte if full */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
  }
}

/* A receive error (e.g. overrun) stops HAL's reception; restart it. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
  }
}

static void print_status(void) {
  SensorSample sample;
  int valid;

  /* Copy under a critical section so the sensor task can't update the
   * struct halfway through our read.
   */
  taskENTER_CRITICAL();
  sample = latest_sample;
  valid = latest_valid;
  taskEXIT_CRITICAL();

  char message[96];
  if (!valid) {
    uart_send_text("No sample yet\r\n");
  } else if (sample.status != SHT31_OK) {
    snprintf(message, sizeof(message), "Last sample %lu: SHT31 %s\r\n",
             (unsigned long)sample.counter, SHT31_StatusText(sample.status));
    uart_send_text(message);
  } else {
    long temperature = (long)sample.reading.temperature_f100;
    long magnitude = labs(temperature);
    long humidity = (long)sample.reading.humidity_rh100;
    snprintf(message, sizeof(message),
             "Last sample %lu: T=%s%ld.%02ld F RH=%ld.%02ld %%\r\n",
             (unsigned long)sample.counter, temperature < 0 ? "-" : "",
             magnitude / 100L, magnitude % 100L, humidity / 100L,
             humidity % 100L);
    uart_send_text(message);
  }

  int32_t threshold = alert_threshold_f100;
  if (threshold == ALERT_OFF) {
    snprintf(message, sizeof(message), "Alert: off   OLED: %s\r\n",
             oled_ready ? "ok" : "not responding");
  } else {
    snprintf(message, sizeof(message), "Alert: above %ld F   OLED: %s\r\n",
             (long)(threshold / 100), oled_ready ? "ok" : "not responding");
  }
  uart_send_text(message);
}

static void handle_command(char *line) {
  for (char *p = line; *p != '\0'; p++) {
    *p = (char)tolower((unsigned char)*p);
  }

  if (strcmp(line, "help") == 0) {
    uart_send_text("Commands:\r\n"
                   "  status       latest reading and settings\r\n"
                   "  alert <F>    warn when temperature goes above <F>\r\n"
                   "  alert off    disable the warning\r\n"
                   "  hang         test: freeze this task; watchdog resets\r\n");
  } else if (strcmp(line, "status") == 0) {
    print_status();
  } else if (strcmp(line, "hang") == 0) {
    /* Watchdog test: this task stops checking in, as if it were deadlocked. */
    uart_send_text("Simulating a stuck command task...\r\n");
    user_typing = 0; /* let sample lines keep printing meanwhile */
    for (;;) {
      osDelay(1000U);
    }
  } else if (strcmp(line, "alert off") == 0) {
    alert_threshold_f100 = ALERT_OFF;
    uart_send_text("Alert off\r\n");
  } else if (strncmp(line, "alert ", 6) == 0) {
    char *end;
    long degrees = strtol(line + 6, &end, 10);
    if (end == line + 6 || *end != '\0' || degrees < -40 || degrees > 200) {
      uart_send_text("Usage: alert <whole degrees F, -40..200> or alert off\r\n");
    } else {
      alert_threshold_f100 = (int32_t)(degrees * 100);
      char message[48];
      snprintf(message, sizeof(message), "Alert set: above %ld F\r\n", degrees);
      uart_send_text(message);
    }
  } else {
    uart_send_text("Unknown command; type 'help'\r\n");
  }
}

/* Builds a line from typed bytes, echoing them so you can see what you type
 * (the serial terminal does not echo locally). Enter runs the command.
 */
static void StartCommandTask(void *argument) {
  (void)argument;
  char line[CMD_LINE_MAX];
  size_t length = 0;
  uint8_t ch;

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1U); /* start listening */
  uart_send_text("Type 'help' and press Enter for commands\r\n");

  for (;;) {
    /* Wake at least every WDG_CHECKIN_MS even with no typing, to check in. */
    wdg_checkin(WDG_COMMAND_ALIVE);
    if (osMessageQueueGet(rxQueue, &ch, NULL, WDG_CHECKIN_MS) != osOK) {
      continue;
    }

    if (ch == '\r' || ch == '\n') {
      if (length > 0) {
        uart_send_text("\r\n");
        line[length] = '\0';
        handle_command(line);
        length = 0;
        user_typing = 0; /* command done: sample lines resume */
      }
    } else if (ch == 0x08 || ch == 0x7F) { /* backspace / delete */
      if (length > 0) {
        length--;
        uart_send_text("\b \b");
        if (length == 0) {
          user_typing = 0; /* line erased: nothing left to protect */
        }
      }
    } else if (ch >= 0x20 && ch <= 0x7E && length < CMD_LINE_MAX - 1U) {
      user_typing = 1; /* pause sample lines before echoing the first key */
      line[length++] = (char)ch;
      uart_send((const char *)&ch, 1U);
    }
  }
}

/* ---- Watchdog ------------------------------------------------------------*/

/* The independent watchdog (IWDG) is a hardware down-counter on its own
 * ~32 kHz clock (LSI). If it reaches zero, the chip resets. Writing 0xAAAA
 * to KR reloads it ("feeding"). Once started it cannot be stopped.
 * Driven directly through its registers, since the HAL IWDG module is not
 * enabled in this CubeMX project.
 */
static void iwdg_start(void) {
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP; /* pause it at breakpoints */
  IWDG->KR = 0xCCCCU; /* start the watchdog (also starts LSI) */
  IWDG->KR = 0x5555U; /* unlock PR and RLR for writing */
  IWDG->PR = 4U;      /* LSI / 64: one count every ~2 ms */
  IWDG->RLR = 2500U;  /* ~5 s at 32 kHz; LSI tolerance gives ~3.4-9.4 s */
  while (IWDG->SR != 0U) {
    /* wait until the new prescaler/reload values are taken up */
  }
  IWDG->KR = 0xAAAAU; /* load the counter with RLR */
}

static void iwdg_feed(void) { IWDG->KR = 0xAAAAU; }

static void StartWatchdogTask(void *argument) {
  (void)argument;
  iwdg_start();

  for (;;) {
    /* Wait for every task's bit. On success the bits are cleared, so each
     * round needs a fresh check-in from every task.
     */
    uint32_t result = osEventFlagsWait(wdgFlags, WDG_ALL_TASKS, osFlagsWaitAll,
                                       WDG_WINDOW_MS);
    if ((result & osFlagsError) == 0U) {
      iwdg_feed();
      continue;
    }

    /* Someone missed the window: say who, then stop feeding. The hardware
     * resets the board within a few seconds. (If the stuck task holds the
     * UART lock, the message may not appear, but the reset still happens.)
     */
    uint32_t seen = osEventFlagsGet(wdgFlags);
    char message[96] = "WATCHDOG: no check-in from";
    if ((seen & WDG_SENSOR_ALIVE) == 0U) strcat(message, " sensor");
    if ((seen & WDG_OUTPUT_ALIVE) == 0U) strcat(message, " output");
    if ((seen & WDG_COMMAND_ALIVE) == 0U) strcat(message, " command");
    strcat(message, "; resetting\r\n");
    uart_send_text(message);
    for (;;) {
      osDelay(1000U);
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

  /* Report why we booted, so a watchdog reset is visible. */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    uart_send_text("\r\n*** Reset by watchdog: a task stopped responding ***\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

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
  uartMutex = osMutexNew(&uartMutex_attributes);
  if (i2cMutex == NULL || uartMutex == NULL) {
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
  rxQueue = osMessageQueueNew(RX_QUEUE_LENGTH, sizeof(uint8_t), NULL);
  wdgFlags = osEventFlagsNew(NULL);
  if (sampleQueue == NULL || rxQueue == NULL || wdgFlags == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  outputTaskHandle = osThreadNew(StartOutputTask, NULL, &outputTask_attributes);
  commandTaskHandle =
      osThreadNew(StartCommandTask, NULL, &commandTask_attributes);
  watchdogTaskHandle =
      osThreadNew(StartWatchdogTask, NULL, &watchdogTask_attributes);
  if (outputTaskHandle == NULL || commandTaskHandle == NULL ||
      watchdogTaskHandle == NULL) {
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

    /* Keep a copy for the "status" command. */
    taskENTER_CRITICAL();
    latest_sample = sample;
    latest_valid = 1;
    taskEXIT_CRITICAL();

    counter++; /* Counts attempts, including failed measurements. */
    wdg_checkin(WDG_SENSOR_ALIVE);

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
