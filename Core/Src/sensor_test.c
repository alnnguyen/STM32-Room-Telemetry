/*
 * Standalone SHT31 bring-up test. Only runs when built with -DSENSOR_TEST
 * (see main.c, USER CODE BEGIN 2). Never returns.
 *
 * Every second it prints:
 *   1. every I2C address that ACKs (bus scan)
 *   2. a full SHT31 reading with CRC check, or exactly which step failed
 */
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

#define SHT31_ADDR (0x44U << 1)

static void out(const char *fmt, ...) {
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) {
    if ((size_t)n >= sizeof(buf)) {
      n = sizeof(buf) - 1;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100U);
  }
}

// Sensirion CRC-8: polynomial 0x31, init 0xFF
static uint8_t crc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// A stuck bus (error 0x20 / 0x200) needs the peripheral reset to recover
// (STM32F4 quirk: the BUSY flag can latch after a glitch and only a peripheral
// reset clears it, so pulse the RCC reset line, not just DeInit/Init.)
static void i2c_recover(void) {
  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_I2C_Init(&hi2c1);
}

static void i2c_recover(void);

// Connectivity check: put a weak pull-down on each line and read it.
// 1 = a stronger external pull-up wins (sensor board connected AND powered).
// 0 = nothing is pulling the line up (wire off, or sensor board unpowered).
static void print_lines(void) {
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_Delay(2);
  uint32_t idr = GPIOB->IDR;
  out("pulldown test: SCL(PB8)=%lu SDA(PB9)=%lu  (1 = sensor pull-up seen)\r\n",
      (unsigned long)((idr >> 8) & 1U), (unsigned long)((idr >> 9) & 1U));
  i2c_recover(); // hand the pins back to the I2C peripheral
}

static void scan_bus(void) {
  char line[96];
  int pos = snprintf(line, sizeof(line), "scan:");
  int found = 0;
  for (uint8_t a = 1; a < 128 && pos < (int)sizeof(line) - 8; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1U, 5U) == HAL_OK) {
      pos += snprintf(line + pos, sizeof(line) - pos, " 0x%02X", a);
      found++;
    }
  }
  if (!found) {
    pos += snprintf(line + pos, sizeof(line) - pos, " none (err 0x%02lX)",
                    (unsigned long)HAL_I2C_GetError(&hi2c1));
    i2c_recover();
  }
  out("%s\r\n", line);
}

static void read_sht31(void) {
  uint8_t cmd[2] = {0x24, 0x00}; // single shot, high repeatability
  uint8_t rx[6] = {0};

  if (HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, cmd, sizeof(cmd), 100U) !=
      HAL_OK) {
    out("read: TX failed (err 0x%02lX)\r\n",
        (unsigned long)HAL_I2C_GetError(&hi2c1));
    i2c_recover();
    return;
  }

  HAL_Delay(20);

  if (HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, rx, sizeof(rx), 100U) !=
      HAL_OK) {
    out("read: RX failed (err 0x%02lX)\r\n",
        (unsigned long)HAL_I2C_GetError(&hi2c1));
    i2c_recover();
    return;
  }

  if (crc8(&rx[0], 2) != rx[2] || crc8(&rx[3], 2) != rx[5]) {
    out("read: CRC mismatch (raw %02X %02X %02X %02X %02X %02X)\r\n", rx[0],
        rx[1], rx[2], rx[3], rx[4], rx[5]);
    return;
  }

  uint16_t t_raw = ((uint16_t)rx[0] << 8) | rx[1];
  uint16_t h_raw = ((uint16_t)rx[3] << 8) | rx[4];
  int32_t t100 = -4500 + (int32_t)(17500UL * t_raw / 65535UL);
  int32_t h100 = (int32_t)(10000UL * h_raw / 65535UL);

  out("read: OK  T=%s%ld.%02ld C  RH=%ld.%02ld %%\r\n", t100 < 0 ? "-" : "",
      labs(t100) / 100, labs(t100) % 100, (long)(h100 / 100),
      (long)(h100 % 100));
}

void sensor_test_run(void) {
  out("\r\n--- SHT31 sensor test ---\r\n");
  i2c_recover(); // start from a clean peripheral
  for (;;) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    print_lines();
    scan_bus();
    read_sht31();
    HAL_Delay(1000U);
  }
}
