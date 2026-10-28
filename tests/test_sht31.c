#include "sht31.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static HAL_StatusTypeDef tx_result;
static HAL_StatusTypeDef rx_result;
static unsigned int tx_calls, rx_calls, delay_calls;
static uint8_t received[6] = {0xBE, 0xEF, 0x92, 0xBE, 0xEF, 0x92};

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *i2c,
    uint16_t address, uint8_t *data, uint16_t size, uint32_t timeout) {
  assert(i2c != NULL && address == (0x44U << 1));
  assert(size == 2U && data[0] == 0x24U && data[1] == 0x00U);
  assert(timeout == 100U);
  tx_calls++;
  return tx_result;
}
void HAL_Delay(uint32_t milliseconds) {
  assert(milliseconds == 20U && tx_calls == 1U && rx_calls == 0U);
  delay_calls++;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *i2c,
    uint16_t address, uint8_t *data, uint16_t size, uint32_t timeout) {
  assert(i2c != NULL && address == (0x44U << 1) && size == 6U);
  assert(delay_calls == 1U && timeout == 100U);
  rx_calls++;
  if (rx_result == HAL_OK) {
    memcpy(data, received, sizeof(received));
  }
  return rx_result;
}

static void reset_io(void) {
  tx_result = HAL_OK;
  rx_result = HAL_OK;
  tx_calls = rx_calls = delay_calls = 0U;
}

int main(void) {
  I2C_HandleTypeDef i2c = {0};
  SHT31_Reading value = {123, 456, 789};

  /* Sensirion's independent CRC reference: BEEF -> 92. */
  assert(SHT31_Decode(received, 6U, &value) == SHT31_OK);
  assert(value.temperature_c100 == 8552);
  assert(value.temperature_f100 == 18594);
  assert(value.humidity_rh100 == 7458);

  const uint8_t low[6] = {0, 0, 0x81, 0, 0, 0x81};
  const uint8_t high[6] = {0xFF, 0xFF, 0xAC, 0xFF, 0xFF, 0xAC};
  assert(SHT31_Decode(low, 6U, &value) == SHT31_OK);
  assert(value.temperature_c100 == -4500);
  assert(value.temperature_f100 == -4900 && value.humidity_rh100 == 0);
  assert(SHT31_Decode(high, 6U, &value) == SHT31_OK);
  assert(value.temperature_c100 == 13000);
  assert(value.temperature_f100 == 26600 && value.humidity_rh100 == 10000);

  for (size_t byte = 0; byte < 6U; byte++) {
    uint8_t bad[6];
    memcpy(bad, received, 6U);
    bad[byte] ^= 1U;
    value = (SHT31_Reading){123, 456, 789};
    assert(SHT31_Decode(bad, 6U, &value) == SHT31_CRC_ERROR);
    assert(value.temperature_c100 == 123 && value.temperature_f100 == 456);
    assert(value.humidity_rh100 == 789);
  }
  assert(SHT31_Decode(low, 5U, &value) == SHT31_INVALID_ARGUMENT);
  assert(SHT31_Decode(low, 7U, &value) == SHT31_INVALID_ARGUMENT);
  assert(SHT31_Decode(NULL, 6U, &value) == SHT31_INVALID_ARGUMENT);
  assert(SHT31_Decode(low, 6U, NULL) == SHT31_INVALID_ARGUMENT);

  reset_io();
  assert(SHT31_Read(&i2c, &value) == SHT31_OK);
  assert(tx_calls == 1U && delay_calls == 1U && rx_calls == 1U);

  for (unsigned int error = HAL_ERROR; error <= HAL_TIMEOUT; error++) {
    reset_io();
    tx_result = (HAL_StatusTypeDef)error;
    value = (SHT31_Reading){123, 456, 789};
    assert(SHT31_Read(&i2c, &value) == SHT31_TX_ERROR);
    assert(tx_calls == 1U && delay_calls == 0U && rx_calls == 0U);
    assert(value.temperature_c100 == 123);
    reset_io();
    rx_result = (HAL_StatusTypeDef)error;
    assert(SHT31_Read(&i2c, &value) == SHT31_RX_ERROR);
    assert(tx_calls == 1U && delay_calls == 1U && rx_calls == 1U);
    assert(value.temperature_f100 == 456);
  }
  reset_io();
  received[5] ^= 1U;
  assert(SHT31_Read(&i2c, &value) == SHT31_CRC_ERROR);
  assert(value.humidity_rh100 == 789);
  assert(SHT31_Read(NULL, &value) == SHT31_INVALID_ARGUMENT);
  assert(SHT31_Read(&i2c, NULL) == SHT31_INVALID_ARGUMENT);
  puts("PASS: CRC, byte order, Celsius/Fahrenheit scaling, invalid frames,");
  puts("      unchanged failure output, and mocked command/wait/read failures.");
  return 0;
}

