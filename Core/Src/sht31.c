#include "sht31.h"

#define SHT31_ADDRESS (0x44U << 1)
#define SHT31_TIMEOUT_MS 100U
#define SHT31_CONVERSION_MS 20U

/* Moved from the working sensor test: polynomial 0x31, initial value 0xFF. */
static uint8_t crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0xFFU;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (unsigned int bit = 0; bit < 8U; bit++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U)
                         : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

SHT31_Status SHT31_Decode(const uint8_t *frame, size_t length,
                         SHT31_Reading *reading) {
  if (frame == NULL || reading == NULL || length != 6U) {
    return SHT31_INVALID_ARGUMENT;
  }
  if (crc8(&frame[0], 2U) != frame[2] ||
      crc8(&frame[3], 2U) != frame[5]) {
    return SHT31_CRC_ERROR;
  }

  uint16_t t_raw = (uint16_t)(((uint16_t)frame[0] << 8) | frame[1]);
  uint16_t h_raw = (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);

  /* Integer scaling avoids requiring printf floating-point support. */
  SHT31_Reading result;
  result.temperature_c100 = -4500 + (int32_t)(17500UL * t_raw / 65535UL);
  result.temperature_f100 = -4900 + (int32_t)(31500UL * t_raw / 65535UL);
  result.humidity_rh100 = (int32_t)(10000UL * h_raw / 65535UL);
  *reading = result;
  return SHT31_OK;
}

SHT31_Status SHT31_Read(I2C_HandleTypeDef *i2c, SHT31_Reading *reading) {
  if (i2c == NULL || reading == NULL) {
    return SHT31_INVALID_ARGUMENT;
  }
  uint8_t command[2] = {0x24, 0x00};
  uint8_t response[6];

  if (HAL_I2C_Master_Transmit(i2c, SHT31_ADDRESS, command, sizeof(command),
                              SHT31_TIMEOUT_MS) != HAL_OK) {
    return SHT31_TX_ERROR;
  }
  HAL_Delay(SHT31_CONVERSION_MS);
  if (HAL_I2C_Master_Receive(i2c, SHT31_ADDRESS, response, sizeof(response),
                             SHT31_TIMEOUT_MS) != HAL_OK) {
    return SHT31_RX_ERROR;
  }
  return SHT31_Decode(response, sizeof(response), reading);
}

const char *SHT31_StatusText(SHT31_Status status) {
  switch (status) {
    case SHT31_OK: return "OK";
    case SHT31_TX_ERROR: return "command transfer failed";
    case SHT31_RX_ERROR: return "response read failed";
    case SHT31_CRC_ERROR: return "CRC mismatch";
    case SHT31_INVALID_ARGUMENT: return "invalid argument";
    default: return "unknown error";
  }
}

