/* Project sensor interface. Units are hundredths: 2345 means 23.45.
 * Protocol: Sensirion SHT3x-DIS datasheet, sections 4.3, 4.4, 4.12, 4.13.
 */
#ifndef SHT31_H
#define SHT31_H

#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <stdint.h>

typedef struct {
  int32_t temperature_c100;
  int32_t temperature_f100;
  int32_t humidity_rh100;
} SHT31_Reading;

typedef enum {
  SHT31_OK,
  SHT31_TX_ERROR,
  SHT31_RX_ERROR,
  SHT31_CRC_ERROR,
  SHT31_INVALID_ARGUMENT
} SHT31_Status;

/* Returns one fresh, CRC-checked reading. Output is unchanged on failure.
 * This blocking call uses 100 ms I2C timeouts and a 20 ms conversion delay.
 */
SHT31_Status SHT31_Read(I2C_HandleTypeDef *i2c, SHT31_Reading *reading);

/* Hardware-independent frame validation/conversion, also used by host tests. */
SHT31_Status SHT31_Decode(const uint8_t *frame, size_t length,
                         SHT31_Reading *reading);
const char *SHT31_StatusText(SHT31_Status status);

#endif

