/* SSD1306 128x64 OLED over I2C. Text only, 5x7 font, uppercase.
 * Drawing writes into a RAM framebuffer; nothing appears on the display
 * until SSD1306_Update() sends that framebuffer to the panel.
 */
#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define SSD1306_WIDTH 128U
#define SSD1306_PAGES 8U /* 64 rows of pixels / 8 rows per page */

HAL_StatusTypeDef SSD1306_SendCommand(I2C_HandleTypeDef *i2c, uint8_t command);

/* Runs the power-up sequence and clears the screen. */
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *i2c);

/* Blank the framebuffer (does not touch the panel until Update). */
void SSD1306_Clear(void);

/* Draw text at pixel column x on text row 0..7 (each row is 8 pixels tall).
 * Lowercase is shown as uppercase; unsupported characters show as '?'.
 */
void SSD1306_DrawText(uint8_t x, uint8_t row, const char *text);

/* Send the whole framebuffer to the panel. */
HAL_StatusTypeDef SSD1306_Update(I2C_HandleTypeDef *i2c);

#endif
