#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "main.h"
#include <stdint.h>

void LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *s);

#endif
