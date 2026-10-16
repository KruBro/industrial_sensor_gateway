#ifndef LCDI2C_H
#define LCDI2C_H

#include "main.h"

#define LCD_I2C_ADDR (0x27 << 1)

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t column);
void LCD_Print(char *str);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);

#endif
