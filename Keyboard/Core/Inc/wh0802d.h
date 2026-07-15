#ifndef WH0802D_H
#define WH0802D_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define WH0802D_COLS    8
#define WH0802D_ROWS    2

typedef struct
{
    GPIO_TypeDef *data_port;   // PB0...PB7
    GPIO_TypeDef *ctrl_port;   // PB8, PB9, PB10

    uint16_t rs_pin;
    uint16_t rw_pin;
    uint16_t e_pin;
} WH0802D_HandleTypeDef;

void WH0802D_Init(WH0802D_HandleTypeDef *lcd);

void WH0802D_Clear(WH0802D_HandleTypeDef *lcd);
void WH0802D_Home(WH0802D_HandleTypeDef *lcd);

void WH0802D_SetCursor(WH0802D_HandleTypeDef *lcd, uint8_t col, uint8_t row);

void WH0802D_WriteChar(WH0802D_HandleTypeDef *lcd, char c);
void WH0802D_WriteString(WH0802D_HandleTypeDef *lcd, const char *str);

void WH0802D_WriteStringAt(
    WH0802D_HandleTypeDef *lcd,
    uint8_t col,
    uint8_t row,
    const char *str
);

void WH0802D_Command(WH0802D_HandleTypeDef *lcd, uint8_t cmd);
void WH0802D_Data(WH0802D_HandleTypeDef *lcd, uint8_t data);

#endif

/* Пример первичной инициализации

  WH0802D_HandleTypeDef lcd;

  lcd.data_port = GPIOB;
  lcd.ctrl_port = GPIOB;

  lcd.rs_pin = GPIO_PIN_8;
  lcd.rw_pin = GPIO_PIN_9;
  lcd.e_pin = GPIO_PIN_10;

  WH0802D_Init(&lcd);

  WH0802D_Clear(&lcd);
  WH0802D_SetCursor(&lcd, 0, 0);
  WH0802D_WriteString(&lcd, "HELLO");

 * */
