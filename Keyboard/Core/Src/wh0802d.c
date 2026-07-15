#include "wh0802d.h"

/*
   Команды HD44780
*/
#define LCD_CMD_CLEAR_DISPLAY        0x01
#define LCD_CMD_RETURN_HOME          0x02
#define LCD_CMD_ENTRY_MODE_SET       0x04
#define LCD_CMD_DISPLAY_CONTROL      0x08
#define LCD_CMD_CURSOR_SHIFT         0x10
#define LCD_CMD_FUNCTION_SET         0x20
#define LCD_CMD_SET_CGRAM_ADDR       0x40
#define LCD_CMD_SET_DDRAM_ADDR       0x80

/*
   Entry mode
*/
#define LCD_ENTRY_INCREMENT          0x02
#define LCD_ENTRY_NO_SHIFT           0x00

/*
   Display control
*/
#define LCD_DISPLAY_ON               0x04
#define LCD_DISPLAY_OFF              0x00
#define LCD_CURSOR_ON                0x02
#define LCD_CURSOR_OFF               0x00
#define LCD_BLINK_ON                 0x01
#define LCD_BLINK_OFF                0x00

/*
   Function set
*/
#define LCD_FUNCTION_8BIT            0x10
#define LCD_FUNCTION_4BIT            0x00
#define LCD_FUNCTION_2LINE           0x08
#define LCD_FUNCTION_1LINE           0x00
#define LCD_FUNCTION_5x8             0x00
#define LCD_FUNCTION_5x10            0x04

static void WH0802D_DelayUs(uint32_t us)
{
    /*
       Для STM32F030 @ 48 MHz.
       Грубая задержка, но для LCD более чем достаточно.

       Если включён DWT — на Cortex-M0 его обычно нет,
       поэтому используем простой цикл.
    */

    volatile uint32_t count;

    while (us--)
    {
        /*
           Подобрано с запасом. Точная микросекунда здесь не критична.
           LCD намного медленнее МК.
        */
        count = 8;
        while (count--)
        {
            __NOP();
        }
    }
}

static void WH0802D_SetDataBus(WH0802D_HandleTypeDef *lcd, uint8_t value)
{
    /*
       PB0...PB7 напрямую соответствуют D0...D7.

       Сначала очищаем младшие 8 бит порта,
       затем выставляем нужное значение.
    */

    uint32_t odr = lcd->data_port->ODR;

    odr &= ~0x00FFu;
    odr |= value;

    lcd->data_port->ODR = odr;
}

static void WH0802D_PulseEnable(WH0802D_HandleTypeDef *lcd)
{
    HAL_GPIO_WritePin(lcd->ctrl_port, lcd->e_pin, GPIO_PIN_SET);

    WH0802D_DelayUs(2);

    HAL_GPIO_WritePin(lcd->ctrl_port, lcd->e_pin, GPIO_PIN_RESET);

    WH0802D_DelayUs(50);
}

static void WH0802D_Write8(
    WH0802D_HandleTypeDef *lcd,
    uint8_t value,
    GPIO_PinState rs_state
)
{
    HAL_GPIO_WritePin(lcd->ctrl_port, lcd->rs_pin, rs_state);

    /*
       Только запись.
    */
    HAL_GPIO_WritePin(lcd->ctrl_port, lcd->rw_pin, GPIO_PIN_RESET);

    WH0802D_SetDataBus(lcd, value);
    WH0802D_PulseEnable(lcd);
}

void WH0802D_Command(WH0802D_HandleTypeDef *lcd, uint8_t cmd)
{
    WH0802D_Write8(lcd, cmd, GPIO_PIN_RESET);

    /*
       Clear Display и Return Home выполняются дольше остальных команд.
    */
    if (cmd == LCD_CMD_CLEAR_DISPLAY || cmd == LCD_CMD_RETURN_HOME)
    {
        HAL_Delay(2);
    }
    else
    {
        WH0802D_DelayUs(50);
    }
}

void WH0802D_Data(WH0802D_HandleTypeDef *lcd, uint8_t data)
{
    WH0802D_Write8(lcd, data, GPIO_PIN_SET);
    WH0802D_DelayUs(50);
}

void WH0802D_Init(WH0802D_HandleTypeDef *lcd)
{
    /*
       После подачи питания LCD нужно время на внутренний reset.
       Для HD44780 обычно рекомендуют ждать >15 ms.
    */
    HAL_Delay(50);

    HAL_GPIO_WritePin(lcd->ctrl_port, lcd->rs_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->ctrl_port, lcd->rw_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->ctrl_port, lcd->e_pin,  GPIO_PIN_RESET);

    /*
       Инициализация 8-битного режима.
       Для надёжности три раза отправляем Function Set.
    */

    WH0802D_Write8(lcd,
                   LCD_CMD_FUNCTION_SET |
                   LCD_FUNCTION_8BIT |
                   LCD_FUNCTION_2LINE |
                   LCD_FUNCTION_5x8,
                   GPIO_PIN_RESET);

    HAL_Delay(5);

    WH0802D_Write8(lcd,
                   LCD_CMD_FUNCTION_SET |
                   LCD_FUNCTION_8BIT |
                   LCD_FUNCTION_2LINE |
                   LCD_FUNCTION_5x8,
                   GPIO_PIN_RESET);

    WH0802D_DelayUs(150);

    WH0802D_Write8(lcd,
                   LCD_CMD_FUNCTION_SET |
                   LCD_FUNCTION_8BIT |
                   LCD_FUNCTION_2LINE |
                   LCD_FUNCTION_5x8,
                   GPIO_PIN_RESET);

    WH0802D_DelayUs(150);

    /*
       Финальная настройка:
       8 бит, 2 строки, символ 5x8.
    */
    WH0802D_Command(lcd,
                    LCD_CMD_FUNCTION_SET |
                    LCD_FUNCTION_8BIT |
                    LCD_FUNCTION_2LINE |
                    LCD_FUNCTION_5x8);

    /*
       Дисплей включен, курсор выключен, мигание выключено.
    */
    WH0802D_Command(lcd,
                    LCD_CMD_DISPLAY_CONTROL |
                    LCD_DISPLAY_ON |
                    LCD_CURSOR_OFF |
                    LCD_BLINK_OFF);

    /*
       Очистка.
    */
    WH0802D_Clear(lcd);

    /*
       Автоинкремент адреса, без сдвига экрана.
    */
    WH0802D_Command(lcd,
                    LCD_CMD_ENTRY_MODE_SET |
                    LCD_ENTRY_INCREMENT |
                    LCD_ENTRY_NO_SHIFT);
}

void WH0802D_Clear(WH0802D_HandleTypeDef *lcd)
{
    WH0802D_Command(lcd, LCD_CMD_CLEAR_DISPLAY);
}

void WH0802D_Home(WH0802D_HandleTypeDef *lcd)
{
    WH0802D_Command(lcd, LCD_CMD_RETURN_HOME);
}

void WH0802D_SetCursor(WH0802D_HandleTypeDef *lcd, uint8_t col, uint8_t row)
{
    if (col >= WH0802D_COLS)
    {
        col = WH0802D_COLS - 1;
    }

    if (row >= WH0802D_ROWS)
    {
        row = WH0802D_ROWS - 1;
    }

    /*
       Для 8x2 HD44780 обычно:
       строка 0: DDRAM 0x00
       строка 1: DDRAM 0x40
    */

    uint8_t row_offsets[] = {0x00, 0x40};
    uint8_t address = row_offsets[row] + col;

    WH0802D_Command(lcd, LCD_CMD_SET_DDRAM_ADDR | address);
}

void WH0802D_WriteChar(WH0802D_HandleTypeDef *lcd, char c)
{
    WH0802D_Data(lcd, (uint8_t)c);
}

void WH0802D_WriteString(WH0802D_HandleTypeDef *lcd, const char *str)
{
    if (str == NULL)
    {
        return;
    }

    while (*str)
    {
        WH0802D_WriteChar(lcd, *str++);
    }
}

void WH0802D_WriteStringAt(
    WH0802D_HandleTypeDef *lcd,
    uint8_t col,
    uint8_t row,
    const char *str
)
{
    WH0802D_SetCursor(lcd, col, row);
    WH0802D_WriteString(lcd, str);
}
