#ifndef ADS1115_H
#define ADS1115_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define ADS1115_DEFAULT_ADDR      (0x48 << 1)

#define ADS1115_REG_CONVERSION    0x00
#define ADS1115_REG_CONFIG        0x01
#define ADS1115_REG_LO_THRESH     0x02
#define ADS1115_REG_HI_THRESH     0x03

typedef enum
{
    ADS1115_CHANNEL_0 = 0,
    ADS1115_CHANNEL_1 = 1,
    ADS1115_CHANNEL_2 = 2,
    ADS1115_CHANNEL_3 = 3
} ADS1115_Channel;

typedef enum
{
    ADS1115_PGA_6_144V = 0,
    ADS1115_PGA_4_096V = 1,
    ADS1115_PGA_2_048V = 2,
    ADS1115_PGA_1_024V = 3,
    ADS1115_PGA_0_512V = 4,
    ADS1115_PGA_0_256V = 5
} ADS1115_PGA;

typedef enum
{
    ADS1115_DR_8SPS   = 0,
    ADS1115_DR_16SPS  = 1,
    ADS1115_DR_32SPS  = 2,
    ADS1115_DR_64SPS  = 3,
    ADS1115_DR_128SPS = 4,
    ADS1115_DR_250SPS = 5,
    ADS1115_DR_475SPS = 6,
    ADS1115_DR_860SPS = 7
} ADS1115_DataRate;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;

    ADS1115_PGA pga;
    ADS1115_DataRate data_rate;
} ADS1115_HandleTypeDef;

HAL_StatusTypeDef ADS1115_Init(ADS1115_HandleTypeDef *dev);

HAL_StatusTypeDef ADS1115_ReadRawSingleEnded(
    ADS1115_HandleTypeDef *dev,
    ADS1115_Channel channel,
    int16_t *raw
);

HAL_StatusTypeDef ADS1115_ReadVoltageSingleEnded(
    ADS1115_HandleTypeDef *dev,
    ADS1115_Channel channel,
    float *voltage
);

float ADS1115_RawToVoltage(ADS1115_HandleTypeDef *dev, int16_t raw);

#endif
