#ifndef AD5328_H
#define AD5328_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
    AD5328_CH_A = 0,
    AD5328_CH_B = 1,
    AD5328_CH_C = 2,
    AD5328_CH_D = 3,
    AD5328_CH_E = 4,
    AD5328_CH_F = 5,
    AD5328_CH_G = 6,
    AD5328_CH_H = 7
} AD5328_Channel;

typedef struct
{
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *sync_port;
    uint16_t sync_pin;

    GPIO_TypeDef *ldac_port;
    uint16_t ldac_pin;

    float vref;
    /* Усиление внешнего ОУ после ЦАП; напряжения API заданы после ОУ. */
    float output_gain;
} AD5328_HandleTypeDef;

HAL_StatusTypeDef AD5328_Init(AD5328_HandleTypeDef *dev);

HAL_StatusTypeDef AD5328_WriteRaw(
    AD5328_HandleTypeDef *dev,
    AD5328_Channel channel,
    uint16_t value
);

HAL_StatusTypeDef AD5328_WriteVoltage(
    AD5328_HandleTypeDef *dev,
    AD5328_Channel channel,
    float voltage
);

HAL_StatusTypeDef AD5328_WriteABCD_Raw(
    AD5328_HandleTypeDef *dev,
    uint16_t a,
    uint16_t b,
    uint16_t c,
    uint16_t d
);

HAL_StatusTypeDef AD5328_WriteABCD_Voltage(
    AD5328_HandleTypeDef *dev,
    float va,
    float vb,
    float vc,
    float vd
);

void AD5328_LDAC_Pulse(AD5328_HandleTypeDef *dev);

HAL_StatusTypeDef AD5328_ResetData(AD5328_HandleTypeDef *dev);
HAL_StatusTypeDef AD5328_ResetDataAndControl(AD5328_HandleTypeDef *dev);

HAL_StatusTypeDef AD5328_SetGainABCD_X2(AD5328_HandleTypeDef *dev);
HAL_StatusTypeDef AD5328_SetGainABCD_X1(AD5328_HandleTypeDef *dev);

HAL_StatusTypeDef AD5328_SetGainAll_X2(AD5328_HandleTypeDef *dev);
HAL_StatusTypeDef AD5328_SetGainAll_X1(AD5328_HandleTypeDef *dev);

#endif
