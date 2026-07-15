#include "ad5328.h"

#define AD5328_MAX_CODE 4095U

static HAL_StatusTypeDef AD5328_SendWord(AD5328_HandleTypeDef *dev, uint16_t word)
{
    uint8_t data[2];

    data[0] = (uint8_t)(word >> 8);
    data[1] = (uint8_t)(word & 0xFF);

    HAL_GPIO_WritePin(dev->sync_port, dev->sync_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, data, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(dev->sync_port, dev->sync_pin, GPIO_PIN_SET);

    return status;
}

/*HAL_StatusTypeDef AD5328_Init(AD5328_HandleTypeDef *dev)
{
    if (dev == 0 || dev->hspi == 0)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(dev->sync_port, dev->sync_pin, GPIO_PIN_SET);


    HAL_GPIO_WritePin(dev->ldac_port, dev->ldac_pin, GPIO_PIN_SET);

    HAL_StatusTypeDef status;

    status = AD5328_ResetDataAndControl(dev);
    if (status != HAL_OK) return status;


        status = AD5328_SetGainABCD_X2(dev);
        if (status != HAL_OK) return status;

        return HAL_OK;
}*/

HAL_StatusTypeDef AD5328_Init(AD5328_HandleTypeDef *dev)
{
    if (dev == 0 || dev->hspi == 0)
    {
        return HAL_ERROR;
    }

    // SYNC в покое HIGH
    HAL_GPIO_WritePin(dev->sync_port, dev->sync_pin, GPIO_PIN_SET);

    // ВАЖНО: временно LDAC постоянно LOW
    // Тогда выход обновляется сразу после записи DAC-регистра
    HAL_GPIO_WritePin(dev->ldac_port, dev->ldac_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status;

    status = AD5328_ResetDataAndControl(dev);
    if (status != HAL_OK) return status;

    status = AD5328_SetGainABCD_X2(dev);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef AD5328_WriteRaw(
    AD5328_HandleTypeDef *dev,
    AD5328_Channel channel,
    uint16_t value
)
{
    if (channel > AD5328_CH_H)
    {
        return HAL_ERROR;
    }

    if (value > AD5328_MAX_CODE)
    {
        value = AD5328_MAX_CODE;
    }

    /*
       Формат AD5328 DAC Write:

       Bit 15    = 0       DAC write
       Bits 14:12 = channel
       Bits 11:0  = 12-bit DAC code
    */

    uint16_t word = ((uint16_t)channel << 12) | (value & 0x0FFF);

    return AD5328_SendWord(dev, word);
}

HAL_StatusTypeDef AD5328_WriteVoltage(
    AD5328_HandleTypeDef *dev,
    AD5328_Channel channel,
    float voltage
)
{
    float full_scale = dev->vref * dev->gain;

    if (voltage < 0.0f)
    {
        voltage = 0.0f;
    }

    if (voltage > full_scale)
    {
        voltage = full_scale;
    }

    uint16_t code = (uint16_t)((voltage / full_scale) * 4095.0f + 0.5f);

    return AD5328_WriteRaw(dev, channel, code);
}

void AD5328_LDAC_Pulse(AD5328_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->ldac_port, dev->ldac_pin, GPIO_PIN_RESET);

    /*
       Для STM32F103 при обычных частотах этого уже достаточно.
       Если нужно — можно добавить короткую задержку через NOP.
    */
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    HAL_GPIO_WritePin(dev->ldac_port, dev->ldac_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef AD5328_WriteABCD_Raw(
    AD5328_HandleTypeDef *dev,
    uint16_t a,
    uint16_t b,
    uint16_t c,
    uint16_t d
)
{
    HAL_StatusTypeDef status;

    status = AD5328_WriteRaw(dev, AD5328_CH_A, a);
    if (status != HAL_OK) return status;

    status = AD5328_WriteRaw(dev, AD5328_CH_B, b);
    if (status != HAL_OK) return status;

    status = AD5328_WriteRaw(dev, AD5328_CH_C, c);
    if (status != HAL_OK) return status;

    status = AD5328_WriteRaw(dev, AD5328_CH_D, d);
    if (status != HAL_OK) return status;

    AD5328_LDAC_Pulse(dev);

    return HAL_OK;
}

HAL_StatusTypeDef AD5328_WriteABCD_Voltage(
    AD5328_HandleTypeDef *dev,
    float va,
    float vb,
    float vc,
    float vd
)
{
    HAL_StatusTypeDef status;

    status = AD5328_WriteVoltage(dev, AD5328_CH_A, va);
    if (status != HAL_OK) return status;

    status = AD5328_WriteVoltage(dev, AD5328_CH_B, vb);
    if (status != HAL_OK) return status;

    status = AD5328_WriteVoltage(dev, AD5328_CH_C, vc);
    if (status != HAL_OK) return status;

    status = AD5328_WriteVoltage(dev, AD5328_CH_D, vd);
    if (status != HAL_OK) return status;

    //AD5328_LDAC_Pulse(dev);
return HAL_OK;
}

HAL_StatusTypeDef AD5328_ResetData(AD5328_HandleTypeDef *dev)
{
    /*
       Reset DAC data:
       Bit 15 = 1
       Bit 14 = 1
       Bit 13 = 1
       Bit 12 = 0
    */
    return AD5328_SendWord(dev, 0xE000);
}

HAL_StatusTypeDef AD5328_ResetDataAndControl(AD5328_HandleTypeDef *dev)
{
    /*
       Reset data and control:
       Bit 15 = 1
       Bit 14 = 1
       Bit 13 = 1
       Bit 12 = 1
    */
    return AD5328_SendWord(dev, 0xF000);
}

HAL_StatusTypeDef AD5328_SetGainABCD_X2(AD5328_HandleTypeDef *dev)
{
    /*
       Reference and Gain Mode

       bit15 = 1
       bit14 = 0
       bit13 = 0

       bit4 = 1  => DAC A/B/C/D output range 0...2*VREF

       VDD reference bits bit1/bit0 = 0,
       иначе VDD reference имеет приоритет и gain x2 не сработает как надо.
    */

    return AD5328_SendWord(dev, 0x8010);
}

HAL_StatusTypeDef AD5328_SetGainABCD_X1(AD5328_HandleTypeDef *dev)
{
    /*
       DAC A/B/C/D output range 0...VREF
    */

    return AD5328_SendWord(dev, 0x8000);
}

HAL_StatusTypeDef AD5328_SetGainAll_X2(AD5328_HandleTypeDef *dev)
{
    /*
       bit5 = 1 => DAC E/F/G/H gain x2
       bit4 = 1 => DAC A/B/C/D gain x2
    */

    return AD5328_SendWord(dev, 0x8030);
}

HAL_StatusTypeDef AD5328_SetGainAll_X1(AD5328_HandleTypeDef *dev)
{
    return AD5328_SendWord(dev, 0x8000);
}
