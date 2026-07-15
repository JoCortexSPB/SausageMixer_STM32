#include "ads1115.h"

static HAL_StatusTypeDef ADS1115_WriteRegister(
    ADS1115_HandleTypeDef *dev,
    uint8_t reg,
    uint16_t value
)
{
    uint8_t data[3];

    data[0] = reg;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value & 0xFF);

    return HAL_I2C_Master_Transmit(dev->hi2c, dev->address, data, 3, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef ADS1115_ReadRegister(
    ADS1115_HandleTypeDef *dev,
    uint8_t reg,
    uint16_t *value
)
{
    HAL_StatusTypeDef status;
    uint8_t data[2];

    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_I2C_Master_Receive(dev->hi2c, dev->address, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return status;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];

    return HAL_OK;
}

static uint16_t ADS1115_GetMuxBitsSingleEnded(ADS1115_Channel channel)
{
    /*
       MUX bits для single-ended:
       AIN0-GND = 100
       AIN1-GND = 101
       AIN2-GND = 110
       AIN3-GND = 111
    */

    switch (channel)
    {
        case ADS1115_CHANNEL_0: return 0x04;
        case ADS1115_CHANNEL_1: return 0x05;
        case ADS1115_CHANNEL_2: return 0x06;
        case ADS1115_CHANNEL_3: return 0x07;
        default:                return 0x04;
    }
}

static uint32_t ADS1115_GetConversionDelayMs(ADS1115_DataRate dr)
{
    /*
       В single-shot conversion time примерно 1 / SPS.
       Добавим небольшой запас.
    */

    switch (dr)
    {
        case ADS1115_DR_8SPS:   return 130;
        case ADS1115_DR_16SPS:  return 70;
        case ADS1115_DR_32SPS:  return 35;
        case ADS1115_DR_64SPS:  return 20;
        case ADS1115_DR_128SPS: return 10;
        case ADS1115_DR_250SPS: return 6;
        case ADS1115_DR_475SPS: return 4;
        case ADS1115_DR_860SPS: return 2;
        default:                return 10;
    }
}

HAL_StatusTypeDef ADS1115_Init(ADS1115_HandleTypeDef *dev)
{
    if (dev == 0 || dev->hi2c == 0)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->address, 3, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ADS1115_ReadRawSingleEnded(
    ADS1115_HandleTypeDef *dev,
    ADS1115_Channel channel,
    int16_t *raw
)
{
    if (dev == 0 || raw == 0)
    {
        return HAL_ERROR;
    }

    uint16_t mux = ADS1115_GetMuxBitsSingleEnded(channel);

    /*
       CONFIG REGISTER:

       bit15    OS      = 1       start single conversion
       bits14:12 MUX    = channel single-ended
       bits11:9  PGA    = dev->pga
       bit8     MODE    = 1       single-shot / power-down
       bits7:5  DR      = dev->data_rate
       bit4     COMP_MODE = 0
       bit3     COMP_POL  = 0
       bit2     COMP_LAT  = 0
       bits1:0  COMP_QUE  = 11    comparator disabled
    */

    uint16_t config = 0;

    config |= (1U << 15);
    config |= (mux & 0x07) << 12;
    config |= ((uint16_t)dev->pga & 0x07) << 9;
    config |= (1U << 8);
    config |= ((uint16_t)dev->data_rate & 0x07) << 5;
    config |= 0x0003;

    HAL_StatusTypeDef status;

    status = ADS1115_WriteRegister(dev, ADS1115_REG_CONFIG, config);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(ADS1115_GetConversionDelayMs(dev->data_rate));

    uint16_t value;

    status = ADS1115_ReadRegister(dev, ADS1115_REG_CONVERSION, &value);
    if (status != HAL_OK)
    {
        return status;
    }

    *raw = (int16_t)value;

    return HAL_OK;
}

float ADS1115_RawToVoltage(ADS1115_HandleTypeDef *dev, int16_t raw)
{
    float fsr;

    switch (dev->pga)
    {
        case ADS1115_PGA_6_144V: fsr = 6.144f; break;
        case ADS1115_PGA_4_096V: fsr = 4.096f; break;
        case ADS1115_PGA_2_048V: fsr = 2.048f; break;
        case ADS1115_PGA_1_024V: fsr = 1.024f; break;
        case ADS1115_PGA_0_512V: fsr = 0.512f; break;
        case ADS1115_PGA_0_256V: fsr = 0.256f; break;
        default:                 fsr = 4.096f; break;
    }

    /*
       ADS1115 — знаковый 16-битный АЦП.
       Для single-ended измерений диапазон raw обычно 0...32767.
       LSB = FSR / 32768.
    */

    return ((float)raw * fsr) / 32768.0f;
}

HAL_StatusTypeDef ADS1115_ReadVoltageSingleEnded(
    ADS1115_HandleTypeDef *dev,
    ADS1115_Channel channel,
    float *voltage
)
{
    int16_t raw;

    HAL_StatusTypeDef status = ADS1115_ReadRawSingleEnded(dev, channel, &raw);
    if (status != HAL_OK)
    {
        return status;
    }

    *voltage = ADS1115_RawToVoltage(dev, raw);

    return HAL_OK;
}
