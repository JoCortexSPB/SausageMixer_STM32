#ifndef MIXER_CDC_H
#define MIXER_CDC_H

#include <stdint.h>

typedef enum
{
    MIXER_CDC_CMD_NONE = 0,
    MIXER_CDC_CMD_AIR,
    MIXER_CDC_CMD_AIROUT,
    MIXER_CDC_CMD_STREAM_START,
    MIXER_CDC_CMD_STREAM_STOP,
    MIXER_CDC_CMD_DROP_THRESHOLD,
    MIXER_CDC_CMD_PING
} MixerCDC_CommandType;

typedef struct
{
    MixerCDC_CommandType type;
    float value;
    uint32_t period_ms;
} MixerCDC_Command;

/* Вызывается из USB CDC callback. Функция только копирует байты в кольцевой буфер. */
void MixerCDC_ReceiveFromISR(const uint8_t *data, uint32_t length);

/* Вызывается в основном цикле и разбирает накопленные текстовые строки. */
void MixerCDC_Process(void);

uint8_t MixerCDC_PopCommand(MixerCDC_Command *command);

/* TEL,<ITV_PRESSURE>,<REG_ITV>,<AIROUT>,<bigairon> */
uint8_t MixerCDC_SendTelemetry(
    float pressure_v,
    float reg_itv_v,
    float airout_v,
    uint8_t bigairon,
    uint8_t pressure_valid
);

extern volatile uint32_t mixer_cdc_rx_overflow_count;
extern volatile uint32_t mixer_cdc_command_overflow_count;

#endif
