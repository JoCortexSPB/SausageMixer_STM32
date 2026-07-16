#ifndef MIXER_CDC_H
#define MIXER_CDC_H

#include <stdint.h>

#define MIXER_CDC_PROTOCOL_VERSION 2U

typedef enum
{
    MIXER_CDC_CMD_NONE = 0,
    MIXER_CDC_CMD_AIR,
    MIXER_CDC_CMD_AIROUT,
    MIXER_CDC_CMD_STREAM_START,
    MIXER_CDC_CMD_STREAM_STOP,
    MIXER_CDC_CMD_DROP_THRESHOLD,
    MIXER_CDC_CMD_START,
    MIXER_CDC_CMD_STOP,
    MIXER_CDC_CMD_RESET,
    MIXER_CDC_CMD_LEFT,
    MIXER_CDC_CMD_PAUSE_ENABLE,
    MIXER_CDC_CMD_PAUSE_LENGTH,
    MIXER_CDC_CMD_COUNTERS_RESET,
    MIXER_CDC_CMD_VERSION,
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

typedef struct
{
    float pressure_v;
    float pressure_target_v;
    float reg_itv_v;
    float reg_carousel_v;
    float reg_carriage_v;
    float airout_v;
    float carriage_speed_v;
    float meter_m;
    float pause_length_m;
    uint32_t cycle_count;
    uint32_t product_count;
    uint8_t bigairon;
    uint8_t pressure_valid;
    uint8_t pressure_ready;
    uint8_t state;
    uint8_t stage;
    uint8_t pause_enabled;
    uint8_t sensors;
    uint8_t carriage_on;
    uint8_t vf_forward;
    uint8_t vf_reverse;
    uint8_t cycle_active;
} MixerCDC_Telemetry;

uint8_t MixerCDC_SendTelemetry(
    const MixerCDC_Telemetry *telemetry
);

uint8_t MixerCDC_SendProtocolInfo(void);

extern volatile uint32_t mixer_cdc_rx_overflow_count;
extern volatile uint32_t mixer_cdc_command_overflow_count;

#endif
