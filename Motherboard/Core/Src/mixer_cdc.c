#include "mixer_cdc.h"

#include "usbd_cdc_if.h"

#include <string.h>

#define MIXER_CDC_RX_BUFFER_SIZE       256U
#define MIXER_CDC_LINE_SIZE             64U
#define MIXER_CDC_COMMAND_QUEUE_SIZE     8U

static volatile uint8_t rx_buffer[MIXER_CDC_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static char line_buffer[MIXER_CDC_LINE_SIZE];
static uint16_t line_length = 0;
static uint8_t line_discard = 0U;

static MixerCDC_Command command_queue[MIXER_CDC_COMMAND_QUEUE_SIZE];
static uint8_t command_head = 0;
static uint8_t command_tail = 0;

volatile uint32_t mixer_cdc_rx_overflow_count = 0;
volatile uint32_t mixer_cdc_command_overflow_count = 0;

static void MixerCDC_ParseLine(char *line);
static void MixerCDC_PushCommand(const MixerCDC_Command *command);
static uint32_t MixerCDC_VoltageToMillivolts(float voltage);
static uint8_t MixerCDC_ParseVoltage(const char *text, float *value);
static uint8_t MixerCDC_ParseUnsigned(const char *text, uint32_t *value);
static uint16_t MixerCDC_AppendText(uint8_t *buffer, uint16_t position, const char *text);
static uint16_t MixerCDC_AppendUnsigned(uint8_t *buffer, uint16_t position, uint32_t value);
static uint16_t MixerCDC_AppendVoltage(uint8_t *buffer, uint16_t position, float voltage);

void MixerCDC_ReceiveFromISR(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == NULL)
    {
        return;
    }

    for (index = 0; index < length; index++)
    {
        uint16_t next_head = (uint16_t)((rx_head + 1U) % MIXER_CDC_RX_BUFFER_SIZE);

        if (next_head == rx_tail)
        {
            mixer_cdc_rx_overflow_count++;
            break;
        }

        rx_buffer[rx_head] = data[index];
        rx_head = next_head;
    }
}

void MixerCDC_Process(void)
{
    while (rx_tail != rx_head)
    {
        char byte = (char)rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % MIXER_CDC_RX_BUFFER_SIZE);

        if (byte == '\r')
        {
            continue;
        }

        if (byte == '\n')
        {
            if ((line_discard == 0U) && (line_length > 0U))
            {
                line_buffer[line_length] = '\0';
                MixerCDC_ParseLine(line_buffer);
            }
            line_length = 0U;
            line_discard = 0U;
            continue;
        }

        if (line_discard != 0U)
        {
            continue;
        }

        if (line_length < (MIXER_CDC_LINE_SIZE - 1U))
        {
            line_buffer[line_length++] = byte;
        }
        else
        {
            /* Слишком длинная строка отбрасывается до следующего перевода строки. */
            line_length = 0U;
            line_discard = 1U;
        }
    }
}

uint8_t MixerCDC_PopCommand(MixerCDC_Command *command)
{
    if ((command == NULL) || (command_tail == command_head))
    {
        return 0U;
    }

    *command = command_queue[command_tail];
    command_tail = (uint8_t)((command_tail + 1U) % MIXER_CDC_COMMAND_QUEUE_SIZE);
    return 1U;
}

uint8_t MixerCDC_SendTelemetry(const MixerCDC_Telemetry *telemetry)
{
    static uint8_t message[160];
    uint16_t length = 0U;

    if (telemetry == NULL)
    {
        return 1U;
    }

    length = MixerCDC_AppendText(message, length, "TEL,");
    length = MixerCDC_AppendVoltage(message, length, telemetry->pressure_v);
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->pressure_target_v);
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->reg_itv_v);
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->reg_carousel_v);
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->reg_carriage_v);
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->airout_v);
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->carriage_speed_v);
    message[length++] = ',';
    message[length++] = telemetry->bigairon ? '1' : '0';
    message[length++] = ',';
    message[length++] = telemetry->pressure_valid ? '1' : '0';
    message[length++] = ',';
    message[length++] = telemetry->pressure_ready ? '1' : '0';
    message[length++] = ',';
    length = MixerCDC_AppendUnsigned(message, length, telemetry->state);
    message[length++] = ',';
    length = MixerCDC_AppendUnsigned(message, length, telemetry->stage);
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->meter_m);
    message[length++] = ',';
    length = MixerCDC_AppendUnsigned(message, length, telemetry->cycle_count);
    message[length++] = ',';
    length = MixerCDC_AppendUnsigned(message, length, telemetry->product_count);
    message[length++] = ',';
    message[length++] = telemetry->pause_enabled ? '1' : '0';
    message[length++] = ',';
    length = MixerCDC_AppendVoltage(message, length, telemetry->pause_length_m);
    message[length++] = ',';
    length = MixerCDC_AppendUnsigned(message, length, telemetry->sensors);
    message[length++] = ',';
    message[length++] = telemetry->carriage_on ? '1' : '0';
    message[length++] = ',';
    message[length++] = telemetry->vf_forward ? '1' : '0';
    message[length++] = ',';
    message[length++] = telemetry->vf_reverse ? '1' : '0';
    message[length++] = ',';
    message[length++] = telemetry->cycle_active ? '1' : '0';
    message[length++] = '\n';

    return CDC_Transmit_FS(message, length);
}

uint8_t MixerCDC_SendProtocolInfo(void)
{
    static uint8_t message[] = "PROTO,2\n";
    return CDC_Transmit_FS(message, (uint16_t)(sizeof(message) - 1U));
}

static void MixerCDC_ParseLine(char *line)
{
    MixerCDC_Command command = {0};

    if (strcmp(line, "VERSION") == 0)
    {
        command.type = MIXER_CDC_CMD_VERSION;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "START") == 0)
    {
        command.type = MIXER_CDC_CMD_START;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "STOP") == 0)
    {
        command.type = MIXER_CDC_CMD_STOP;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "RESET") == 0)
    {
        command.type = MIXER_CDC_CMD_RESET;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "LEFT") == 0)
    {
        command.type = MIXER_CDC_CMD_LEFT;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "COUNTERS,RESET") == 0)
    {
        command.type = MIXER_CDC_CMD_COUNTERS_RESET;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "PAUSE,1") == 0)
    {
        command.type = MIXER_CDC_CMD_PAUSE_ENABLE;
        command.value = 1.0f;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "PAUSE,0") == 0)
    {
        command.type = MIXER_CDC_CMD_PAUSE_ENABLE;
        command.value = 0.0f;
        MixerCDC_PushCommand(&command);
    }
    else if (strncmp(line, "LENGTH,", 7U) == 0)
    {
        float value;
        if (MixerCDC_ParseVoltage(&line[7], &value) != 0U)
        {
            command.type = MIXER_CDC_CMD_PAUSE_LENGTH;
            command.value = value;
            MixerCDC_PushCommand(&command);
        }
    }
    else if (strcmp(line, "AIR,1") == 0)
    {
        command.type = MIXER_CDC_CMD_AIR;
        command.value = 1.0f;
        MixerCDC_PushCommand(&command);
    }
    else if (strcmp(line, "AIR,0") == 0)
    {
        command.type = MIXER_CDC_CMD_AIR;
        command.value = 0.0f;
        MixerCDC_PushCommand(&command);
    }
    else if (strncmp(line, "AIROUT,", 7U) == 0)
    {
        float value;
        if (MixerCDC_ParseVoltage(&line[7], &value) != 0U)
        {
            command.type = MIXER_CDC_CMD_AIROUT;
            command.value = value;
            MixerCDC_PushCommand(&command);
        }
    }
    else if (strncmp(line, "STREAM,START,", 13U) == 0)
    {
        uint32_t period;
        if (MixerCDC_ParseUnsigned(&line[13], &period) != 0U)
        {
            command.type = MIXER_CDC_CMD_STREAM_START;
            command.period_ms = period;
            MixerCDC_PushCommand(&command);
        }
    }
    else if (strcmp(line, "STREAM,STOP") == 0)
    {
        command.type = MIXER_CDC_CMD_STREAM_STOP;
        MixerCDC_PushCommand(&command);
    }
    else if (strncmp(line, "DROP,", 5U) == 0)
    {
        float value;
        if (MixerCDC_ParseVoltage(&line[5], &value) != 0U)
        {
            command.type = MIXER_CDC_CMD_DROP_THRESHOLD;
            command.value = value;
            MixerCDC_PushCommand(&command);
        }
    }
    else if (strcmp(line, "PING") == 0)
    {
        command.type = MIXER_CDC_CMD_PING;
        MixerCDC_PushCommand(&command);
    }
}

static void MixerCDC_PushCommand(const MixerCDC_Command *command)
{
    uint8_t next_head = (uint8_t)((command_head + 1U) % MIXER_CDC_COMMAND_QUEUE_SIZE);

    if (next_head == command_tail)
    {
        mixer_cdc_command_overflow_count++;
        return;
    }

    command_queue[command_head] = *command;
    command_head = next_head;
}

static uint32_t MixerCDC_VoltageToMillivolts(float voltage)
{
    if (voltage <= 0.0f)
    {
        return 0U;
    }

    if (voltage >= 100000.0f)
    {
        return 100000000U;
    }

    return (uint32_t)(voltage * 1000.0f + 0.5f);
}

static uint8_t MixerCDC_ParseVoltage(const char *text, float *value)
{
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint32_t fraction_scale = 1U;
    uint8_t has_digit = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        whole = whole * 10U + (uint32_t)(*text - '0');
        has_digit = 1U;
        text++;
    }

    if (*text == '.')
    {
        text++;
        while ((*text >= '0') && (*text <= '9'))
        {
            if (fraction_scale < 1000U)
            {
                fraction = fraction * 10U + (uint32_t)(*text - '0');
                fraction_scale *= 10U;
            }
            has_digit = 1U;
            text++;
        }
    }

    if ((has_digit == 0U) || (*text != '\0'))
    {
        return 0U;
    }

    *value = (float)whole + ((float)fraction / (float)fraction_scale);
    return 1U;
}

static uint8_t MixerCDC_ParseUnsigned(const char *text, uint32_t *value)
{
    uint32_t result = 0U;
    uint8_t has_digit = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        result = result * 10U + (uint32_t)(*text - '0');
        has_digit = 1U;
        text++;
    }

    if ((has_digit == 0U) || (*text != '\0'))
    {
        return 0U;
    }

    *value = result;
    return 1U;
}

static uint16_t MixerCDC_AppendText(uint8_t *buffer, uint16_t position, const char *text)
{
    while (*text != '\0')
    {
        buffer[position++] = (uint8_t)*text++;
    }
    return position;
}

static uint16_t MixerCDC_AppendUnsigned(uint8_t *buffer, uint16_t position, uint32_t value)
{
    uint8_t digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count++] = (uint8_t)('0' + (value % 10U));
        value /= 10U;
    }
    while ((value > 0U) && (count < sizeof(digits)));

    while (count > 0U)
    {
        buffer[position++] = digits[--count];
    }

    return position;
}

static uint16_t MixerCDC_AppendVoltage(uint8_t *buffer, uint16_t position, float voltage)
{
    uint32_t millivolts = MixerCDC_VoltageToMillivolts(voltage);
    uint32_t fraction = millivolts % 1000U;

    position = MixerCDC_AppendUnsigned(buffer, position, millivolts / 1000U);
    buffer[position++] = '.';
    buffer[position++] = (uint8_t)('0' + ((fraction / 100U) % 10U));
    buffer[position++] = (uint8_t)('0' + ((fraction / 10U) % 10U));
    buffer[position++] = (uint8_t)('0' + (fraction % 10U));
    return position;
}
