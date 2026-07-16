#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#include "ADS1115.h"
#include "AD5328.h"

/* =========================================================
 * ПЕРИФЕРИЯ
 * ========================================================= */

I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

AD5328_HandleTypeDef dac;
ADS1115_HandleTypeDef adc;

/* =========================================================
 * ПРОТОТИПЫ
 * ========================================================= */

void Motherboard_ProcessCommand(uint8_t cmd);
static void Motherboard_ProcessCDC(void);
static void Motherboard_SendCDCTelemetry(void);
static void Motherboard_HandleCDCLine(char *line);
static uint32_t Motherboard_VoltageToMillivolts(float voltage);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);

void carriage_move_frw(float speed);
void carriage_move_rev(void);
void carriage_stop(void);

void ITV_PressureControl(void);

/* =========================================================
 * СОСТОЯНИЯ СТАНКА
 * ========================================================= */

typedef enum
{
    STATE_IDLE = 0,
    STATE_RUNNING = 1,
    STATE_ERROR = 10
} MachineState_t;

typedef enum
{
    STAGE_NOINIT = 0,
    STAGE_CYCLE = 1,
    STAGE_REWARD = 2,
    STAGE_SEEKHOME = 3
} ProgramStage_t;

/* =========================================================
 * НАСТРОЙКИ РЕГУЛИРОВАНИЯ ITV
 * ========================================================= */

/* REG_ITV 0...5 В задаёт рабочее давление 0...2 бар. */
#define REG_ITV_INPUT_MAX_V           5.0f
#define WORK_PRESSURE_MAX_BAR         2.0f
#define PRESSURE_SENSOR_FULL_BAR      10.0f
#define PRESSURE_SENSOR_ZERO_V        0.40f
#define PRESSURE_SENSOR_SPAN_V        1.60f

/* Быстрая PI-коррекция прямого задания REG_ITV -> AIROUT. */
#define ITV_CONTROL_KP                12.0f
#define ITV_CONTROL_KI                6.0f
#define ITV_CONTROL_DEADBAND_V        0.0015f
#define ITV_INTEGRAL_LIMIT_V          2.0f

/*
 * Давление, при котором разрешается движение каретки.
 */
#define ITV_PRESSURE_READY            0.390f

/*
 * Ограничения управляющего выхода ЦАП.
 */
#define ITV_OUTPUT_MIN                0.0f
#define ITV_OUTPUT_MAX                5.0f

/*
 * Период регулирования.
 */
#define ITV_CONTROL_PERIOD_MS         10U

/*
 * Защита от явно ошибочного показания ADS1115.
 */
#define ITV_SENSOR_MIN_VALID          0.35f
#define ITV_SENSOR_MAX_VALID          2.05f

#define CDC_PROTOCOL_VERSION          3U
#define CDC_RX_BUFFER_SIZE            256U
#define CDC_LINE_BUFFER_SIZE          64U
#define CDC_STREAM_PERIOD_DEFAULT_MS  50U
#define CDC_STREAM_PERIOD_MIN_MS      20U
#define CDC_STREAM_PERIOD_MAX_MS      1000U

/* =========================================================
 * АНАЛОГОВЫЕ СИГНАЛЫ
 * ========================================================= */

float ITV_PRESSURE = PRESSURE_SENSOR_ZERO_V;

float REG_ITV = 0.0f;

float REG_CAROUSEL = 0.0f;
float REG_CARRIAGE = 0.0f;

/* =========================================================
 * SPI
 * ========================================================= */

uint8_t rx_cmd;

volatile uint8_t spi_cmd_ready = 0;
volatile uint8_t spi_last_cmd = 0;

volatile HAL_StatusTypeDef spi_rx_status;
volatile uint32_t spi_error_code = 0;
volatile uint32_t spi_rx_count = 0;
volatile uint32_t spi_err_count = 0;

/* =========================================================
 * СОСТОЯНИЯ ПРОГРАММЫ
 * ========================================================= */

volatile uint8_t g_state = STATE_IDLE;
volatile uint8_t g_stage = STAGE_NOINIT;
volatile uint8_t g_CarriageDirection = 0;

/* =========================================================
 * ДАТЧИКИ
 * ========================================================= */

uint8_t HOMESENSOR = 0;
uint8_t CNTSENSOR = 0;
uint8_t NEARSENSOR = 0;
uint8_t FARSENSOR = 0;

/* =========================================================
 * УПРАВЛЕНИЕ
 * ========================================================= */

volatile float CARRIAGE_SPEED = 0.38f;

/*
 * Выходное напряжение управления ITV.
 * Передаётся на канал A AD5328.
 */
volatile float AIROUT = 0.0f;

/*
 * 0 — регулирование воздуха выключено;
 * 1 — регулирование воздуха включено.
 */
volatile uint8_t bigairon = 0;

/*
 * Время последнего шага регулирования.
 */
static uint32_t itv_control_tick = 0;
static float itv_integral_v = 0.0f;
static float itv_target_pressure_v = PRESSURE_SENSOR_ZERO_V;

/* =========================================================
 * USB CDC
 * ========================================================= */

static volatile uint8_t cdc_rx_buffer[CDC_RX_BUFFER_SIZE];
static volatile uint16_t cdc_rx_write_index = 0U;
static volatile uint16_t cdc_rx_read_index = 0U;
static uint8_t cdc_stream_enabled = 0U;
static uint32_t cdc_stream_period_ms = CDC_STREAM_PERIOD_DEFAULT_MS;
static uint32_t cdc_stream_tick = 0U;

/* =========================================================
 * СЧЁТЧИКИ
 * ========================================================= */

float coff = 0.0f;

volatile uint8_t counter = 0;
volatile uint8_t cntblock = 0;

volatile float meter = 0.0f;
volatile uint8_t metblock = 0;

volatile uint8_t isCollected = 0;

/* =========================================================
 * MAIN
 * ========================================================= */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_RTC_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_USB_DEVICE_Init();

    /* =====================================================
     * НАСТРОЙКА ЦАП AD5328
     * ===================================================== */

    dac.hspi = &hspi1;

    dac.sync_port = AD5328_SYNC_GPIO_Port;
    dac.sync_pin = AD5328_SYNC_Pin;

    dac.ldac_port = AD5328_LDAC_GPIO_Port;
    dac.ldac_pin = AD5328_LDAC_Pin;

    dac.vref = 5.0f;
    dac.gain = 2.0f;

    AD5328_Init(&dac);

    /* =====================================================
     * НАСТРОЙКА АЦП ADS1115
     * ===================================================== */

    adc.hi2c = &hi2c1;
    adc.address = ADS1115_DEFAULT_ADDR;

    adc.pga = ADS1115_PGA_6_144V;
    adc.data_rate = ADS1115_DR_475SPS;

    ADS1115_Init(&adc);

    /*
     * Начальное состояние выхода ITV.
     */
    AIROUT = 0.0f;

    /*
     * Запуск приёма команд по SPI2.
     */
    spi_rx_status = HAL_SPI_Receive_IT(
        &hspi2,
        &rx_cmd,
        1
    );

    while (1)
    {
        /* =================================================
         * ЧТЕНИЕ АНАЛОГОВЫХ ВХОДОВ
         * ================================================= */

        ADS1115_ReadVoltageSingleEnded(
            &adc,
            ADS1115_CHANNEL_0,
            &ITV_PRESSURE
        );

        ADS1115_ReadVoltageSingleEnded(
            &adc,
            ADS1115_CHANNEL_1,
            &REG_CAROUSEL
        );

        ADS1115_ReadVoltageSingleEnded(
            &adc,
            ADS1115_CHANNEL_2,
            &REG_CARRIAGE
        );

        ADS1115_ReadVoltageSingleEnded(
            &adc,
            ADS1115_CHANNEL_3,
            &REG_ITV
        );

        Motherboard_ProcessCDC();

        /* =================================================
         * РЕГУЛИРОВАНИЕ ДАВЛЕНИЯ
         * ================================================= */

        ITV_PressureControl();

        /* =================================================
         * ОБНОВЛЕНИЕ ЦАП
         * ================================================= */

        AD5328_WriteABCD_Voltage(
            &dac,
            AIROUT,
            REG_CAROUSEL * 2.0f,
            CARRIAGE_SPEED,
            0.8f
        );

        /* =================================================
         * ОБРАБОТКА SPI-КОМАНД
         * ================================================= */

        if (spi_cmd_ready)
        {
            spi_cmd_ready = 0;
            Motherboard_ProcessCommand(spi_last_cmd);
        }

        /* =================================================
         * ЧТЕНИЕ ДИСКРЕТНЫХ ДАТЧИКОВ
         * ================================================= */

        HOMESENSOR = HAL_GPIO_ReadPin(
            HMSENSOR_GPIO_Port,
            HMSENSOR_Pin
        );

        CNTSENSOR = HAL_GPIO_ReadPin(
            CNTSENSOR_GPIO_Port,
            CNTSENSOR_Pin
        );

        NEARSENSOR = HAL_GPIO_ReadPin(
            NEARSENSOR_GPIO_Port,
            NEARSENSOR_Pin
        );

        FARSENSOR = HAL_GPIO_ReadPin(
            FARSENSOR_GPIO_Port,
            FARSENSOR_Pin
        );

        /* =================================================
         * ПОДСЧЁТ ИМПУЛЬСОВ И МЕТРАЖА
         * ================================================= */

        if ((CNTSENSOR == 1) && (cntblock == 0))
        {
            counter++;
            meter += 0.24f;
            cntblock = 1;
        }
        else if ((CNTSENSOR == 0) && (cntblock == 1))
        {
            cntblock = 0;
        }

        if (counter >= 7)
        {
            coff += 0.09f;
            counter = 0;
        }

        /* =================================================
         * ЛОГИКА СОСТОЯНИЙ
         * ================================================= */

        if (g_state == STATE_IDLE)
        {
            carriage_stop();

            HAL_GPIO_WritePin(
                VF_FRW_CTRL_GPIO_Port,
                VF_FRW_CTRL_Pin,
                GPIO_PIN_RESET
            );
        }
        else if (g_state == STATE_RUNNING)
        {
            if (g_stage == STAGE_NOINIT)
            {
                if (NEARSENSOR == 0)
                {
                    carriage_move_rev();
                }
                else if ((NEARSENSOR == 1) &&
                         (HOMESENSOR == 0))
                {
                    carriage_stop();

                    HAL_GPIO_WritePin(
                        VF_FRW_CTRL_GPIO_Port,
                        VF_FRW_CTRL_Pin,
                        GPIO_PIN_SET
                    );
                }
                else if ((NEARSENSOR == 1) &&
                         (HOMESENSOR == 1))
                {
                    HAL_GPIO_WritePin(
                        VF_FRW_CTRL_GPIO_Port,
                        VF_FRW_CTRL_Pin,
                        GPIO_PIN_RESET
                    );

                    carriage_stop();

                    g_state = STATE_IDLE;
                    g_stage = STAGE_CYCLE;
                    meter = 0.0f;
                }
            }
            else if (g_stage == STAGE_CYCLE)
            {
                if (FARSENSOR == 0)
                {
                    /*
                     * Включаем регулирование воздуха.
                     */
                    bigairon = 1;

                    HAL_GPIO_WritePin(
                        SMALLVALVE_CTRL_GPIO_Port,
                        SMALLVALVE_CTRL_Pin,
                        GPIO_PIN_SET
                    );

                    /*
                     * Разрешаем движение после набора
                     * давления примерно до рабочего уровня.
                     */
                    if (ITV_PRESSURE >= ITV_PRESSURE_READY)
                    {
                        carriage_move_frw(REG_CARRIAGE);

                        HAL_GPIO_WritePin(
                            VF_FRW_CTRL_GPIO_Port,
                            VF_FRW_CTRL_Pin,
                            GPIO_PIN_SET
                        );
                    }
                    else
                    {
                        carriage_stop();

                        HAL_GPIO_WritePin(
                            VF_FRW_CTRL_GPIO_Port,
                            VF_FRW_CTRL_Pin,
                            GPIO_PIN_RESET
                        );
                    }

                    if ((meter >= 10.0f) &&
                        (isCollected == 0))
                    {
                        g_state = STATE_IDLE;
                        isCollected = 1;
                    }
                }
                else if (FARSENSOR == 1)
                {
                    carriage_stop();

                    HAL_GPIO_WritePin(
                        VF_FRW_CTRL_GPIO_Port,
                        VF_FRW_CTRL_Pin,
                        GPIO_PIN_RESET
                    );

                    g_state = STATE_IDLE;
                    g_stage = STAGE_REWARD;
                    isCollected = 0;
                }
            }
            else if (g_stage == STAGE_REWARD)
            {
                if (NEARSENSOR == 0)
                {
                    carriage_move_rev();
                }
                else if (NEARSENSOR == 1)
                {
                    carriage_stop();

                    g_state = STATE_IDLE;
                    g_stage = STAGE_SEEKHOME;
                }
            }
            else if (g_stage == STAGE_SEEKHOME)
            {
                if (HOMESENSOR == 0)
                {
                    HAL_GPIO_WritePin(
                        VF_FRW_CTRL_GPIO_Port,
                        VF_FRW_CTRL_Pin,
                        GPIO_PIN_SET
                    );
                }
                else if (HOMESENSOR == 1)
                {
                    HAL_GPIO_WritePin(
                        VF_FRW_CTRL_GPIO_Port,
                        VF_FRW_CTRL_Pin,
                        GPIO_PIN_RESET
                    );

                    g_state = STATE_IDLE;
                    g_stage = STAGE_CYCLE;
                    meter = 0.0f;
                }
            }
        }

        Motherboard_SendCDCTelemetry();
    }
}

/* =========================================================
 * РЕГУЛИРОВАНИЕ ITV
 * ========================================================= */

void ITV_PressureControl(void)
{
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms = current_tick - itv_control_tick;
    float reg_itv;
    float error_v;
    float proportional_v;
    float candidate_v;
    float dt_s;

    if (elapsed_ms < ITV_CONTROL_PERIOD_MS)
    {
        return;
    }

    itv_control_tick = current_tick;

    reg_itv = REG_ITV;
    if (reg_itv < 0.0f)
    {
        reg_itv = 0.0f;
    }
    else if (reg_itv > REG_ITV_INPUT_MAX_V)
    {
        reg_itv = REG_ITV_INPUT_MAX_V;
    }

    itv_target_pressure_v = PRESSURE_SENSOR_ZERO_V +
        (reg_itv / REG_ITV_INPUT_MAX_V) *
        (WORK_PRESSURE_MAX_BAR / PRESSURE_SENSOR_FULL_BAR) *
        PRESSURE_SENSOR_SPAN_V;

    if (bigairon == 0)
    {
        AIROUT = 0.0f;
        itv_integral_v = 0.0f;
        return;
    }

    if ((ITV_PRESSURE < ITV_SENSOR_MIN_VALID) ||
        (ITV_PRESSURE > ITV_SENSOR_MAX_VALID))
    {
        AIROUT = 0.0f;
        itv_integral_v = 0.0f;
        return;
    }

    error_v = itv_target_pressure_v - ITV_PRESSURE;
    if ((error_v > -ITV_CONTROL_DEADBAND_V) &&
        (error_v < ITV_CONTROL_DEADBAND_V))
    {
        error_v = 0.0f;
    }

    proportional_v = ITV_CONTROL_KP * error_v;
    candidate_v = reg_itv + proportional_v + itv_integral_v;
    dt_s = (float)elapsed_ms / 1000.0f;

    /* Не накапливаем интеграл дальше в сторону насыщения выхода. */
    if (((candidate_v < ITV_OUTPUT_MAX) &&
         (candidate_v > ITV_OUTPUT_MIN)) ||
        ((candidate_v >= ITV_OUTPUT_MAX) && (error_v < 0.0f)) ||
        ((candidate_v <= ITV_OUTPUT_MIN) && (error_v > 0.0f)))
    {
        itv_integral_v += ITV_CONTROL_KI * error_v * dt_s;
        if (itv_integral_v > ITV_INTEGRAL_LIMIT_V)
        {
            itv_integral_v = ITV_INTEGRAL_LIMIT_V;
        }
        else if (itv_integral_v < -ITV_INTEGRAL_LIMIT_V)
        {
            itv_integral_v = -ITV_INTEGRAL_LIMIT_V;
        }
    }

    AIROUT = reg_itv + proportional_v + itv_integral_v;
    if (AIROUT > ITV_OUTPUT_MAX)
    {
        AIROUT = ITV_OUTPUT_MAX;
    }
    else if (AIROUT < ITV_OUTPUT_MIN)
    {
        AIROUT = ITV_OUTPUT_MIN;
    }
}

/* =========================================================
 * USB CDC
 * ========================================================= */

void Motherboard_CDCReceive(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    for (index = 0U; index < length; index++)
    {
        uint16_t next_index =
            (uint16_t)((cdc_rx_write_index + 1U) % CDC_RX_BUFFER_SIZE);

        if (next_index == cdc_rx_read_index)
        {
            break;
        }

        cdc_rx_buffer[cdc_rx_write_index] = data[index];
        cdc_rx_write_index = next_index;
    }
}

static void Motherboard_ProcessCDC(void)
{
    static char line[CDC_LINE_BUFFER_SIZE];
    static uint16_t line_length = 0U;

    while (cdc_rx_read_index != cdc_rx_write_index)
    {
        uint8_t value = cdc_rx_buffer[cdc_rx_read_index];
        cdc_rx_read_index =
            (uint16_t)((cdc_rx_read_index + 1U) % CDC_RX_BUFFER_SIZE);

        if (value == '\r')
        {
            continue;
        }

        if (value == '\n')
        {
            line[line_length] = '\0';
            if (line_length > 0U)
            {
                Motherboard_HandleCDCLine(line);
            }
            line_length = 0U;
        }
        else if (line_length < (CDC_LINE_BUFFER_SIZE - 1U))
        {
            line[line_length++] = (char)value;
        }
        else
        {
            line_length = 0U;
        }
    }
}

static void Motherboard_HandleCDCLine(char *line)
{
    static const uint8_t protocol_message[] = "PROTO,3\n";
    unsigned long requested_period;

    if (strcmp(line, "VERSION") == 0)
    {
        CDC_Transmit_FS((uint8_t *)protocol_message,
                        (uint16_t)(sizeof(protocol_message) - 1U));
    }
    else if (sscanf(line, "STREAM,START,%lu", &requested_period) == 1)
    {
        if (requested_period < CDC_STREAM_PERIOD_MIN_MS)
        {
            requested_period = CDC_STREAM_PERIOD_MIN_MS;
        }
        else if (requested_period > CDC_STREAM_PERIOD_MAX_MS)
        {
            requested_period = CDC_STREAM_PERIOD_MAX_MS;
        }

        cdc_stream_period_ms = (uint32_t)requested_period;
        cdc_stream_enabled = 1U;
        cdc_stream_tick = 0U;
        CDC_Transmit_FS((uint8_t *)protocol_message,
                        (uint16_t)(sizeof(protocol_message) - 1U));
    }
    else if (strcmp(line, "STREAM,STOP") == 0)
    {
        cdc_stream_enabled = 0U;
    }
    else if (strcmp(line, "START") == 0)
    {
        Motherboard_ProcessCommand(CMD_START);
    }
    else if (strcmp(line, "STOP") == 0)
    {
        Motherboard_ProcessCommand(CMD_STOP);
    }
    else if (strcmp(line, "RESET") == 0)
    {
        Motherboard_ProcessCommand(CMD_RESET);
    }
    else if (strcmp(line, "LEFT") == 0)
    {
        Motherboard_ProcessCommand(CMD_LEFT);
    }
    else if (strcmp(line, "AIR") == 0)
    {
        Motherboard_ProcessCommand(CMD_AIR);
    }
    else if (strcmp(line, "AIR,1") == 0)
    {
        if (bigairon == 0U)
        {
            Motherboard_ProcessCommand(CMD_AIR);
        }
    }
    else if (strcmp(line, "AIR,0") == 0)
    {
        if (bigairon != 0U)
        {
            Motherboard_ProcessCommand(CMD_AIR);
        }
    }
}

static void Motherboard_SendCDCTelemetry(void)
{
    static uint8_t message[192];
    uint32_t current_tick;
    uint8_t pressure_valid;
    uint8_t sensors;
    int length;

    if (cdc_stream_enabled == 0U)
    {
        return;
    }

    current_tick = HAL_GetTick();
    if ((current_tick - cdc_stream_tick) < cdc_stream_period_ms)
    {
        return;
    }
    cdc_stream_tick = current_tick;

    pressure_valid =
        ((ITV_PRESSURE >= ITV_SENSOR_MIN_VALID) &&
         (ITV_PRESSURE <= ITV_SENSOR_MAX_VALID)) ? 1U : 0U;
    sensors =
        (HOMESENSOR ? 0x01U : 0U) |
        (CNTSENSOR ? 0x02U : 0U) |
        (NEARSENSOR ? 0x04U : 0U) |
        (FARSENSOR ? 0x08U : 0U);

    length = snprintf(
        (char *)message,
        sizeof(message),
        "TEL,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%lu,%u,%u,%u,%u\n",
        (unsigned long)Motherboard_VoltageToMillivolts(ITV_PRESSURE),
        (unsigned long)Motherboard_VoltageToMillivolts(itv_target_pressure_v),
        (unsigned long)Motherboard_VoltageToMillivolts(REG_ITV),
        (unsigned long)Motherboard_VoltageToMillivolts(AIROUT * 2.0f),
        (unsigned int)bigairon,
        (unsigned int)pressure_valid,
        (unsigned int)g_state,
        (unsigned int)g_stage,
        (unsigned long)((meter > 0.0f) ? (meter * 1000.0f + 0.5f) : 0.0f),
        (unsigned int)sensors,
        (unsigned int)(HAL_GPIO_ReadPin(
            SRV_ON_CTRL_GPIO_Port, SRV_ON_CTRL_Pin) ? 1U : 0U),
        (unsigned int)(HAL_GPIO_ReadPin(
            VF_FRW_CTRL_GPIO_Port, VF_FRW_CTRL_Pin) ? 1U : 0U),
        (unsigned int)(HAL_GPIO_ReadPin(
            VF_REV_CTRL_GPIO_Port, VF_REV_CTRL_Pin) ? 1U : 0U));

    if ((length > 0) && ((uint32_t)length < sizeof(message)))
    {
        CDC_Transmit_FS(message, (uint16_t)length);
    }
}

static uint32_t Motherboard_VoltageToMillivolts(float voltage)
{
    if (voltage <= 0.0f)
    {
        return 0U;
    }

    return (uint32_t)(voltage * 1000.0f + 0.5f);
}

/* =========================================================
 * SPI CALLBACK
 * ========================================================= */

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        spi_rx_count++;

        spi_last_cmd = rx_cmd;
        spi_cmd_ready = 1;

        spi_rx_status = HAL_SPI_Receive_IT(
            &hspi2,
            &rx_cmd,
            1
        );
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        spi_err_count++;
        spi_error_code = HAL_SPI_GetError(hspi);

        __HAL_SPI_CLEAR_OVRFLAG(hspi);

        HAL_SPI_Abort_IT(hspi);

        spi_rx_status = HAL_SPI_Receive_IT(
            &hspi2,
            &rx_cmd,
            1
        );
    }
}

/* =========================================================
 * ОБРАБОТКА КОМАНД
 * ========================================================= */

void Motherboard_ProcessCommand(uint8_t cmd)
{
    switch (cmd)
    {
        case CMD_START:
            g_state = STATE_RUNNING;
            break;

        case CMD_STOP:
            g_state = STATE_IDLE;
            break;

        case CMD_AIR:
            if (bigairon == 0)
            {
                bigairon = 1;

                HAL_GPIO_WritePin(
                    SMALLVALVE_CTRL_GPIO_Port,
                    SMALLVALVE_CTRL_Pin,
                    GPIO_PIN_SET
                );
            }
            else
            {
                bigairon = 0;

                /*
                 * Немедленно выключаем выход ITV.
                 */
                AIROUT = 0.0f;

                HAL_GPIO_WritePin(
                    SMALLVALVE_CTRL_GPIO_Port,
                    SMALLVALVE_CTRL_Pin,
                    GPIO_PIN_RESET
                );
            }
            break;

        case CMD_RESET:
            g_stage = STAGE_NOINIT;
            meter = 0.0f;
            isCollected = 0;
            break;

        case CMD_LEFT:
            if ((HAL_GPIO_ReadPin(
                    VF_FRW_CTRL_GPIO_Port,
                    VF_FRW_CTRL_Pin) == 0) &&
                (HAL_GPIO_ReadPin(
                    VF_REV_CTRL_GPIO_Port,
                    VF_REV_CTRL_Pin) == 0))
            {
                HAL_GPIO_WritePin(
                    VF_REV_CTRL_GPIO_Port,
                    VF_REV_CTRL_Pin,
                    GPIO_PIN_SET
                );
            }
            else if (HAL_GPIO_ReadPin(
                         VF_REV_CTRL_GPIO_Port,
                         VF_REV_CTRL_Pin) == 1)
            {
                HAL_GPIO_WritePin(
                    VF_REV_CTRL_GPIO_Port,
                    VF_REV_CTRL_Pin,
                    GPIO_PIN_RESET
                );
            }
            break;

        case CMD_RIGHT:
            break;

        case CMD_UP:
            g_CarriageDirection = 1;
            break;

        case CMD_DOWN:
            g_CarriageDirection = 0;
            break;

        default:
            HAL_GPIO_WritePin(
                VF_REV_CTRL_GPIO_Port,
                VF_REV_CTRL_Pin,
                GPIO_PIN_RESET
            );
            break;
    }
}

/* =========================================================
 * СИСТЕМНАЯ ЧАСТОТА
 * ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_LSI |
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue =
        RCC_HSE_PREDIV_DIV1;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLMUL =
        RCC_PLL_MUL6;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_RTC |
        RCC_PERIPHCLK_USB;

    PeriphClkInit.RTCClockSelection =
        RCC_RTCCLKSOURCE_LSI;

    PeriphClkInit.UsbClockSelection =
        RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(
            &PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
 * I2C1
 * ========================================================= */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed = 10000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;

    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c1.Init.OwnAddress2 = 0;

    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
 * RTC
 * ========================================================= */

static void MX_RTC_Init(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef DateToUpdate = {0};

    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;

    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }

    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;

    if (HAL_RTC_SetTime(
            &hrtc,
            &sTime,
            RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }

    DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
    DateToUpdate.Month = RTC_MONTH_JANUARY;
    DateToUpdate.Date = 0x1;
    DateToUpdate.Year = 0x0;

    if (HAL_RTC_SetDate(
            &hrtc,
            &DateToUpdate,
            RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
 * SPI1 — ЦАП
 * ========================================================= */

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;

    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_1LINE;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;

    hspi1.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_128;

    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;

    hspi1.Init.CRCCalculation =
        SPI_CRCCALCULATION_DISABLE;

    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
 * SPI2 — КОМАНДЫ
 * ========================================================= */

static void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;

    hspi2.Init.Mode = SPI_MODE_SLAVE;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;

    hspi2.Init.CRCCalculation =
        SPI_CRCCALCULATION_DISABLE;

    hspi2.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
 * GPIO
 * ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(
        GPIOA,
        AD5328_SYNC_Pin |
        AD5328_LDAC_Pin |
        SMALLVALVE_CTRL_Pin |
        BIGVALVE_CTRL_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOB,
        VF_FRW_CTRL_Pin |
        VF_REV_CTRL_Pin |
        SRV_ON_CTRL_Pin |
        SRV_PULS_CTRL_Pin |
        SRV_SIGN_CTRL_Pin |
        CNT_ECHO_Pin,
        GPIO_PIN_RESET
    );

    GPIO_InitStruct.Pin =
        HMSENSOR_Pin |
        CNTSENSOR_Pin |
        NEARSENSOR_Pin |
        FARSENSOR_Pin |
        USB_HPD_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        AD5328_SYNC_Pin |
        AD5328_LDAC_Pin |
        SMALLVALVE_CTRL_Pin |
        BIGVALVE_CTRL_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        VF_FRW_CTRL_Pin |
        VF_REV_CTRL_Pin |
        SRV_ON_CTRL_Pin |
        SRV_PULS_CTRL_Pin |
        SRV_SIGN_CTRL_Pin |
        CNT_ECHO_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* =========================================================
 * УПРАВЛЕНИЕ КАРЕТКОЙ
 * ========================================================= */

void carriage_move_frw(float speed)
{
    CARRIAGE_SPEED = 1.5f + speed;

    if (CARRIAGE_SPEED > 5.0f)
    {
        CARRIAGE_SPEED = 5.0f;
    }

    HAL_GPIO_WritePin(
        SRV_ON_CTRL_GPIO_Port,
        SRV_ON_CTRL_Pin,
        GPIO_PIN_SET
    );
}

void carriage_move_rev(void)
{
    CARRIAGE_SPEED = 0.0f;

    HAL_GPIO_WritePin(
        SRV_ON_CTRL_GPIO_Port,
        SRV_ON_CTRL_Pin,
        GPIO_PIN_SET
    );
}

void carriage_stop(void)
{
    CARRIAGE_SPEED = 1.5f;

    HAL_GPIO_WritePin(
        SRV_ON_CTRL_GPIO_Port,
        SRV_ON_CTRL_Pin,
        GPIO_PIN_RESET
    );
}

/* =========================================================
 * ERROR HANDLER
 * ========================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    /*
     * Пользовательская обработка ошибки assert.
     */
}

#endif
