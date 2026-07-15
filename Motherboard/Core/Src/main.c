#include "main.h"
#include "usb_device.h"
#include <string.h>

#include "ADS1115.h"
#include "AD5328.h"
#include "mixer_cdc.h"

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
static void Motherboard_ProcessCDCCommands(void);
static void Motherboard_SetAir(uint8_t enabled);
static void Motherboard_SafeAirOff(void);
static void Motherboard_SafeStop(void);
static void Motherboard_CDCSafetyTick(void);
static void Motherboard_CDCBurstDetection(void);
static void Motherboard_SendCDCTelemetry(void);
static void Motherboard_ResetBurstDetector(void);
static void Motherboard_UpdatePressureTarget(void);
static void Motherboard_UpdatePressureReady(void);

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
    STATE_STOPPED = 2,
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

#define REG_ITV_INPUT_MAX_V           5.0f
#define WORK_PRESSURE_MAX_BAR         2.0f
#define PRESSURE_SENSOR_FULL_BAR      10.0f
#define PRESSURE_SENSOR_SPAN_V        1.60f
#define PRESSURE_READY_TOLERANCE_V    0.008f
#define PRESSURE_READY_RELEASE_V      0.016f
#define PRESSURE_READY_STABLE_MS      300U
#define ITV_CONTROL_DEADBAND_V        0.004f

/*
 * Ограничения управляющего выхода ЦАП.
 */
#define ITV_OUTPUT_MIN                0.0f
#define ITV_OUTPUT_MAX                10.0f

/*
 * Период регулирования.
 */
#define ITV_CONTROL_PERIOD_MS         20U

/*
 * Защита от явно ошибочного показания ADS1115.
 */
#define ITV_SENSOR_MIN_VALID          0.35f
#define ITV_SENSOR_MAX_VALID          2.05f

/* 4...20 мА на шунте 100 Ом: 0.40...2.00 В. */
#define ITV_SENSOR_ZERO_V             0.40f

/* Защитные параметры удалённого испытания оболочки. */
#define CDC_CONTROL_TIMEOUT_MS        750U
#define CDC_STREAM_PERIOD_MIN_MS      20U
#define CDC_STREAM_PERIOD_MAX_MS      1000U
#define CDC_BURST_HISTORY_SIZE        32U
#define CDC_BURST_MIN_RISE_V          0.06f
#define CDC_BURST_CONFIRM_SAMPLES     2U

/* =========================================================
 * АНАЛОГОВЫЕ СИГНАЛЫ
 * ========================================================= */

float ITV_PRESSURE = ITV_SENSOR_ZERO_V;

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
static float itv_target_pressure_v = ITV_SENSOR_ZERO_V;
static float itv_control_trim_v = 0.0f;
static uint8_t pressure_ready = 0U;
static uint32_t pressure_ready_since_tick = 0U;

/* =========================================================
 * USB CDC / ИСПЫТАНИЕ ОБОЛОЧКИ
 * ========================================================= */

static uint8_t cdc_remote_airout = 0;
static uint8_t cdc_stream_enabled = 0;
static uint8_t pressure_sensor_valid = 0;
static uint32_t cdc_last_control_tick = 0;
static uint32_t cdc_last_stream_tick = 0;
static uint32_t cdc_stream_period_ms = 20U;
static float cdc_burst_drop_threshold_v = 0.08f;
static float cdc_pressure_history[CDC_BURST_HISTORY_SIZE];
static uint8_t cdc_pressure_history_count = 0;
static uint8_t cdc_pressure_history_index = 0;
static uint8_t cdc_burst_confirm_count = 0;

/* =========================================================
 * СЧЁТЧИКИ
 * ========================================================= */

float coff = 0.0f;

volatile uint8_t counter = 0;
volatile uint8_t cntblock = 0;

volatile float meter = 0.0f;
volatile uint8_t metblock = 0;

volatile uint8_t isCollected = 0;
static uint8_t meter_pause_enabled = 1U;
static float meter_pause_length_m = 10.0f;
static uint8_t meter_pause_done = 0U;
static uint8_t cycle_active = 0U;
static uint32_t cycle_count = 0U;
static uint32_t product_count = 0U;

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
    /* Внешний ОУ после AD5328 усиливает 0...5 В в 0...10 В. */
    dac.output_gain = 2.0f;

    if (AD5328_Init(&dac) != HAL_OK)
    {
        Error_Handler();
    }

    /* =====================================================
     * НАСТРОЙКА АЦП ADS1115
     * ===================================================== */

    adc.hi2c = &hi2c1;
    adc.address = ADS1115_DEFAULT_ADDR;

    adc.pga = ADS1115_PGA_6_144V;
    adc.data_rate = ADS1115_DR_475SPS;

    pressure_sensor_valid = (ADS1115_Init(&adc) == HAL_OK) ? 1U : 0U;

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

        HAL_StatusTypeDef pressure_status = ADS1115_ReadVoltageSingleEnded(
            &adc,
            ADS1115_CHANNEL_0,
            &ITV_PRESSURE
        );

        pressure_sensor_valid =
            ((pressure_status == HAL_OK) &&
             (ITV_PRESSURE >= ITV_SENSOR_MIN_VALID) &&
             (ITV_PRESSURE <= ITV_SENSOR_MAX_VALID)) ? 1U : 0U;

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

        Motherboard_UpdatePressureTarget();

        /* =================================================
         * ОБРАБОТКА USB CDC И ЗАЩИТА ИСПЫТАНИЯ
         * ================================================= */

        MixerCDC_Process();
        Motherboard_ProcessCDCCommands();
        Motherboard_CDCSafetyTick();
        Motherboard_CDCBurstDetection();

        /* =================================================
         * РЕГУЛИРОВАНИЕ ДАВЛЕНИЯ
         * ================================================= */

        ITV_PressureControl();
        Motherboard_UpdatePressureReady();

        /* =================================================
         * ОБНОВЛЕНИЕ ЦАП
         * ================================================= */

        if (AD5328_WriteABCD_Voltage(
                &dac,
                AIROUT,
                REG_CAROUSEL * 2.0f,
                CARRIAGE_SPEED,
                0.8f) != HAL_OK)
        {
            Motherboard_SafeAirOff();
        }

        Motherboard_SendCDCTelemetry();

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

        if ((g_state == STATE_IDLE) || (g_state == STATE_STOPPED))
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
                    if (bigairon == 0U)
                    {
                        cdc_remote_airout = 0U;
                        Motherboard_SetAir(1U);
                    }

                    /*
                     * Разрешаем движение после набора
                     * давления примерно до рабочего уровня.
                     */
                    if (pressure_ready != 0U)
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

                    if ((meter_pause_enabled != 0U) &&
                        (meter_pause_done == 0U) &&
                        (meter >= meter_pause_length_m))
                    {
                        g_state = STATE_IDLE;
                        isCollected = 1;
                        meter_pause_done = 1U;
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
                    meter_pause_done = 0U;
                    cycle_active = 0U;
                    cycle_count++;
                    product_count++;
                }
            }
        }
    }
}

/* =========================================================
 * РЕГУЛИРОВАНИЕ ITV
 * ========================================================= */

void ITV_PressureControl(void)
{
    uint32_t current_tick = HAL_GetTick();
    float feed_forward_v;
    float error_v;
    float step_v;

    /* Во время испытания оболочки AIROUT задаётся программой напрямую. */
    if (cdc_remote_airout != 0U)
    {
        return;
    }

    /*
     * Запускаем регулирование раз в 20 мс.
     */
    if ((current_tick - itv_control_tick) <
        ITV_CONTROL_PERIOD_MS)
    {
        return;
    }

    itv_control_tick = current_tick;

    /*
     * При выключенном воздухе сбрасываем
     * выход ЦАП в ноль.
     */
    if (bigairon == 0)
    {
        AIROUT = 0.0f;
        itv_control_trim_v = 0.0f;
        return;
    }

    /*
     * Защита от явно некорректного показания датчика.
     */
    if (pressure_sensor_valid == 0U)
    {
        Motherboard_SafeStop();
        g_state = STATE_ERROR;
        return;
    }

    /*
     * REG_ITV 0...5 В соответствует настроенному диапазону ITV 0...2 бар.
     * Поэтому базовое задание на вход ITV3010/3050 равно REG_ITV * 2.
     * Обратная связь по FPSX добавляет небольшую интегральную коррекцию.
     */
    feed_forward_v = REG_ITV * (ITV_OUTPUT_MAX / REG_ITV_INPUT_MAX_V);
    if (feed_forward_v < ITV_OUTPUT_MIN)
    {
        feed_forward_v = ITV_OUTPUT_MIN;
    }
    else if (feed_forward_v > ITV_OUTPUT_MAX)
    {
        feed_forward_v = ITV_OUTPUT_MAX;
    }

    error_v = itv_target_pressure_v - ITV_PRESSURE;
    if (error_v > ITV_CONTROL_DEADBAND_V)
    {
        step_v = (error_v > 0.08f) ? 0.05f :
                 (error_v > 0.03f) ? 0.02f : 0.005f;
        itv_control_trim_v += step_v;
    }
    else if (error_v < -ITV_CONTROL_DEADBAND_V)
    {
        float excess_v = -error_v;
        step_v = (excess_v > 0.08f) ? 0.05f :
                 (excess_v > 0.03f) ? 0.02f : 0.005f;
        itv_control_trim_v -= step_v;
    }

    AIROUT = feed_forward_v + itv_control_trim_v;
    if (AIROUT > ITV_OUTPUT_MAX)
    {
        AIROUT = ITV_OUTPUT_MAX;
        itv_control_trim_v = AIROUT - feed_forward_v;
    }
    else if (AIROUT < ITV_OUTPUT_MIN)
    {
        AIROUT = ITV_OUTPUT_MIN;
        itv_control_trim_v = AIROUT - feed_forward_v;
    }
}

static void Motherboard_UpdatePressureTarget(void)
{
    float reg_itv = REG_ITV;

    if (reg_itv < 0.0f)
    {
        reg_itv = 0.0f;
    }
    else if (reg_itv > REG_ITV_INPUT_MAX_V)
    {
        reg_itv = REG_ITV_INPUT_MAX_V;
    }

    itv_target_pressure_v = ITV_SENSOR_ZERO_V +
        (reg_itv / REG_ITV_INPUT_MAX_V) *
        (WORK_PRESSURE_MAX_BAR / PRESSURE_SENSOR_FULL_BAR) *
        PRESSURE_SENSOR_SPAN_V;
}

static void Motherboard_UpdatePressureReady(void)
{
    uint32_t current_tick = HAL_GetTick();

    if ((bigairon == 0U) || (pressure_sensor_valid == 0U))
    {
        pressure_ready = 0U;
        pressure_ready_since_tick = 0U;
        return;
    }

    if ((ITV_PRESSURE + PRESSURE_READY_TOLERANCE_V) >= itv_target_pressure_v)
    {
        if (pressure_ready_since_tick == 0U)
        {
            pressure_ready_since_tick = current_tick;
        }
        else if ((current_tick - pressure_ready_since_tick) >=
                 PRESSURE_READY_STABLE_MS)
        {
            pressure_ready = 1U;
        }
    }
    else if ((ITV_PRESSURE + PRESSURE_READY_RELEASE_V) < itv_target_pressure_v)
    {
        pressure_ready = 0U;
        pressure_ready_since_tick = 0U;
    }
}

/* =========================================================
 * USB CDC / ИСПЫТАНИЕ ОБОЛОЧКИ
 * ========================================================= */

static void Motherboard_ProcessCDCCommands(void)
{
    MixerCDC_Command command;

    while (MixerCDC_PopCommand(&command) != 0U)
    {
        switch (command.type)
        {
            case MIXER_CDC_CMD_START:
                Motherboard_ProcessCommand(CMD_START);
                break;

            case MIXER_CDC_CMD_STOP:
                Motherboard_ProcessCommand(CMD_STOP);
                break;

            case MIXER_CDC_CMD_RESET:
                Motherboard_ProcessCommand(CMD_RESET);
                break;

            case MIXER_CDC_CMD_LEFT:
                Motherboard_ProcessCommand(CMD_LEFT);
                break;

            case MIXER_CDC_CMD_AIR:
                if (command.value >= 0.5f)
                {
                    if (pressure_sensor_valid != 0U)
                    {
                        Motherboard_SetAir(1U);
                        cdc_last_control_tick = HAL_GetTick();
                    }
                    else
                    {
                        Motherboard_SafeAirOff();
                    }
                }
                else
                {
                    Motherboard_SafeAirOff();
                }
                break;

            case MIXER_CDC_CMD_PAUSE_ENABLE:
                meter_pause_enabled = (command.value >= 0.5f) ? 1U : 0U;
                break;

            case MIXER_CDC_CMD_PAUSE_LENGTH:
                if ((command.value >= 0.1f) && (command.value <= 1000.0f))
                {
                    meter_pause_length_m = command.value;
                }
                break;

            case MIXER_CDC_CMD_COUNTERS_RESET:
                if (g_state != STATE_RUNNING)
                {
                    cycle_count = 0U;
                    product_count = 0U;
                }
                break;

            case MIXER_CDC_CMD_AIROUT:
                if (command.value < ITV_OUTPUT_MIN)
                {
                    command.value = ITV_OUTPUT_MIN;
                }
                else if (command.value > ITV_OUTPUT_MAX)
                {
                    command.value = ITV_OUTPUT_MAX;
                }

                AIROUT = command.value;
                cdc_remote_airout = 1U;
                cdc_last_control_tick = HAL_GetTick();
                break;

            case MIXER_CDC_CMD_STREAM_START:
                if (command.period_ms < CDC_STREAM_PERIOD_MIN_MS)
                {
                    command.period_ms = CDC_STREAM_PERIOD_MIN_MS;
                }
                else if (command.period_ms > CDC_STREAM_PERIOD_MAX_MS)
                {
                    command.period_ms = CDC_STREAM_PERIOD_MAX_MS;
                }

                cdc_stream_period_ms = command.period_ms;
                cdc_stream_enabled = 1U;
                cdc_last_stream_tick = 0U;
                break;

            case MIXER_CDC_CMD_STREAM_STOP:
                cdc_stream_enabled = 0U;
                break;

            case MIXER_CDC_CMD_DROP_THRESHOLD:
                if ((command.value >= 0.02f) && (command.value <= 0.50f))
                {
                    cdc_burst_drop_threshold_v = command.value;
                }
                break;

            case MIXER_CDC_CMD_PING:
                cdc_last_control_tick = HAL_GetTick();
                break;

            default:
                break;
        }
    }
}

static void Motherboard_SetAir(uint8_t enabled)
{
    if (enabled != 0U)
    {
        if (pressure_sensor_valid == 0U)
        {
            Motherboard_SafeAirOff();
            return;
        }

        bigairon = 1U;
        itv_control_trim_v = 0.0f;
        AIROUT = REG_ITV * (ITV_OUTPUT_MAX / REG_ITV_INPUT_MAX_V);
        if (AIROUT > ITV_OUTPUT_MAX)
        {
            AIROUT = ITV_OUTPUT_MAX;
        }
        HAL_GPIO_WritePin(
            SMALLVALVE_CTRL_GPIO_Port,
            SMALLVALVE_CTRL_Pin,
            GPIO_PIN_SET
        );
        Motherboard_ResetBurstDetector();
    }
    else
    {
        Motherboard_SafeAirOff();
    }
}

static void Motherboard_SafeAirOff(void)
{
    AIROUT = 0.0f;
    bigairon = 0U;
    cdc_remote_airout = 0U;
    pressure_ready = 0U;
    pressure_ready_since_tick = 0U;
    itv_control_trim_v = 0.0f;

    HAL_GPIO_WritePin(
        SMALLVALVE_CTRL_GPIO_Port,
        SMALLVALVE_CTRL_Pin,
        GPIO_PIN_RESET
    );
    HAL_GPIO_WritePin(
        BIGVALVE_CTRL_GPIO_Port,
        BIGVALVE_CTRL_Pin,
        GPIO_PIN_RESET
    );
}

static void Motherboard_SafeStop(void)
{
    Motherboard_SafeAirOff();
    carriage_stop();

    HAL_GPIO_WritePin(
        VF_FRW_CTRL_GPIO_Port,
        VF_FRW_CTRL_Pin,
        GPIO_PIN_RESET
    );
    HAL_GPIO_WritePin(
        VF_REV_CTRL_GPIO_Port,
        VF_REV_CTRL_Pin,
        GPIO_PIN_RESET
    );

    g_state = STATE_STOPPED;
}

static void Motherboard_CDCSafetyTick(void)
{
    if ((cdc_remote_airout == 0U) || (bigairon == 0U))
    {
        return;
    }

    if ((pressure_sensor_valid == 0U) ||
        ((HAL_GetTick() - cdc_last_control_tick) > CDC_CONTROL_TIMEOUT_MS))
    {
        Motherboard_SafeAirOff();
    }
}

static void Motherboard_CDCBurstDetection(void)
{
    float recent_peak = ITV_PRESSURE;
    uint8_t index;

    if ((cdc_remote_airout == 0U) ||
        (bigairon == 0U) ||
        (pressure_sensor_valid == 0U))
    {
        return;
    }

    for (index = 0U; index < cdc_pressure_history_count; index++)
    {
        if (cdc_pressure_history[index] > recent_peak)
        {
            recent_peak = cdc_pressure_history[index];
        }
    }

    cdc_pressure_history[cdc_pressure_history_index] = ITV_PRESSURE;
    cdc_pressure_history_index =
        (uint8_t)((cdc_pressure_history_index + 1U) % CDC_BURST_HISTORY_SIZE);

    if (cdc_pressure_history_count < CDC_BURST_HISTORY_SIZE)
    {
        cdc_pressure_history_count++;
    }

    if ((recent_peak >= (ITV_SENSOR_ZERO_V + CDC_BURST_MIN_RISE_V)) &&
        ((recent_peak - ITV_PRESSURE) >= cdc_burst_drop_threshold_v))
    {
        cdc_burst_confirm_count++;
    }
    else
    {
        cdc_burst_confirm_count = 0U;
    }

    if (cdc_burst_confirm_count >= CDC_BURST_CONFIRM_SAMPLES)
    {
        Motherboard_SafeAirOff();
    }
}

static void Motherboard_SendCDCTelemetry(void)
{
    uint32_t current_tick;
    MixerCDC_Telemetry telemetry;

    if (cdc_stream_enabled == 0U)
    {
        return;
    }

    current_tick = HAL_GetTick();
    if ((current_tick - cdc_last_stream_tick) < cdc_stream_period_ms)
    {
        return;
    }

    cdc_last_stream_tick = current_tick;
    telemetry.pressure_v = ITV_PRESSURE;
    telemetry.pressure_target_v = itv_target_pressure_v;
    telemetry.reg_itv_v = REG_ITV;
    telemetry.reg_carousel_v = REG_CAROUSEL;
    telemetry.reg_carriage_v = REG_CARRIAGE;
    telemetry.airout_v = AIROUT;
    telemetry.carriage_speed_v = CARRIAGE_SPEED;
    telemetry.meter_m = meter;
    telemetry.pause_length_m = meter_pause_length_m;
    telemetry.cycle_count = cycle_count;
    telemetry.product_count = product_count;
    telemetry.bigairon = bigairon;
    telemetry.pressure_valid = pressure_sensor_valid;
    telemetry.pressure_ready = pressure_ready;
    telemetry.state = g_state;
    telemetry.stage = g_stage;
    telemetry.pause_enabled = meter_pause_enabled;
    telemetry.sensors =
        (HOMESENSOR ? 0x01U : 0U) |
        (CNTSENSOR ? 0x02U : 0U) |
        (NEARSENSOR ? 0x04U : 0U) |
        (FARSENSOR ? 0x08U : 0U);
    telemetry.carriage_on = HAL_GPIO_ReadPin(
        SRV_ON_CTRL_GPIO_Port, SRV_ON_CTRL_Pin) ? 1U : 0U;
    telemetry.vf_forward = HAL_GPIO_ReadPin(
        VF_FRW_CTRL_GPIO_Port, VF_FRW_CTRL_Pin) ? 1U : 0U;
    telemetry.vf_reverse = HAL_GPIO_ReadPin(
        VF_REV_CTRL_GPIO_Port, VF_REV_CTRL_Pin) ? 1U : 0U;
    telemetry.cycle_active = cycle_active;

    MixerCDC_SendTelemetry(&telemetry);
}

static void Motherboard_ResetBurstDetector(void)
{
    uint8_t index;

    cdc_pressure_history_count = 0U;
    cdc_pressure_history_index = 0U;
    cdc_burst_confirm_count = 0U;

    for (index = 0U; index < CDC_BURST_HISTORY_SIZE; index++)
    {
        cdc_pressure_history[index] = ITV_SENSOR_ZERO_V;
    }
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
            if (g_state != STATE_ERROR)
            {
                if ((g_stage == STAGE_CYCLE) && (cycle_active == 0U))
                {
                    cycle_active = 1U;
                    meter_pause_done = 0U;
                }

                g_state = STATE_RUNNING;
            }
            break;

        case CMD_STOP:
            Motherboard_SafeStop();
            break;

        case CMD_AIR:
            if (bigairon == 0)
            {
                cdc_remote_airout = 0U;
                Motherboard_SetAir(1U);
            }
            else
            {
                Motherboard_SafeAirOff();
            }
            break;

        case CMD_RESET:
            /*
             * RESET is a safe reinitialisation of the scenario, not an MCU
             * reset: first remove every actuator command, then return the
             * state machine to the homing stage. Production counters survive.
             */
            Motherboard_SafeStop();
            g_stage = STAGE_NOINIT;
            meter = 0.0f;
            isCollected = 0;
            meter_pause_done = 0U;
            cycle_active = 0U;
            g_state = STATE_IDLE;
            break;

        case CMD_LEFT:
            if ((g_state != STATE_RUNNING) &&
                (HAL_GPIO_ReadPin(
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
        case CMD_UP:
        case CMD_DOWN:
            /* Reserved physical keys; intentionally inactive for now. */
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

    hi2c1.Init.ClockSpeed = 100000;
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
