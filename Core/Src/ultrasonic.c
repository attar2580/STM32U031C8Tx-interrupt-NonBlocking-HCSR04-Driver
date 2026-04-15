/**
 * @file       ultrasonic.c
 * @author     Fardeen
 * @brief      Implementation of the 3-sensor ultrasonic measurement module.
 * @details    Provides trigger generation, GPIO edge processing, timeout
 *             handling, and distance conversion based on a microsecond timer.
 *             The module supports both EXTI-driven capture and blocking
 *             polling capture paths.
 * @version    1.0.0
 * @date       2026-04-15
 *
 * @copyright  Copyright (c) Fardeen. All rights reserved.
 */

#include "ultrasonic.h"


#define ULTRASONIC_TIMEOUT_US                 (65000U)  /* bounded by 16-bit timer window */
#define ULTRASONIC_DISTANCE_PER_TICK_MM       (0.1715F) /* 1 MHz timer nominal */
#define ULTRASONIC_SPEED_OF_SOUND_MM_PER_US   (0.343F)
#define ULTRASONIC_TRIGGER_WIDTH_US           (50U)
#define ULTRASONIC_TRIGGER_SETTLE_US          (5U)
#define ULTRASONIC_PRETRIGGER_LOW_TIMEOUT_US  (10000U)

volatile uint16_t ultrasonic_start_ticks[ULTRASONIC_SENSOR_COUNT];
volatile float ultrasonic_distance_mm[ULTRASONIC_SENSOR_COUNT];
volatile uint8_t ultrasonic_data_ready[ULTRASONIC_SENSOR_COUNT];
/* Shared ISR/application state: required as module-level volatile storage. */
static volatile uint8_t ultrasonic_rise_seen[ULTRASONIC_SENSOR_COUNT];

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
static volatile uint32_t ultrasonic_diag_rise_count[ULTRASONIC_SENSOR_COUNT];
static volatile uint32_t ultrasonic_diag_fall_count[ULTRASONIC_SENSOR_COUNT];
static volatile uint32_t ultrasonic_diag_invalid_fall_count[ULTRASONIC_SENSOR_COUNT];
static volatile uint32_t ultrasonic_diag_timeout_count[ULTRASONIC_SENSOR_COUNT];
#endif

static float ultrasonic_mm_per_tick = ULTRASONIC_DISTANCE_PER_TICK_MM;

static GPIO_TypeDef *const trigger_ports[ULTRASONIC_SENSOR_COUNT] = {
    TRIG1_PORT,
    TRIG2_PORT,
    TRIG3_PORT
};

static const uint16_t trigger_pins[ULTRASONIC_SENSOR_COUNT] = {
    TRIG1_PIN,
    TRIG2_PIN,
    TRIG3_PIN
};

static GPIO_TypeDef *const echo_ports[ULTRASONIC_SENSOR_COUNT] = {
    ECHO1_PORT,
    ECHO2_PORT,
    ECHO3_PORT
};

static const uint16_t echo_pins[ULTRASONIC_SENSOR_COUNT] = {
    ECHO1_PIN,
    ECHO2_PIN,
    ECHO3_PIN
};

static void Delay_Microseconds(uint16_t us);
static uint32_t sensor_to_index(Ultrasonic_Sensor_e sensorID);
static void configure_sensor_state(void);
static uint32_t resolve_echo_index(uint16_t gpio_pin, GPIO_TypeDef **gpio_port);
static void reset_capture_state(uint32_t index);

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief  Busy-wait delay based on TIM3 microsecond counter.
 * @param  us Delay duration in microseconds.
 * @retval None
 */
static void Delay_Microseconds(uint16_t us)
{
    /* Record the current tick without modifying the timer */
    const uint16_t start_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    
    /* Wait until the difference exceeds the requested delay */
    while (((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) - start_ticks) < us)
    {
        /* Busy-wait for the trigger width */
    }
}

/**
 * @brief  Converts a sensor enum to zero-based array index.
 * @param  sensorID Sensor identifier.
 * @retval Zero-based index or ULTRASONIC_SENSOR_COUNT if invalid.
 */
static uint32_t sensor_to_index(Ultrasonic_Sensor_e sensorID)
{
    if ((sensorID < ULTRASONIC_SENSOR_1) || (sensorID > ULTRASONIC_SENSOR_3))
    {
        return ULTRASONIC_SENSOR_COUNT;
    }
    return ULTRASONIC_SENSOR_INDEX(sensorID);
}

/**
 * @brief  Initializes runtime state for all sensors.
 * @retval None
 */
static void configure_sensor_state(void)
{
    for (uint32_t index = 0U; index < ULTRASONIC_SENSOR_COUNT; index++)
    {
        ultrasonic_start_ticks[index] = 0U;
        ultrasonic_distance_mm[index] = ULTRASONIC_ERROR_DISTANCE_MM;
        ultrasonic_data_ready[index] = 0U;
        ultrasonic_rise_seen[index] = 0U;

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
        ultrasonic_diag_rise_count[index] = 0U;
        ultrasonic_diag_fall_count[index] = 0U;
        ultrasonic_diag_invalid_fall_count[index] = 0U;
        ultrasonic_diag_timeout_count[index] = 0U;
#endif
    }
}

/**
 * @brief  Resets capture state for one sensor index.
 * @param  index Zero-based sensor index.
 * @retval None
 */
static void reset_capture_state(uint32_t index)
{
    if (index < ULTRASONIC_SENSOR_COUNT)
    {
        ultrasonic_start_ticks[index] = 0U;
        ultrasonic_distance_mm[index] = ULTRASONIC_ERROR_DISTANCE_MM;
        ultrasonic_data_ready[index] = 0U;
        ultrasonic_rise_seen[index] = 0U;
    }
}

/**
 * @brief  Resolves EXTI GPIO pin to sensor index and GPIO port.
 * @param  gpio_pin EXTI pin mask.
 * @param  gpio_port Output pointer for resolved GPIO port.
 * @retval Sensor index or ULTRASONIC_SENSOR_COUNT if unresolved.
 */
static uint32_t resolve_echo_index(uint16_t gpio_pin, GPIO_TypeDef **gpio_port)
{
    uint32_t index = ULTRASONIC_SENSOR_COUNT;
    switch (gpio_pin)
    {
        case ECHO1_PIN:
            index = ULTRASONIC_SENSOR_INDEX(ULTRASONIC_SENSOR_1);
            break;
        case ECHO2_PIN:
            index = ULTRASONIC_SENSOR_INDEX(ULTRASONIC_SENSOR_2);
            break;
        case ECHO3_PIN:
            index = ULTRASONIC_SENSOR_INDEX(ULTRASONIC_SENSOR_3);
            break;
        default:
            break;
    }

    if ((index < ULTRASONIC_SENSOR_COUNT) && (gpio_port != NULL))
    {
        *gpio_port = echo_ports[index];
    }

    return index;
}

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief   Initializes ultrasonic timing and internal module state.
 *
 * @details Starts TIM3 base counting for microsecond timing and initializes
 *          all per-sensor capture, distance, and diagnostic state.
 *
 * @pre     TIM3 must be configured before this call.
 * @post    Module state is reset and ready for measurements.
 */
void Ultrasonic_Init(void)
{
    (void)HAL_TIM_Base_Start(&htim3); /* Start failure is handled by system error policy. */
    configure_sensor_state();

    SystemCoreClockUpdate();
    const uint32_t prescaler = (uint32_t)(htim3.Instance->PSC) + 1U;
    float timer_frequency = (float)SystemCoreClock / (float)prescaler;
    if (timer_frequency <= 0.0F)
    {
        timer_frequency = 1.0F;
    }

    const float tick_duration_us = 1000000.0F / timer_frequency;
    ultrasonic_mm_per_tick = tick_duration_us * ULTRASONIC_SPEED_OF_SOUND_MM_PER_US * 0.5F;
}

/**
 * @brief   Issues a trigger pulse for one ultrasonic sensor.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @post    Selected sensor capture state is reset and a trigger pulse is sent.
 */
void Ultrasonic_TriggerSensor(Ultrasonic_Sensor_e sensorID)
{
    const uint32_t index = sensor_to_index(sensorID);
    if (index >= ULTRASONIC_SENSOR_COUNT)
    {
        return;
    }

    reset_capture_state(index);
    GPIO_TypeDef *const port = trigger_ports[index];
    const uint16_t pin = trigger_pins[index];

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    Delay_Microseconds(ULTRASONIC_TRIGGER_SETTLE_US);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    Delay_Microseconds(ULTRASONIC_TRIGGER_WIDTH_US);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/**
 * @brief   Returns the last stored distance for a sensor.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @return  float
 * @retval  Last measured distance in millimeters.
 * @retval  ULTRASONIC_ERROR_DISTANCE_MM for an invalid sensor identifier.
 */
float Ultrasonic_GetLastDistance(Ultrasonic_Sensor_e sensorID)
{
    const uint32_t index = sensor_to_index(sensorID);
    if (index >= ULTRASONIC_SENSOR_COUNT)
    {
        return ULTRASONIC_ERROR_DISTANCE_MM;
    }
    return ultrasonic_distance_mm[index];
}

/**
 * @brief   Reports whether new distance data is available for a sensor.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @return  uint8_t
 * @retval  1U if data is ready.
 * @retval  0U if data is not ready or sensor identifier is invalid.
 */
uint8_t Ultrasonic_IsDataReady(Ultrasonic_Sensor_e sensorID)
{
    const uint32_t index = sensor_to_index(sensorID);
    if (index >= ULTRASONIC_SENSOR_COUNT)
    {
        return 0U;
    }
    return ultrasonic_data_ready[index];
}

/**
 * @brief   Clears the data-ready flag for a sensor.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @post    Data-ready flag is cleared when sensor identifier is valid.
 */
void Ultrasonic_ClearDataReady(Ultrasonic_Sensor_e sensorID)
{
    const uint32_t index = sensor_to_index(sensorID);
    if (index < ULTRASONIC_SENSOR_COUNT)
    {
        ultrasonic_data_ready[index] = 0U;
    }
}

/**
 * @brief   Records a timeout event and resets active capture state.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @post    Timeout counter may increment and sensor state is marked invalid.
 */
void Ultrasonic_NotifyTimeout(Ultrasonic_Sensor_e sensorID)
{
    const uint32_t index = sensor_to_index(sensorID);
    if (index < ULTRASONIC_SENSOR_COUNT)
    {
#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
        ultrasonic_diag_timeout_count[index]++;
#endif
        ultrasonic_rise_seen[index] = 0U;
        ultrasonic_data_ready[index] = 0U;
        ultrasonic_distance_mm[index] = ULTRASONIC_ERROR_DISTANCE_MM;
    }
}

/**
 * @brief   Reads diagnostic counters for a sensor.
 *
 * @param[in]  sensorID           Sensor identifier.
 * @param[out] rise_count         Rising-edge count output pointer.
 * @param[out] fall_count         Valid falling-edge count output pointer.
 * @param[out] invalid_fall_count Invalid falling-edge count output pointer.
 * @param[out] timeout_count      Timeout count output pointer.
 *
 * @post    Output pointers contain current diagnostic counter values.
 */
void Ultrasonic_GetDiagnostics(Ultrasonic_Sensor_e sensorID,
                               uint32_t *rise_count,
                               uint32_t *fall_count,
                               uint32_t *invalid_fall_count,
                               uint32_t *timeout_count)
{
    const uint32_t index = sensor_to_index(sensorID);
    if ((index >= ULTRASONIC_SENSOR_COUNT) ||
        (rise_count == NULL) ||
        (fall_count == NULL) ||
        (invalid_fall_count == NULL) ||
        (timeout_count == NULL))
    {
        return;
    }

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
    *rise_count = ultrasonic_diag_rise_count[index];
    *fall_count = ultrasonic_diag_fall_count[index];
    *invalid_fall_count = ultrasonic_diag_invalid_fall_count[index];
    *timeout_count = ultrasonic_diag_timeout_count[index];
#else
    *rise_count = 0U;
    *fall_count = 0U;
    *invalid_fall_count = 0U;
    *timeout_count = 0U;
#endif
}

/**
 * @brief   Performs blocking distance measurement by polling echo pin edges.
 *
 * @param[in] sensorID   Sensor identifier.
 * @param[in] timeout_us Timeout in microseconds for rising and falling waits.
 *
 * @return  float
 * @retval  Measured distance in millimeters on success.
 * @retval  ULTRASONIC_ERROR_DISTANCE_MM on timeout or invalid sensor ID.
 *
 * @warning This function blocks until edge completion or timeout.
 */
float Ultrasonic_MeasureBlocking(Ultrasonic_Sensor_e sensorID, uint32_t timeout_us)
{
    const uint32_t index = sensor_to_index(sensorID);
    uint32_t effective_timeout_us = timeout_us;

    if (index >= ULTRASONIC_SENSOR_COUNT)
    {
        return ULTRASONIC_ERROR_DISTANCE_MM;
    }

    if (effective_timeout_us == 0U)
    {
        return ULTRASONIC_ERROR_DISTANCE_MM;
    }

    if (effective_timeout_us > 65535U)
    {
        effective_timeout_us = 65535U;
    }

    {
        const GPIO_PinState pin_set_state = GPIO_PIN_SET;
        const GPIO_PinState pin_reset_state = GPIO_PIN_RESET;
        GPIO_TypeDef *const echo_port = echo_ports[index];
        const uint16_t echo_pin = echo_pins[index];
        uint16_t wait_start_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

        while (HAL_GPIO_ReadPin(echo_port, echo_pin) == pin_set_state)
        {
            const uint16_t now_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            const uint32_t elapsed_ticks = (uint32_t)((uint16_t)(now_ticks - wait_start_ticks));
            if (elapsed_ticks >= ULTRASONIC_PRETRIGGER_LOW_TIMEOUT_US)
            {
                Ultrasonic_NotifyTimeout(sensorID);
                return ULTRASONIC_ERROR_DISTANCE_MM;
            }
        }

        Ultrasonic_TriggerSensor(sensorID);
        wait_start_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        while (HAL_GPIO_ReadPin(echo_port, echo_pin) == pin_reset_state)
        {
            const uint16_t now_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            const uint32_t elapsed_ticks = (uint32_t)((uint16_t)(now_ticks - wait_start_ticks));
            if (elapsed_ticks >= effective_timeout_us)
            {
                Ultrasonic_NotifyTimeout(sensorID);
                return ULTRASONIC_ERROR_DISTANCE_MM;
            }
        }

        ultrasonic_start_ticks[index] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        ultrasonic_rise_seen[index] = 1U;

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
        ultrasonic_diag_rise_count[index]++;
#endif

        wait_start_ticks = ultrasonic_start_ticks[index];
        while (HAL_GPIO_ReadPin(echo_port, echo_pin) == pin_set_state)
        {
            const uint16_t now_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            const uint32_t elapsed_ticks = (uint32_t)((uint16_t)(now_ticks - wait_start_ticks));
            if (elapsed_ticks >= effective_timeout_us)
            {
                Ultrasonic_NotifyTimeout(sensorID);
                return ULTRASONIC_ERROR_DISTANCE_MM;
            }
        }

        {
            const uint16_t end_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            const uint16_t delta = (uint16_t)(end_ticks - ultrasonic_start_ticks[index]);

            if (delta <= ULTRASONIC_TIMEOUT_US)
            {
                ultrasonic_distance_mm[index] = ((float)delta) * ultrasonic_mm_per_tick;
            }
            else
            {
                ultrasonic_distance_mm[index] = ULTRASONIC_ERROR_DISTANCE_MM;

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
                ultrasonic_diag_timeout_count[index]++;
#endif
            }

            ultrasonic_data_ready[index] = 1U;
            ultrasonic_rise_seen[index] = 0U;

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
            ultrasonic_diag_fall_count[index]++;
#endif
        }
    }

    return ultrasonic_distance_mm[index];
}

/**
 * @brief  HAL EXTI callback for echo edge capture path.
 * @param  GPIO_Pin Triggered EXTI GPIO pin mask.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO_TypeDef *gpio_port = NULL;
    const uint32_t index = resolve_echo_index(GPIO_Pin, &gpio_port);
    if (index >= ULTRASONIC_SENSOR_COUNT)
    {
        return;
    }

    const uint16_t pin_mask = echo_pins[index];
    if ((gpio_port->IDR & pin_mask) != 0U)
    {
        ultrasonic_start_ticks[index] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
        ultrasonic_diag_rise_count[index]++;
#endif
        ultrasonic_rise_seen[index] = 1U;
    }
    else if ((ultrasonic_data_ready[index] == 0U) && (ultrasonic_rise_seen[index] != 0U))
    {
        const uint16_t end_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        const uint16_t start_ticks = ultrasonic_start_ticks[index];
        const uint16_t delta = (uint16_t)(end_ticks - start_ticks);
        if (delta <= ULTRASONIC_TIMEOUT_US)
        {
            ultrasonic_distance_mm[index] = ((float)delta) * ultrasonic_mm_per_tick;
        }
        else
        {
            ultrasonic_distance_mm[index] = ULTRASONIC_ERROR_DISTANCE_MM;
#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
            ultrasonic_diag_timeout_count[index]++;
#endif
        }

        ultrasonic_data_ready[index] = 1U;
        ultrasonic_rise_seen[index] = 0U;

#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
        ultrasonic_diag_fall_count[index]++;
#endif
    }
    else
    {
#if (ULTRASONIC_DIAGNOSTICS_ENABLE == 1U)
        ultrasonic_diag_invalid_fall_count[index]++;
#endif
    }
}
