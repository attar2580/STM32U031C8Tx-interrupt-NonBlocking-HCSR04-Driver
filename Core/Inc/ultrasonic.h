/**
 * @file       ultrasonic.h
 * @author     Fardeen
 * @brief      Public interface for the 3-sensor ultrasonic measurement module.
 * @details    Declares sensor identifiers, fixed configuration macros, shared
 *             measurement state, and the API used by application code to
 *             trigger and read ultrasonic distance measurements.
 * @version    1.0.0
 * @date       2026-04-15
 *
 * @copyright  Copyright (c) Fardeen. All rights reserved.
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "stm32u0xx_hal.h"

/* -----------------------------------------------------------------
 * HARDWARE TIMER REFERENCE
 * Ensure this timer is configured in STM32CubeMX to tick at 1 MHz.
 * (1 tick = 1 microsecond). 
 * ----------------------------------------------------------------- */
extern TIM_HandleTypeDef htim3; 

/* -----------------------------------------------------------------
 * SENSOR CONFIGURATION MACROS
 * ----------------------------------------------------------------- */
/* Sensor 1 */
#define TRIG1_PORT  GPIOA
#define TRIG1_PIN   GPIO_PIN_10   /* D2 */
#define ECHO1_PORT  GPIOB
#define ECHO1_PIN   GPIO_PIN_3    /* D3 */

/* Sensor 2 */
#define TRIG2_PORT  GPIOB
#define TRIG2_PIN   GPIO_PIN_4    /* D5 */
#define ECHO2_PORT  GPIOB
#define ECHO2_PIN   GPIO_PIN_5    /* D4 */

/* Sensor 3 */
#define TRIG3_PORT  GPIOA
#define TRIG3_PIN   GPIO_PIN_8    /* D7 */
#define ECHO3_PORT  GPIOB
#define ECHO3_PIN   GPIO_PIN_10   /* D6 */

/**
 * @brief  Failsafe return value when an invalid measurement occurred.
 */
#define ULTRASONIC_ERROR_DISTANCE_MM (4000.0F)

#define ULTRASONIC_SENSOR_COUNT      (3U)
#define ULTRASONIC_SENSOR_INDEX(id)  ((uint32_t)((id) - 1U))
#define ULTRASONIC_DIAGNOSTICS_ENABLE (1U)

/* -----------------------------------------------------------------
 * TYPE DEFINITIONS
 * ----------------------------------------------------------------- */
/**
 * @brief Strongly typed enumeration for identifying ultrasonic sensors
 */
typedef enum {
    ULTRASONIC_SENSOR_1 = 1U,
    ULTRASONIC_SENSOR_2 = 2U,
    ULTRASONIC_SENSOR_3 = 3U
} Ultrasonic_Sensor_e;

extern volatile uint16_t ultrasonic_start_ticks[ULTRASONIC_SENSOR_COUNT];
extern volatile float ultrasonic_distance_mm[ULTRASONIC_SENSOR_COUNT];
extern volatile uint8_t ultrasonic_data_ready[ULTRASONIC_SENSOR_COUNT];

/* -----------------------------------------------------------------
 * FUNCTION PROTOTYPES
 * ----------------------------------------------------------------- */
/**
 * @brief   Initializes the ultrasonic module runtime state and timing base.
 *
 * @details Starts the configured timer base used for microsecond-level timing,
 *          then initializes internal measurement state for all sensors.
 *
 * @pre     TIM3 must be configured by MX_TIM3_Init() before this call.
 * @post    Internal state is reset and ready for measurement calls.
 */
void Ultrasonic_Init(void);

/**
 * @brief   Sends a trigger pulse to a selected ultrasonic sensor.
 *
 * @details Clears sensor capture state, drives the trigger pin high for the
 *          configured pulse width, then drives it low.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @pre     Ultrasonic_Init() has been called.
 * @post    Selected sensor has a fresh measurement window.
 */
void Ultrasonic_TriggerSensor(Ultrasonic_Sensor_e sensorID);

/**
 * @brief   Gets the last measured distance for a selected sensor.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @return  float
 * @retval  Last stored distance in millimeters.
 * @retval  ULTRASONIC_ERROR_DISTANCE_MM on invalid sensor identifier.
 */
float Ultrasonic_GetLastDistance(Ultrasonic_Sensor_e sensorID);

/**
 * @brief   Checks whether new distance data is available.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @return  uint8_t
 * @retval  1U if new data is ready.
 * @retval  0U if data is not ready or identifier is invalid.
 */
uint8_t Ultrasonic_IsDataReady(Ultrasonic_Sensor_e sensorID);

/**
 * @brief   Clears the data-ready flag for a selected sensor.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @pre     sensorID must identify a valid sensor.
 * @post    Data-ready flag is cleared for the selected sensor.
 */
void Ultrasonic_ClearDataReady(Ultrasonic_Sensor_e sensorID);

/**
 * @brief   Performs a blocking echo measurement using polling.
 *
 * @details Triggers the selected sensor, waits for the echo rising and falling
 *          edges by polling GPIO state, computes pulse width using TIM3, and
 *          converts the result to millimeters.
 *
 * @param[in] sensorID   Sensor identifier.
 * @param[in] timeout_us Maximum wait time in microseconds for each edge.
 *
 * @return  float
 * @retval  Measured distance in millimeters on success.
 * @retval  ULTRASONIC_ERROR_DISTANCE_MM on timeout or invalid identifier.
 *
 * @pre     Ultrasonic_Init() has been called.
 * @post    Internal measurement state is updated for the selected sensor.
 *
 * @warning This function is blocking; do not call from ISR context.
 */
float Ultrasonic_MeasureBlocking(Ultrasonic_Sensor_e sensorID, uint32_t timeout_us);

/**
 * @brief   Records a timeout condition for diagnostics and state reset.
 *
 * @param[in] sensorID Sensor identifier.
 *
 * @post    Timeout counter may increment and selected sensor state is reset.
 */
void Ultrasonic_NotifyTimeout(Ultrasonic_Sensor_e sensorID);

/**
 * @brief   Reads per-sensor diagnostic counters.
 *
 * @param[in]  sensorID           Sensor identifier.
 * @param[out] rise_count         Rising-edge count output pointer.
 * @param[out] fall_count         Valid falling-edge count output pointer.
 * @param[out] invalid_fall_count Invalid falling-edge count output pointer.
 * @param[out] timeout_count      Timeout count output pointer.
 *
 * @pre     Output pointers must not be NULL.
 * @post    Output locations contain the current counter snapshot.
 */
void Ultrasonic_GetDiagnostics(Ultrasonic_Sensor_e sensorID,
                               uint32_t *rise_count,
                               uint32_t *fall_count,
                               uint32_t *invalid_fall_count,
                               uint32_t *timeout_count);

#endif /* ULTRASONIC_H_ */
