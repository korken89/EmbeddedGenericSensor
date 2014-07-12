#ifndef __SENSORS_H
#define __SENSORS_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Sensor functions, parameters and priorities configuration.
 */
struct _sensor
{
    /**
     * @brief   Pointer to the initialization function of the sensor.
     * @note    Is not allowed be NULL.
     */
    msg_t (* init_sensor)(void *);
    /**
     * @brief   Pointer to the read function of the sensor.
     * @note    Is not allowed be NULL.
     */
    msg_t (* read_sensor)(void *);
    /**
     * @brief   Pointer to the parameter to be sent with the init and read
     *          calls.
     */
    void *params;
    /**
     * @brief   Flag for if the sensor is a priority sensor. This allows the
     *          sensor reads to be put first in the read queue.
     */
    bool priority_sensor;
};

/**
 * @brief   Polled sensor configuration.
 */
typedef struct
{
    /**
     * @brief   The number of Hertz the sensor will be read at.
     */
    uint32_t frequency_hz;
    /**
     * @brief   Pointer to an accumulation variable.
     * @details This is used if the sampling rate over time is important to
     *          the senor. The accumulator will keep track of the delta times
     *          between executions and add extra delay to keep the average
     *          frequency to be the same as frequency_hz.
     */
    uint32_t *accumulated_ticks;
    /**
     * @brief   Sensor configuration.
     */
    struct _sensor sensor;
    /**
     * @brief   Pointer to the virtual timer that keeps track of the read
     *          execution of the sensor.
     */
    virtual_timer_t *polling_vt;
} polled_sensor_t;

/**
 * @brief   Interrupt sensor configuration.
 */
typedef struct
{
    /**
     * @brief   Which channel the sensor's interrupt is connected to.
     */
    expchannel_t interrupt_channel;
    /**
     * @brief   Sensor configuration.
     */
    struct _sensor sensor;
} interrupt_sensor_t;

/**
 * @brief   Sensor read driver's possible states.
 */
typedef enum 
{
    /**
     * @brief   Uninitialized sate.
     */
    SRD_UNINITIALIZED = 0,
    /**
     * @brief   Stopped state.
     */
    SRD_STOPPED = 1,
    /**
     * @brief   Started state.
     */
    SRD_STARTED = 2
} sensor_read_state_t;

/**
 * @brief   Sensor read driver object.
 */
typedef struct
{
    /**
     * @brief   Current state of the driver.
     */
    sensor_read_state_t state;
    /**
     * @brief   Pointer to the list of interrupt sensors.
     */
    const interrupt_sensor_t *interrupt_sensor_ptr;
    /**
     * @brief   Pointer to the list of polled sensors.
     */
    const polled_sensor_t *polled_sensor_ptr;
    /**
     * @brief   Number of interrupt sensors.
     */
    size_t interrupt_sensor_cnt;
    /**
     * @brief   Number of polled sensors.
     */
    size_t polled_sensor_cnt;
    /**
     * @brief   Lookup table for the interrupt sensor to convert 
     *          interrupt channel -> interrupt_sensor_ptr array index.
     */
    int8_t expchannel_lookup[EXT_MAX_CHANNELS];
} SensorReadDriver;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

#define COUNT_OF(array)         (sizeof(array)/sizeof(*(array)))

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

msg_t SensorsInit(SensorReadDriver *srdp,
                  const interrupt_sensor_t *intsenp,
                  const polled_sensor_t *pollsenp,
                  size_t intsencnt,
                  size_t pollsencnt);
msg_t SensorsStart(SensorReadDriver *srdp);
msg_t SensorsStop(SensorReadDriver *srdp);
void sensors_interrupt_callback(EXTDriver *extp, expchannel_t channel);
void sensors_polled_callback(void *param);

#endif /* __SENSORS_H */
