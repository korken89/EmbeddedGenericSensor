#ifndef __SENSORS_H
#define __SENSORS_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define SRD_MAILBOX_SIZE        10
#define SRD_THREAD_PRIORITY     HIGHPRIO
#define SRD_THREAD_STACKSIZE    256
#define SRD_DEBUG               TRUE

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Sensor functions, parameters and priorities configuration.
 */
typedef struct
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
} _sensor_t;

/**
 * @brief   Polled sensor configuration.
 */
typedef struct
{
    /**
     * @brief   The sensor read rate.
     * @note    The calculation av the time between reads is calculated by a
     *          division with system tick: CH_CFG_ST_FREQUENCY / frequency_hz
     *          This always rounds down if it is not a perfect division.
     *          Ex, 1000 Hz tick and 90 Hz sensor rate will round to 90.9 Hz
     */
    uint32_t frequency_hz;
    /**
     * @brief   Pointer to an accumulation variable.
     * @details This is used if the sampling rate over time is important to
     *          the sensor. The accumulator will keep track of the delta times
     *          between executions and add extra delay to keep the average
     *          frequency to be the same as frequency_hz.
     */
    uint32_t *accumulator;
    /**
     * @brief   Sensor configuration.
     */
    _sensor_t sensor;
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
    _sensor_t sensor;
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
    /**
     * @brief   Number of polled sensors.
     */
    mailbox_t srd_mailbox;
    /**
     * @brief   Number of polled sensors.
     */
    msg_t messages[SRD_MAILBOX_SIZE];
#if SRD_DEBUG
    /**
     * @brief   Keeps the debug status of the mailbox in case of
     *          to many requests.
     */
    bool dbg_mailbox_overflow;
#endif
} SensorReadDriver;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

#define COUNT_OF(array)         (sizeof(array)/sizeof(*(array)))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern SensorReadDriver SRD1;

msg_t SensorsInit(SensorReadDriver *srdp,
                  const interrupt_sensor_t *intsenp,
                  const polled_sensor_t *pollsenp,
                  size_t intsencnt,
                  size_t pollsencnt);
msg_t SensorsStart(SensorReadDriver *srdp);
msg_t SensorsStop(SensorReadDriver *srdp);
void sensors_interrupt_callback(EXTDriver *extp, expchannel_t channel);
void sensors_polled_callback(void *param);

/*===========================================================================*/
/* Module exported inline functions.                                                  */
/*===========================================================================*/

#endif /* __SENSORS_H */
