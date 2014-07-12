/* *
 *
 * Abstraction Layer for Sensor Initialization and Reading.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "sensors.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define SRD_EXT_DRIVER              EXTD1

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

SensorReadDriver SRD1;

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

THD_WORKING_AREA(waThreadSRD1, SRD_THREAD_STACKSIZE);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

__attribute__((noreturn))
static THD_FUNCTION(ThreadSRD1, arg)
{
    SensorReadDriver *srdp = (SensorReadDriver *)arg;
    msg_t message;
    _sensor_t *senp;
#if SRD_DEBUG
    chRegSetThreadName("ThreadSRD1");
#endif

    while (1)
    {
        /* Get the latest sensor read request */
        chMBFetch(&srdp->srd_mailbox, &message, TIME_INFINITE);

        /* Convert the message to a sensor pointer */
        senp = (_sensor_t *)message;

        /* Call its read function */
        senp->read_sensor(senp->params);
    }
}

/*===========================================================================*/
/* Module local inline functions.                                                  */
/*===========================================================================*/

static inline const interrupt_sensor_t *extchannel2interrupt_sensor(SensorReadDriver *srdp,
                                                                    expchannel_t ch)
{
    int8_t id = srdp->expchannel_lookup[ch];

    if (id != -1)
        return &srdp->interrupt_sensor_ptr[id];
    else
        return NULL;
}

static inline msg_t queueReadI(SensorReadDriver *srdp, const _sensor_t *senp)
{
    if (senp->priority_sensor == true)
        /* Priority sensor, put it at the front of the queue */
        return chMBPostAheadI(&srdp->srd_mailbox, (msg_t)senp);
    else
        /* Not a priority sensor, put it at the end of the queue */
       return chMBPostI(&srdp->srd_mailbox, (msg_t)senp);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/
msg_t SensorsInit(SensorReadDriver *srdp,
                  const interrupt_sensor_t *intsenp,
                  const polled_sensor_t *pollsenp,
                  size_t intsencnt,
                  size_t pollsencnt)
{
    chDbgCheck(srdp != NULL);
    chDbgAssert((intsenp == NULL) && (intsencnt != 0),
                "Pointer can't be NULL when the count is > 0");
    chDbgAssert((pollsenp == NULL) && (pollsencnt != 0),
                "Pointer can't be NULL when the count is > 0");
    
    size_t i;
    msg_t retval;

    /* Initialize lookup table */
    for (i = 0; i < EXT_MAX_CHANNELS; i++)
        srdp->expchannel_lookup[i] = -1;

    /* Generate interrupt channel -> array index table */
    for (i = 0; i < intsencnt; i++)
        srdp->expchannel_lookup[intsenp[i].interrupt_channel] = i;

    /* Copy sensor pointers and sizes to driver structures */
    srdp->interrupt_sensor_ptr = intsenp;
    srdp->interrupt_sensor_cnt = intsencnt;
    srdp->polled_sensor_ptr = pollsenp;
    srdp->polled_sensor_cnt = pollsencnt;

    /* Run interrupt sensor initialization */
    for (i = 0; i < intsencnt; i++)
    {
        /* Run sensor initialization with requested params */
        if (intsenp[i].sensor.init_sensor != NULL)
            retval = intsenp[i].sensor.init_sensor(intsenp[i].sensor.params);
        else
            retval = MSG_RESET;

        if (retval != MSG_OK)
            return retval;
    }

    /* Run polled sensor initialization */
    for (i = 0; i < pollsencnt; i++)
    {
        /* Check so the requested rate is plausible */
        if (CH_CFG_ST_FREQUENCY / pollsenp[i].frequency_hz == 0)
            return MSG_TIMEOUT;

        /* Zero the accumulated ticks */
        if (pollsenp[i].accumulator != NULL)
            *(pollsenp[i].accumulator) = 0;

        /* Run sensor initialization with requested params */
        if (pollsenp[i].sensor.init_sensor != NULL)
            retval = pollsenp[i].sensor.init_sensor(pollsenp[i].sensor.params);
        else
            retval = MSG_RESET;

        if (retval != MSG_OK)
            return retval;
        else /* Initialization is OK, initialize the virtual timer */
            chVTObjectInit(pollsenp[i].polling_vt);
    }

    /* Initialize the read mailbox */
    chMBObjectInit(&srdp->srd_mailbox, srdp->messages, SRD_MAILBOX_SIZE);
#if SRD_DEBUG
    SRD1.dbg_mailbox_overflow = false;
#endif

    /* Everything OK, transverse the state and start the read thread */
    srdp->state = SRD_STOPPED;

    chThdCreateStatic(waThreadSRD1,
                      sizeof(waThreadSRD1), 
                      SRD_THREAD_PRIORITY, 
                      ThreadSRD1, 
                      srdp);

    return MSG_OK;
}

msg_t SensorsStart(SensorReadDriver *srdp)
{
    size_t i;

    if (srdp->state == SRD_STOPPED)
    {
        /* Enable interrupts for interrupt driven sensors */
        for (i = 0; i < srdp->interrupt_sensor_cnt; i++)
            extChannelEnable(&SRD_EXT_DRIVER,
                             srdp->interrupt_sensor_ptr[i].interrupt_channel);

        /* Enable timers for polled driven sensors */
        for (i = 0; i < srdp->polled_sensor_cnt; i++)
            /* The parameter sent to the VT is the pointer to the corresponding
               polled_sensor_t structure to be able to access it from the
               callback function */
            chVTSet(srdp->polled_sensor_ptr[i].polling_vt,
                CH_CFG_ST_FREQUENCY / srdp->polled_sensor_ptr[i].frequency_hz,
                sensors_polled_callback,
                (void *)&srdp->polled_sensor_ptr[i]);

        /* Everything OK, transverse the state */
        srdp->state = SRD_STARTED;

        return MSG_OK;    
    }
    else
        return MSG_RESET;
}

msg_t SensorsStop(SensorReadDriver *srdp)
{
    size_t i;

    if (srdp->state == SRD_STARTED)
    {
        /* Disable interrupts for interrupt driven sensors */
        for (i = 0; i < srdp->interrupt_sensor_cnt; i++)
            extChannelDisable(&SRD_EXT_DRIVER,
                              srdp->interrupt_sensor_ptr[i].interrupt_channel);

        /* Reset timers for polled driven sensors */
        for (i = 0; i < srdp->polled_sensor_cnt; i++)
            chVTReset(srdp->polled_sensor_ptr[i].polling_vt);

        /* Reset the mailbox */
        chMBReset(&srdp->srd_mailbox);

        /* Everything OK, transverse the state */
        srdp->state = SRD_STOPPED;

        return MSG_OK;    
    }
    else
        return MSG_RESET;
}

void sensors_interrupt_callback(EXTDriver *extp, expchannel_t channel)
{
    const interrupt_sensor_t *p;
    osalSysLockFromISR();

    /* Check if the driver is in the correct state,
       else disable the interrupt */
    if (SRD1.state == SRD_STARTED)
    {
        p = extchannel2interrupt_sensor(&SRD1, channel);

        if (p != NULL)
        {
            /* Tell the reading thread to read the sensor requested sensor */
            if (queueReadI(&SRD1, &p->sensor) != MSG_OK)
            {
#if SRD_DEBUG
                SRD1.dbg_mailbox_overflow = true;
#endif
            }
        }
        else
            /* Something in the initial settings structures are faulty */
            extChannelDisableI(extp, channel);    
    }
    else
        extChannelDisableI(extp, channel);

    osalSysUnlockFromISR();
}

/**
 * @brief               Virtual timer callback function for sensor reads.
 * 
 * @param[in] param     Input parameter pointing to the corresponding
 *                      polled_sensor_t configuration.
 */
void sensors_polled_callback(void *param)
{
    systime_t wait_time;
    uint32_t frequency_hz;

    /* Convert the parameter to the correct type */
    const polled_sensor_t *p = (const polled_sensor_t *)param;

    /* Check if the driver is in the correct state and that the polled sensor
       pointer is correct, else don't restart the timer */
    if ((SRD1.state == SRD_STARTED) && (p != NULL))
    {
        osalSysLockFromISR();
        frequency_hz = p->frequency_hz;

        /* Calculate the new virtual timer time */
        wait_time = CH_CFG_ST_FREQUENCY / frequency_hz;

        /* Calculate the accumulator correction */
        if (p->accumulator != NULL)
        {
            *(p->accumulator) += CH_CFG_ST_FREQUENCY % frequency_hz;

            if (*(p->accumulator) >= frequency_hz)
            {
                *(p->accumulator) -= frequency_hz;
                wait_time += 1;
            }
        }

        /* Reinitialize the virtual timer with the calculated wait time */
        chVTSetI(p->polling_vt,
                 wait_time,
                 sensors_polled_callback,
                 param);

        /* Tell the reading thread to read the sensor requested sensor */
        if (queueReadI(&SRD1, &p->sensor) != MSG_OK)
        {
#if SRD_DEBUG
            SRD1.dbg_mailbox_overflow = true;
#endif
        }
    
        osalSysUnlockFromISR();
    }
}