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

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

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
        if (pollsenp[i].accumulated_ticks != NULL)
            *(pollsenp[i].accumulated_ticks) = 0;

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

    /* Everything OK, transverse the state */
    srdp->state = SRD_STOPPED;

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
            /* The parameter sent to the VT is the pointer to the
               corresponding polled_sensor_t structure to be able to
               access it from the callback function */
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

        /* Everything OK, transverse the state */
        srdp->state = SRD_STOPPED;
        return MSG_OK;    
    }
    else
        return MSG_RESET;
}

void sensors_interrupt_callback(EXTDriver *extp, expchannel_t channel)
{
    /* Check if the driver is in the correct state, else disable the
       interrupt */
    if (SRD1.state == SRD_STARTED)
    {
        osalSysLockFromISR();
        osalSysUnlockFromISR();
    }
    else
        extChannelDisable(extp, channel);
}

/**
 * @brief               Virtual timer callback function for sensor reads.
 * 
 * @param[in] param     Input parameter pointing to the corresponding
 *                      polled_sensor_t configuration.
 */
void sensors_polled_callback(void *param)
{
    polled_sensor_t *p = (polled_sensor_t *)param;

    /* Check if the driver is in the correct state, else don't restart the
       timer to keep it off */
    if (SRD1.state == SRD_STARTED)
    {
        osalSysLockFromISR();

        /* Tell the reading thread to read the sensor requested sensor */
        if (p->sensor.priority_sensor == true)
        {
            /* Priority sensor, put it to the front of the queue */
            // chMBPostAhead();
        }
        else
        {
         /* Not a priority sensor, put it to the end of the queue */
           // chMBPost();
        }
    
        osalSysUnlockFromISR();
    }
}