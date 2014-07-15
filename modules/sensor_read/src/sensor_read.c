/*
    Copyright (C) 2014- Emil Fresk

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 *
 * Abstraction Layer for sensor initialization and reading driver. Designed
 * to works with ChibiOS/RT 3.0.0 - Copyright (C) 2006-2013 Giovanni Di Sirio .
 *
 */

#include "ch.h"
#include "hal.h"
#include "sensor_read.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SRD1 driver identifier.
 */
SensorReadDriver SRD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   SRD1 thread work area.
 */
THD_WORKING_AREA(waThreadSRD1, SRD_THREAD_STACKSIZE);

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief           The thread that reads the sensors beeing queued
 *                  by the drier.
 * 
 * @param[in] arg   Pointer to the SensorReadDriver object.
 */
__attribute__((noreturn))
static THD_FUNCTION(ThreadSRD1, arg)
{
    SensorReadDriver *srdp = (SensorReadDriver *)arg;
    msg_t message;
    generic_sensor_t *senp;
#if SRD_DEBUG
    chRegSetThreadName("ThreadSRD1");
#endif

    while (1)
    {
        /* Get the latest sensor read request */
        chMBFetch(&srdp->srd_mailbox, &message, TIME_INFINITE);

        /* Convert the message to a sensor pointer */
        senp = (generic_sensor_t *)message;

        /* Call its read function */
        senp->read_sensor(senp->params);
    }
}

/*===========================================================================*/
/* Driver local inline functions.                                            */
/*===========================================================================*/

/**
 * @brief           Converts a EXT channel to the corresponding
 *                  interrupt_sensor_t pointer.
 * 
 * @param[in] srdp  Pointer to the SensorReadDriver object.
 * @param[in] ch    Channel to convert.
 * 
 * @return          Pointer to the corresponding interrupt_sensor_t object.
 * @retval NULL     If there was no interrupt_sensor_t object associated with
 *                  the inputed channel.
 */
static inline
const interrupt_sensor_t *extchannel2interrupt_sensor(SensorReadDriver *srdp,
                                                      expchannel_t ch)
{
    int8_t id = srdp->expchannel_lookup[ch];

    if (id != -1)
        return &srdp->interrupt_sensor_ptr[id];
    else
        return NULL;
}

/**
 * @brief           Queues a sensor read to the reading thread.
 * 
 * @param[in] srdp  Pointer to the SensorReadDriver object.
 * @param[in] senp  Pointer the generic_sensor_t object to be queued.
 * 
 * @return              The operation status.
 * @retval MSG_OK       If a read request has been correctly queued.
 * @retval MSG_TIMEOUT  If the queue is full and the request cannot be queued.
 * 
 * @iclass
 */
static inline msg_t queueReadI(SensorReadDriver *srdp,
                               const generic_sensor_t *senp)
{
    if (senp->priority_sensor == true)
        /* Priority sensor, put it at the front of the queue */
        return chMBPostAheadI(&srdp->srd_mailbox, (msg_t)senp);
    else
        /* Not a priority sensor, put it at the end of the queue */
        return chMBPostI(&srdp->srd_mailbox, (msg_t)senp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief               Interrupt callback to be used in the EXT driver config
 *                      for sensor reads. Used with interrupt driven sensors.
 * 
 * @param[in] extp      Pointer to the EXTDriver object.
 * @param[in] channel   Pointer the ext channel.
 * 
 * 
 */
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
static void sensors_polled_callback(void *param)
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

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief           Initializes the SensorReadDriver object.
 * @note            This in done explicitly in SensorsInit.
 * 
 * @param[in] srdp  Pointer to the SensorReadDriver object.
 * 
 * @init
 */
void SensorReadObjectInit(SensorReadDriver *srdp)
{
    srdp->state = SRD_UNINITIALIZED;
    srdp->interrupt_sensor_ptr = NULL;
    srdp->polled_sensor_ptr = NULL;
    srdp->interrupt_sensor_cnt = 0;
    srdp->polled_sensor_cnt = 0;
    chMBObjectInit(&srdp->srd_mailbox, srdp->messages, SRD_MAILBOX_SIZE);
#if SRD_DEBUG
    SRD1.dbg_mailbox_overflow = false;
#endif
}

/**
 * @brief                   Initializes the sensor read functionallity.
 * 
 * @param[in] srdp          Pointer to the SensorReadDriver object.
 * @param[in] intsenp       Pointer to the interrupt sensor read configuration.
 * @param[in] pollsenp      Pointer to the polled sensor read configuration.
 * @param[in] intsencnt     Number of interrupt driven sensors.
 * @param[in] pollsencnt    Number of polled driven sensors.
 * 
 * @return                  The operation status.
 * @retval MSG_OK           The initialization was successful.
 * @retval MSG_RESET        Something went wrong during the initialization.
 * 
 * @api
 */
msg_t SensorReadInit(SensorReadDriver *srdp,
                     const interrupt_sensor_t *intsenp,
                     const polled_sensor_t *pollsenp,
                     size_t intsencnt,
                     size_t pollsencnt)
{
    chDbgCheck(srdp != NULL);
    chDbgCheck(srdp->state == SRD_UNINITIALIZED);
    chDbgAssert((intsenp == NULL) && (intsencnt != 0),
                "Pointer can't be NULL when the count is > 0");
    chDbgAssert((pollsenp == NULL) && (pollsencnt != 0),
                "Pointer can't be NULL when the count is > 0");
    
    size_t i;
    msg_t retval;

    /* Clear the lookup table */
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

    /* Everything OK, transverse the state and start the read thread */
    srdp->state = SRD_STOPPED;

    chThdCreateStatic(waThreadSRD1,
                      sizeof(waThreadSRD1), 
                      SRD_THREAD_PRIORITY, 
                      ThreadSRD1, 
                      srdp);

    return MSG_OK;
}

/**
 * @brief               Starts the interrupts and timers so sensors will be 
 *                      serviced.
 * 
 * @param[in] srdp      Pointer to the SensorReadDriver object.
 * 
 * @return              The operation status.
 * @retval MSG_OK       The enabling of interrupts and starting of the
 *                      virtual timers was successful.
 * @retval MSG_RESET    The driver was not in the correct state.
 * 
 * @api
 */
msg_t SensorReadStart(SensorReadDriver *srdp)
{
    size_t i;

    chDbgCheck(srdp != NULL);

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

/**
 * @brief               Stops the interrupts and timers so sensors will stop 
 *                      beeing serviced. The queued sensor reads are lost as
 *                      well.
 * 
 * @param[in] srdp      Pointer to the SensorReadDriver object.
 * 
 * @return              The operation status.
 * @retval MSG_OK       The disabling of interrupts and starting of the
 *                      virtual timers was successful.
 * @retval MSG_RESET    The driver was not in the correct state.
 * 
 * @api
 */
msg_t SensorReadStop(SensorReadDriver *srdp)
{
    size_t i;

    chDbgCheck(srdp != NULL);

    if (srdp->state == SRD_STARTED)
    {
        osalSysLock();

        /* Disable interrupts for interrupt driven sensors */
        for (i = 0; i < srdp->interrupt_sensor_cnt; i++)
            extChannelDisableI(&SRD_EXT_DRIVER,
                              srdp->interrupt_sensor_ptr[i].interrupt_channel);

        /* Reset timers for polled driven sensors */
        for (i = 0; i < srdp->polled_sensor_cnt; i++)
            chVTResetI(srdp->polled_sensor_ptr[i].polling_vt);

        osalSysUnlock();

        /* Reset the mailbox */
        chMBReset(&srdp->srd_mailbox);

        /* Everything OK, transverse the state */
        srdp->state = SRD_STOPPED;

        return MSG_OK;    
    }
    else
        return MSG_RESET;
}

/**
 * @brief           Queues a sensor read to the reading thread.
 * 
 * @param[in] srdp  Pointer to the SensorReadDriver object.
 * @param[in] senp  Pointer the generic_sensor_t object to be queued.
 * 
 * @return              The operation status.
 * @retval MSG_OK       If a read request has been correctly queued.
 * @retval MSG_TIMEOUT  If the queue is full and the request cannot be queued.
 * 
 * @iclass
 */
msg_t SensorReadInjectReadI(SensorReadDriver *srdp,
                            const generic_sensor_t *senp)
{
    return queueReadI(srdp, senp);
}

/**
 * @brief           Queues a sensor read to the reading thread.
 * 
 * @param[in] srdp  Pointer to the SensorReadDriver object.
 * @param[in] senp  Pointer the generic_sensor_t object to be queued.
 * @param[in] time  The number of ticks before the opertion timeouts.
 *                  TIME_IMMEDIATE and TIME_INFINITE is allowed.
 * 
 * @return              The operation status.
 * @retval MSG_OK       If a read request has been correctly queued.
 * @retval MSG_RESET    If the queue has been reset while waiting.
 * @retval MSG_TIMEOUT  If the queue operation timed out and the request
 *                      cannot be queued.
 * 
 * @sclass
 */
msg_t SensorReadInjectReadS(SensorReadDriver *srdp,
                            const generic_sensor_t *senp,
                            systime_t time)
{
    if (senp->priority_sensor == true)
        /* Priority sensor, put it at the front of the queue */
        return chMBPostAheadS(&srdp->srd_mailbox, (msg_t)senp, time);
    else
        /* Not a priority sensor, put it at the end of the queue */
        return chMBPostS(&srdp->srd_mailbox, (msg_t)senp, time);
}
