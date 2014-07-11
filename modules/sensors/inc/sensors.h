#ifndef __SENSORS_H
#define __SENSORS_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

struct _sensor_calls
{
    msg_t (* init_sensor)(void);
    msg_t (* read_sensor)(void);
};

typedef struct
{
    uint16_t frequency_hz;
    struct _sensor_calls sensor;
    virtual_timer_t *polling_vt;
} polled_sensor_t;

typedef struct
{
    expchannel_t interrupt_channel;
    uint32_t mode;
    struct _sensor_calls sensor;
} interrupt_sensor_t;

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

#endif /* __SENSORS_H */
