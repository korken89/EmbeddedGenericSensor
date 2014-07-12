#include "ch.h"
#include "hal.h"
#include "myusb.h"
#include "sensors.h"

msg_t my_polled_sensor1_init(void *);
msg_t my_polled_sensor1_read(void *);

msg_t my_polled_sensor2_init(void *);
msg_t my_polled_sensor2_read(void *);

static virtual_timer_t polled_vt1;
static virtual_timer_t polled_vt2;
static uint32_t polled1_accumulator;

static const polled_sensor_t polled_sensors[] = {
    {
        .frequency_hz = 80,
        .accumulator = &polled1_accumulator,
        .sensor = {
            .init_sensor = my_polled_sensor1_init,
            .read_sensor = my_polled_sensor1_read,
            .params = NULL,
            .priority_sensor = false
        },
        .polling_vt = &polled_vt1
    },
    {
        .frequency_hz = 10,
        .accumulator = NULL,
        .sensor = {
            .init_sensor = my_polled_sensor2_init,
            .read_sensor = my_polled_sensor2_read,
            .params = NULL,
            .priority_sensor = false
        },
        .polling_vt = &polled_vt2
    }
};

int main(void)
{
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured 
     *   device drivers and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread
     *   and the RTOS is active.
     */
    halInit();
    chSysInit();

    /*
     *
     * Initializes a serial-over-USB CDC driver.
     * 
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     *
     * Activates the USB driver and then the USB bus pull-up on D+.
     * 
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    /* 
     *
     *  User application.
     *
     */

    while(1)
    {
        palTogglePad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(200);
    }
}
