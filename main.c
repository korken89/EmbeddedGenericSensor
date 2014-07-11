#include "ch.h"
#include "hal.h"
#include "myusb.h"
#include "sensors.h"

msg_t my_polled_sensor1_init(void);
msg_t my_polled_sensor1_read(void);

msg_t my_polled_sensor2_init(void);
msg_t my_polled_sensor2_read(void);


static virtual_timer_t vt1;
static virtual_timer_t vt2;

static const polled_sensor_t polled_sensors[] = {
    {
        .frequency_hz = 80,
        .sensor = {
            .init_sensor = my_polled_sensor1_init,
            .read_sensor = my_polled_sensor1_read
        },
        .polling_vt = &vt1
    },
    {
        .frequency_hz = 10,
        .sensor = {
            .init_sensor = my_polled_sensor2_init,
            .read_sensor = my_polled_sensor2_read
        },
        .polling_vt = &vt2
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
    chThdSleepMilliseconds(500);
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
