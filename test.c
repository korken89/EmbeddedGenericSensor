struct _sensor
{
	msg_t (* init_sensor)(void);
	msg_t (* read_sensor)(void);
};

typedef struct
{
	uint16_t frequency_hz;
	struct _sensor sensor;
	virtual_timer_t *vt;
} polled_sensor_t;

typedef struct
{
	expchannel_t interrupt_channel;
	uint32_t mode;
	struct _sensor sensor;
};


msg_t my_polled_sensor1_init(void);
msg_t my_polled_sensor1_read(void);

msg_t my_polled_sensor2_init(void);
msg_t my_polled_sensor2_read(void);


static virtual_timer_t vt1;
static virtual_timer_t vt2;

const polled_sensor_t polled_sensors[] = {
	{
		.frequency_hz = 80,
		.sensor = {
			.init_sensor = my_polled_sensor1_init,
			.read_sensor = my_polled_sensor1_read
		},
		.vt = &vt1
	},
	{
		.frequency_hz = 10,
		.sensor = {
			.init_sensor = my_polled_sensor2_init,
			.read_sensor = my_polled_sensor2_read
		},
		.vt = &vt2
	}
};