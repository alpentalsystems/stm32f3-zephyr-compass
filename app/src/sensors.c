#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "sensors.h"

static const struct device *const accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct device *const magn_dev = DEVICE_DT_GET(DT_ALIAS(magn0));

/* Board axis n = sign * chip axis src. */
struct axis_map {
	uint8_t src;
	int8_t sign;
};

/* Measured: flat gives chip z +1 g, N edge up raises chip y, E edge up lowers chip x. */
static const struct axis_map accel_map[3] = {{1, 1}, {0, -1}, {2, -1}};
/* Measured: facing N vs S changes chip y, W vs E changes chip x; z sign keeps the dip constant. */
static const struct axis_map magn_map[3] = {{1, 1}, {0, -1}, {2, -1}};

static struct vec3 to_board(const struct sensor_value raw[3], const struct axis_map map[3])
{
	float chip[3] = {
		sensor_value_to_float(&raw[0]),
		sensor_value_to_float(&raw[1]),
		sensor_value_to_float(&raw[2]),
	};
	struct vec3 out = {
		(float)map[0].sign * chip[map[0].src],
		(float)map[1].sign * chip[map[1].src],
		(float)map[2].sign * chip[map[2].src],
	};

	return out;
}

int sensors_init(void)
{
	if (!device_is_ready(accel_dev) || !device_is_ready(magn_dev)) {
		return -ENODEV;
	}
	return 0;
}

int sensors_read(struct vec3 *accel, struct vec3 *magn)
{
	struct sensor_value raw[3];
	int err;

	err = sensor_sample_fetch(accel_dev);
	if (err != 0) {
		return err;
	}
	err = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, raw);
	if (err != 0) {
		return err;
	}
	*accel = to_board(raw, accel_map);

	err = sensor_sample_fetch(magn_dev);
	if (err != 0) {
		return err;
	}
	err = sensor_channel_get(magn_dev, SENSOR_CHAN_MAGN_XYZ, raw);
	if (err != 0) {
		return err;
	}
	*magn = to_board(raw, magn_map);
	return 0;
}
