#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "compass.h"
#include "leds.h"
#include "sensors.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define TICK_MS 50

static void print_raw(const struct vec3 *a, const struct vec3 *m)
{
	printk("RAW,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", (double)a->x, (double)a->y,
	       (double)a->z, (double)m->x, (double)m->y, (double)m->z);
}

int main(void)
{
	int err = leds_init();

	if (err != 0) {
		LOG_ERR("LEDs not ready: %d", err);
		return 0;
	}
	err = sensors_init();
	if (err != 0) {
		LOG_ERR("sensors not ready: %d", err);
		return 0;
	}
	for (;;) {
		struct vec3 a;
		struct vec3 m;

		k_msleep(TICK_MS);
		err = sensors_read(&a, &m);
		if (err != 0) {
			LOG_ERR("sensor read failed: %d", err);
			continue;
		}
		if (IS_ENABLED(CONFIG_APP_RAW_STREAM)) {
			print_raw(&a, &m);
		}
	}
	return 0;
}
