#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "compass.h"
#include "leds.h"
#include "sensors.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define TICK_MS 50
#define TICKS_PER_SECOND (1000 / TICK_MS)
#define EMA_ALPHA 0.2f
#define HYSTERESIS_DEG 5.0f
#define MAX_READ_ERRORS 20

static void fatal_blink(void)
{
	for (;;) {
		/* Nothing left to report an LED error to. */
		(void)leds_flash_all(1);
	}
}

static void print_raw(const struct vec3 *a, const struct vec3 *m)
{
	printk("RAW,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", (double)a->x, (double)a->y,
	       (double)a->z, (double)m->x, (double)m->y, (double)m->z);
}

int main(void)
{
	struct compass_ema accel_f = {0};
	struct compass_ema magn_f = {0};
	int led = COMPASS_LED_NONE;
	int read_errors = 0;
	uint32_t tick = 0U;
	int err;

	err = leds_init();
	if (err != 0) {
		LOG_ERR("LEDs not ready: %d", err);
		return 0;
	}
	err = sensors_init();
	if (err != 0) {
		LOG_ERR("sensors not ready: %d", err);
		fatal_blink();
	}

	for (;;) {
		struct vec3 a;
		struct vec3 m;
		float heading;

		k_msleep(TICK_MS);
		tick++;

		err = sensors_read(&a, &m);
		if (err != 0) {
			if (read_errors == 0) {
				LOG_ERR("sensor read failed: %d", err);
			}
			read_errors++;
			if (read_errors >= MAX_READ_ERRORS) {
				LOG_ERR("%d sensor reads failed in a row", read_errors);
				fatal_blink();
			}
			continue;
		}
		read_errors = 0;
		if (IS_ENABLED(CONFIG_APP_RAW_STREAM)) {
			print_raw(&a, &m);
		}

		ema_update(&accel_f, &a, EMA_ALPHA);
		ema_update(&magn_f, &m, EMA_ALPHA);
		if (tilt_compensated_heading(&accel_f.value, &magn_f.value, &heading) !=
		    COMPASS_OK) {
			continue;
		}
		led = north_led_index(heading, led, HYSTERESIS_DEG);
		err = leds_show(led);
		if (err != 0) {
			LOG_ERR("LED update failed: %d", err);
		}
		if ((tick % TICKS_PER_SECOND) == 0U) {
			LOG_INF("heading %.1f deg, north LED %d", (double)heading, led);
		}
	}
	return 0;
}
