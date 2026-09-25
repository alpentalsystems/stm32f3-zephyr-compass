#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include "cal_store.h"
#include "compass.h"
#include "leds.h"
#include "sensors.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define TICK_MS 50
#define TICKS_PER_SECOND (1000 / TICK_MS)
#define EMA_ALPHA 0.2f
#define HYSTERESIS_DEG 5.0f
#define MAX_READ_ERRORS 20
#define CAL_DURATION_MS 15000
#define SPIN_TICKS_PER_STEP 2
#define DEBOUNCE_MS 200

#define FLASHES_UNCALIBRATED 2
#define FLASHES_CAL_SAVED 1
#define FLASHES_CAL_REJECTED 3
#define FLASHES_SAVE_FAILED 5

enum mode {
	MODE_RUN,
	MODE_CALIBRATE,
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb;
static atomic_t button_pressed;
static int64_t last_press_ms;

static void on_button(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	int64_t now = k_uptime_get();

	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if ((now - last_press_ms) < DEBOUNCE_MS) {
		return;
	}
	last_press_ms = now;
	atomic_set(&button_pressed, 1);
}

static int button_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&button)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		return err;
	}
	gpio_init_callback(&button_cb, on_button, BIT(button.pin));
	return gpio_add_callback(button.port, &button_cb);
}

static bool take_button_press(void)
{
	return atomic_cas(&button_pressed, 1, 0);
}

static void fatal_blink(void)
{
	for (;;) {
		/* Nothing left to report an LED error to. */
		(void)leds_flash_all(1);
	}
}

static void flash_all(int times)
{
	int err = leds_flash_all(times);

	if (err != 0) {
		LOG_ERR("LED flash failed: %d", err);
	}
}

static void print_raw(const struct vec3 *a, const struct vec3 *m)
{
	printk("RAW,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", (double)a->x, (double)a->y,
	       (double)a->z, (double)m->x, (double)m->y, (double)m->z);
}

static void load_offset(struct vec3 *offset)
{
	int err = cal_store_load(offset);

	if (err == 0) {
		LOG_INF("calibration loaded: %.3f %.3f %.3f", (double)offset->x,
			(double)offset->y, (double)offset->z);
		return;
	}
	if (err == -ENOENT) {
		LOG_WRN("no saved calibration; press USER to calibrate");
	} else {
		LOG_ERR("loading calibration failed: %d", err);
	}
	offset->x = 0.0f;
	offset->y = 0.0f;
	offset->z = 0.0f;
	flash_all(FLASHES_UNCALIBRATED);
}

static void finish_calibration(const struct compass_cal *cal, struct vec3 *offset)
{
	struct vec3 new_offset;
	int err = cal_offsets(cal, &new_offset);

	if (err != COMPASS_OK) {
		LOG_WRN("calibration rejected (%d): turn flat in a full circle, then flip and tilt",
			err);
		flash_all(FLASHES_CAL_REJECTED);
		return;
	}
	*offset = new_offset;
	LOG_INF("calibration: %.3f %.3f %.3f", (double)offset->x, (double)offset->y,
		(double)offset->z);

	err = cal_store_save(offset);
	if (err != 0) {
		LOG_ERR("saving calibration failed: %d", err);
		flash_all(FLASHES_SAVE_FAILED);
		return;
	}
	flash_all(FLASHES_CAL_SAVED);
}

int main(void)
{
	struct vec3 offset;
	struct compass_cal cal;
	struct compass_ema accel_f = {0};
	struct compass_ema magn_f = {0};
	enum mode mode = MODE_RUN;
	int64_t cal_end_ms = 0;
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
	err = button_init();
	if (err != 0) {
		LOG_ERR("button not ready: %d", err);
		fatal_blink();
	}
	load_offset(&offset);

	for (;;) {
		struct vec3 a;
		struct vec3 m;
		float heading;

		k_msleep(TICK_MS);
		tick++;

		if (take_button_press()) {
			if (mode == MODE_RUN) {
				cal_reset(&cal);
				cal_end_ms = k_uptime_get() + CAL_DURATION_MS;
				mode = MODE_CALIBRATE;
				LOG_INF("calibration started: turn flat in a full circle, "
					"then flip and tilt");
			} else {
				mode = MODE_RUN;
				LOG_INF("calibration canceled");
			}
			led = COMPASS_LED_NONE;
		}

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

		if (mode == MODE_CALIBRATE) {
			cal_update(&cal, &m);
			err = leds_show((int)((tick / SPIN_TICKS_PER_STEP) % COMPASS_LED_COUNT));
			if (err != 0) {
				LOG_ERR("LED update failed: %d", err);
			}
			if (k_uptime_get() >= cal_end_ms) {
				finish_calibration(&cal, &offset);
				magn_f.initialized = false;
				mode = MODE_RUN;
			}
			continue;
		}

		m.x -= offset.x;
		m.y -= offset.y;
		m.z -= offset.z;
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
