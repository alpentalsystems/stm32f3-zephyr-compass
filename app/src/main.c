#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "compass.h"
#include "leds.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

int main(void)
{
	int err = leds_init();

	if (err != 0) {
		LOG_ERR("LEDs not ready: %d", err);
		return 0;
	}
	for (int i = 0;; i = (i + 1) % COMPASS_LED_COUNT) {
		err = leds_show(i);
		if (err != 0) {
			LOG_ERR("LED update failed: %d", err);
		}
		k_msleep(250);
	}
	return 0;
}
