#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "compass.h"
#include "leds.h"

#define FLASH_MS 200

/* Ring positions in compass order, starting at the N edge. */
static const struct gpio_dt_spec ring[COMPASS_LED_COUNT] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(red_led_3), gpios),    /* N  LD3  */
	GPIO_DT_SPEC_GET(DT_NODELABEL(orange_led_5), gpios), /* NE LD5  */
	GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_7), gpios),  /* E  LD7  */
	GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_9), gpios),   /* SE LD9  */
	GPIO_DT_SPEC_GET(DT_NODELABEL(red_led_10), gpios),   /* S  LD10 */
	GPIO_DT_SPEC_GET(DT_NODELABEL(orange_led_8), gpios), /* SW LD8  */
	GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_6), gpios),  /* W  LD6  */
	GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_4), gpios),   /* NW LD4  */
};

int leds_init(void)
{
	for (int i = 0; i < COMPASS_LED_COUNT; i++) {
		int err;

		if (!gpio_is_ready_dt(&ring[i])) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&ring[i], GPIO_OUTPUT_INACTIVE);
		if (err != 0) {
			return err;
		}
	}
	return 0;
}

int leds_show(int index)
{
	for (int i = 0; i < COMPASS_LED_COUNT; i++) {
		int err = gpio_pin_set_dt(&ring[i], (i == index) ? 1 : 0);

		if (err != 0) {
			return err;
		}
	}
	return 0;
}

int leds_set_all(bool on)
{
	for (int i = 0; i < COMPASS_LED_COUNT; i++) {
		int err = gpio_pin_set_dt(&ring[i], on ? 1 : 0);

		if (err != 0) {
			return err;
		}
	}
	return 0;
}

int leds_flash_all(int times)
{
	for (int i = 0; i < times; i++) {
		int err = leds_set_all(true);

		if (err != 0) {
			return err;
		}
		k_msleep(FLASH_MS);
		err = leds_set_all(false);
		if (err != 0) {
			return err;
		}
		k_msleep(FLASH_MS);
	}
	return 0;
}
