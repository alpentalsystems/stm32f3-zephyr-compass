#ifndef LEDS_H_
#define LEDS_H_

#include <stdbool.h>

int leds_init(void);

/* Light one ring position (0 = N, clockwise); COMPASS_LED_NONE turns all off. */
int leds_show(int index);

int leds_set_all(bool on);

/* Blocking: all LEDs on and off, 200 ms each, the given number of times. */
int leds_flash_all(int times);

#endif /* LEDS_H_ */
