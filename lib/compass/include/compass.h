#ifndef COMPASS_H_
#define COMPASS_H_

#include <stdbool.h>

#define COMPASS_OK 0
#define COMPASS_ERR_NO_DATA (-1)
#define COMPASS_ERR_SPAN (-2)
#define COMPASS_ERR_DEGENERATE (-3)
#define COMPASS_ERR_UNBALANCED (-4)

/* Minimum per-axis magnetometer span for a valid calibration. */
#define COMPASS_CAL_MIN_SPAN_GAUSS 0.2f

/*
 * A full rotation gives every axis a span near twice the field strength.
 * Each span must be at least this fraction of the largest one.
 */
#define COMPASS_CAL_MIN_SPAN_RATIO 0.7f

/* Below these magnitudes the heading is undefined. */
#define COMPASS_MIN_ACCEL_MS2 1.0f
#define COMPASS_MIN_HORIZ_GAUSS 0.01f

/* LED ring positions: 0 = N edge, then clockwise in 45 degree steps. */
#define COMPASS_LED_COUNT 8
#define COMPASS_LED_NONE (-1)

/* Vector in board axes: x to the N edge, y to the E edge, z down. */
struct vec3 {
	float x;
	float y;
	float z;
};

struct compass_cal {
	struct vec3 min;
	struct vec3 max;
	unsigned int count;
};

struct compass_ema {
	struct vec3 value;
	bool initialized;
};

void cal_reset(struct compass_cal *cal);
void cal_update(struct compass_cal *cal, const struct vec3 *magn);
int cal_offsets(const struct compass_cal *cal, struct vec3 *offset);

/*
 * Heading of the board's N edge, clockwise from magnetic north, in [0, 360).
 * accel: accelerometer reading in m/s^2 (level and at rest: z = -9.81).
 * magn: calibrated magnetometer reading in gauss.
 */
int tilt_compensated_heading(const struct vec3 *accel, const struct vec3 *magn,
			     float *heading_deg);

/* Exponential moving average; the first sample sets the value. */
void ema_update(struct compass_ema *ema, const struct vec3 *sample, float alpha);

/*
 * Ring position that points to magnetic north as seen from the board.
 * Keeps prev_index until north is more than hysteresis_deg past the
 * sector boundary. Pass COMPASS_LED_NONE when there is no previous index.
 */
int north_led_index(float heading_deg, int prev_index, float hysteresis_deg);

#endif /* COMPASS_H_ */
