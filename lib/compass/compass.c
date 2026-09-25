#include <math.h>

#include "compass.h"

void cal_reset(struct compass_cal *cal)
{
	cal->count = 0U;
}

void cal_update(struct compass_cal *cal, const struct vec3 *magn)
{
	if (cal->count == 0U) {
		cal->min = *magn;
		cal->max = *magn;
	} else {
		cal->min.x = fminf(cal->min.x, magn->x);
		cal->min.y = fminf(cal->min.y, magn->y);
		cal->min.z = fminf(cal->min.z, magn->z);
		cal->max.x = fmaxf(cal->max.x, magn->x);
		cal->max.y = fmaxf(cal->max.y, magn->y);
		cal->max.z = fmaxf(cal->max.z, magn->z);
	}
	cal->count++;
}

int cal_offsets(const struct compass_cal *cal, struct vec3 *offset)
{
	float span_x;
	float span_y;
	float span_z;
	float min_span;

	if (cal->count == 0U) {
		return COMPASS_ERR_NO_DATA;
	}
	span_x = cal->max.x - cal->min.x;
	span_y = cal->max.y - cal->min.y;
	span_z = cal->max.z - cal->min.z;
	min_span = fminf(span_x, fminf(span_y, span_z));
	if (min_span < COMPASS_CAL_MIN_SPAN_GAUSS) {
		return COMPASS_ERR_SPAN;
	}
	if (min_span < COMPASS_CAL_MIN_SPAN_RATIO * fmaxf(span_x, fmaxf(span_y, span_z))) {
		return COMPASS_ERR_UNBALANCED;
	}
	offset->x = (cal->max.x + cal->min.x) / 2.0f;
	offset->y = (cal->max.y + cal->min.y) / 2.0f;
	offset->z = (cal->max.z + cal->min.z) / 2.0f;
	return COMPASS_OK;
}

#define RAD_TO_DEG (180.0f / 3.14159265f)

int tilt_compensated_heading(const struct vec3 *accel, const struct vec3 *magn,
			     float *heading_deg)
{
	/* Gravity points opposite to the accelerometer reading at rest. */
	float gx = -accel->x;
	float gy = -accel->y;
	float gz = -accel->z;
	float roll;
	float pitch;
	float xh;
	float yh;
	float heading;

	if (sqrtf(gx * gx + gy * gy + gz * gz) < COMPASS_MIN_ACCEL_MS2) {
		return COMPASS_ERR_DEGENERATE;
	}

	roll = atan2f(gy, gz);
	pitch = atan2f(-gx, sqrtf(gy * gy + gz * gz));

	/* Rotate the magnetic field back to the horizontal plane. */
	xh = magn->x * cosf(pitch) + magn->y * sinf(roll) * sinf(pitch) +
	     magn->z * cosf(roll) * sinf(pitch);
	yh = magn->y * cosf(roll) - magn->z * sinf(roll);

	if (sqrtf(xh * xh + yh * yh) < COMPASS_MIN_HORIZ_GAUSS) {
		return COMPASS_ERR_DEGENERATE;
	}

	heading = atan2f(-yh, xh) * RAD_TO_DEG;
	if (heading < 0.0f) {
		heading += 360.0f;
	}
	if (heading >= 360.0f) {
		heading -= 360.0f;
	}
	*heading_deg = heading;
	return COMPASS_OK;
}

#define SECTOR_DEG (360.0f / (float)COMPASS_LED_COUNT)

void ema_update(struct compass_ema *ema, const struct vec3 *sample, float alpha)
{
	if (!ema->initialized) {
		ema->value = *sample;
		ema->initialized = true;
		return;
	}
	ema->value.x += alpha * (sample->x - ema->value.x);
	ema->value.y += alpha * (sample->y - ema->value.y);
	ema->value.z += alpha * (sample->z - ema->value.z);
}

static float wrap180(float deg)
{
	float d = fmodf(deg, 360.0f);

	if (d > 180.0f) {
		d -= 360.0f;
	} else if (d <= -180.0f) {
		d += 360.0f;
	}
	return d;
}

int north_led_index(float heading_deg, int prev_index, float hysteresis_deg)
{
	/* Angle of north clockwise from the N edge. */
	float north = fmodf(360.0f - heading_deg, 360.0f);
	int candidate;
	float from_prev;

	if (north < 0.0f) {
		north += 360.0f;
	}
	candidate = (int)lroundf(north / SECTOR_DEG) % COMPASS_LED_COUNT;

	if ((prev_index < 0) || (prev_index >= COMPASS_LED_COUNT) || (candidate == prev_index)) {
		return candidate;
	}
	from_prev = fabsf(wrap180(north - SECTOR_DEG * (float)prev_index));
	if (from_prev > (SECTOR_DEG / 2.0f) + hysteresis_deg) {
		return candidate;
	}
	return prev_index;
}
