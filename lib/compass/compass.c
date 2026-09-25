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
	if (cal->count == 0U) {
		return COMPASS_ERR_NO_DATA;
	}
	if (((cal->max.x - cal->min.x) < COMPASS_CAL_MIN_SPAN_GAUSS) ||
	    ((cal->max.y - cal->min.y) < COMPASS_CAL_MIN_SPAN_GAUSS) ||
	    ((cal->max.z - cal->min.z) < COMPASS_CAL_MIN_SPAN_GAUSS)) {
		return COMPASS_ERR_SPAN;
	}
	offset->x = (cal->max.x + cal->min.x) / 2.0f;
	offset->y = (cal->max.y + cal->min.y) / 2.0f;
	offset->z = (cal->max.z + cal->min.z) / 2.0f;
	return COMPASS_OK;
}
