#ifndef COMPASS_H_
#define COMPASS_H_

#include <stdbool.h>

#define COMPASS_OK 0
#define COMPASS_ERR_NO_DATA (-1)
#define COMPASS_ERR_SPAN (-2)
#define COMPASS_ERR_DEGENERATE (-3)

/* Minimum per-axis magnetometer span for a valid calibration. */
#define COMPASS_CAL_MIN_SPAN_GAUSS 0.2f

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

void cal_reset(struct compass_cal *cal);
void cal_update(struct compass_cal *cal, const struct vec3 *magn);
int cal_offsets(const struct compass_cal *cal, struct vec3 *offset);

#endif /* COMPASS_H_ */
