#ifndef SENSORS_H_
#define SENSORS_H_

#include "compass.h"

int sensors_init(void);

/* Accel in m/s^2 and magn in gauss, in board axes (x N edge, y E edge, z down). */
int sensors_read(struct vec3 *accel, struct vec3 *magn);

#endif /* SENSORS_H_ */
