#ifndef CAL_STORE_H_
#define CAL_STORE_H_

#include "compass.h"

/* Returns 0, -ENOENT when no calibration is saved, or a negative errno. */
int cal_store_load(struct vec3 *offset);

int cal_store_save(const struct vec3 *offset);

#endif /* CAL_STORE_H_ */
