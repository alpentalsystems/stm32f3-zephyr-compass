#include <math.h>
#include <stdio.h>

#include "compass.h"

static int failures;

#define CHECK(cond)                                                             \
	do {                                                                    \
		if (!(cond)) {                                                  \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
			failures++;                                             \
		}                                                               \
	} while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((a) - (b)) <= (tol))

static struct vec3 v(float x, float y, float z)
{
	struct vec3 r = {x, y, z};

	return r;
}

static void test_cal_offsets_without_samples_fails(void)
{
	struct compass_cal cal;
	struct vec3 off;

	cal_reset(&cal);
	CHECK(cal_offsets(&cal, &off) == COMPASS_ERR_NO_DATA);
}

static void test_cal_offsets_are_midpoints(void)
{
	struct compass_cal cal;
	struct vec3 off;
	struct vec3 s1 = v(0.5f, -0.1f, 0.2f);
	struct vec3 s2 = v(-0.1f, 0.3f, -0.4f);
	struct vec3 s3 = v(0.2f, 0.1f, 0.0f);

	cal_reset(&cal);
	cal_update(&cal, &s1);
	cal_update(&cal, &s2);
	cal_update(&cal, &s3);
	CHECK(cal_offsets(&cal, &off) == COMPASS_OK);
	CHECK_NEAR(off.x, 0.2f, 1e-6f);
	CHECK_NEAR(off.y, 0.1f, 1e-6f);
	CHECK_NEAR(off.z, -0.1f, 1e-6f);
}

static void test_cal_small_span_is_rejected(void)
{
	struct compass_cal cal;
	struct vec3 off;
	struct vec3 s1 = v(0.1f, 0.1f, 0.1f);
	struct vec3 s2 = v(0.5f, 0.5f, 0.15f);

	cal_reset(&cal);
	cal_update(&cal, &s1);
	cal_update(&cal, &s2);
	CHECK(cal_offsets(&cal, &off) == COMPASS_ERR_SPAN);
}

static void test_cal_reset_clears_samples(void)
{
	struct compass_cal cal;
	struct vec3 off;
	struct vec3 s1 = v(1.0f, 1.0f, 1.0f);
	struct vec3 s2 = v(-1.0f, -1.0f, -1.0f);

	cal_reset(&cal);
	cal_update(&cal, &s1);
	cal_update(&cal, &s2);
	cal_reset(&cal);
	CHECK(cal_offsets(&cal, &off) == COMPASS_ERR_NO_DATA);
}

int main(void)
{
	test_cal_offsets_without_samples_fails();
	test_cal_offsets_are_midpoints();
	test_cal_small_span_is_rejected();
	test_cal_reset_clears_samples();

	if (failures != 0) {
		printf("%d check(s) failed\n", failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
