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

#define DEG_TO_RAD (3.14159265f / 180.0f)
#define GRAVITY 9.81f

/* Rotate a NED world vector into board axes (yaw, then pitch, then roll). */
static struct vec3 world_to_board(struct vec3 w, float yaw_deg, float pitch_deg, float roll_deg)
{
	float y = yaw_deg * DEG_TO_RAD;
	float p = pitch_deg * DEG_TO_RAD;
	float r = roll_deg * DEG_TO_RAD;
	struct vec3 a = v(cosf(y) * w.x + sinf(y) * w.y, -sinf(y) * w.x + cosf(y) * w.y, w.z);
	struct vec3 b = v(cosf(p) * a.x - sinf(p) * a.z, a.y, sinf(p) * a.x + cosf(p) * a.z);

	return v(b.x, cosf(r) * b.y + sinf(r) * b.z, -sinf(r) * b.y + cosf(r) * b.z);
}

/* Accel and magn readings for a board at the given attitude. */
static void make_inputs(float yaw, float pitch, float roll, struct vec3 *accel, struct vec3 *magn)
{
	struct vec3 g = world_to_board(v(0.0f, 0.0f, GRAVITY), yaw, pitch, roll);

	*accel = v(-g.x, -g.y, -g.z);
	*magn = world_to_board(v(0.30f, 0.0f, 0.40f), yaw, pitch, roll);
}

static float angle_diff(float a, float b)
{
	return fabsf(remainderf(a - b, 360.0f));
}

static void check_heading(float yaw, float pitch, float roll, float tol)
{
	struct vec3 accel;
	struct vec3 magn;
	float heading = -1.0f;

	make_inputs(yaw, pitch, roll, &accel, &magn);
	CHECK(tilt_compensated_heading(&accel, &magn, &heading) == COMPASS_OK);
	CHECK(angle_diff(heading, yaw) <= tol);
}

static void test_heading_level(void)
{
	check_heading(0.0f, 0.0f, 0.0f, 0.1f);
	check_heading(90.0f, 0.0f, 0.0f, 0.1f);
	check_heading(180.0f, 0.0f, 0.0f, 0.1f);
	check_heading(270.0f, 0.0f, 0.0f, 0.1f);
}

static void test_heading_tilted(void)
{
	check_heading(250.0f, -15.0f, 20.0f, 0.5f);
	check_heading(30.0f, 25.0f, -10.0f, 0.5f);
	check_heading(135.0f, 40.0f, 35.0f, 0.5f);
}

static void test_heading_range(void)
{
	struct vec3 accel;
	struct vec3 magn;
	float heading = -1.0f;

	make_inputs(359.5f, 0.0f, 0.0f, &accel, &magn);
	CHECK(tilt_compensated_heading(&accel, &magn, &heading) == COMPASS_OK);
	CHECK(heading >= 0.0f);
	CHECK(heading < 360.0f);
}

static void test_heading_degenerate_inputs(void)
{
	struct vec3 zero = v(0.0f, 0.0f, 0.0f);
	struct vec3 level = v(0.0f, 0.0f, -GRAVITY);
	struct vec3 vertical_field = v(0.0f, 0.0f, 0.5f);
	struct vec3 magn = v(0.3f, 0.0f, 0.4f);
	float heading;

	CHECK(tilt_compensated_heading(&zero, &magn, &heading) == COMPASS_ERR_DEGENERATE);
	CHECK(tilt_compensated_heading(&level, &vertical_field, &heading) ==
	      COMPASS_ERR_DEGENERATE);
}

static void test_ema_first_sample_sets_value(void)
{
	struct compass_ema ema = {0};
	struct vec3 s = v(1.0f, 2.0f, 3.0f);

	ema_update(&ema, &s, 0.25f);
	CHECK(ema.initialized);
	CHECK_NEAR(ema.value.x, 1.0f, 1e-6f);
	CHECK_NEAR(ema.value.y, 2.0f, 1e-6f);
	CHECK_NEAR(ema.value.z, 3.0f, 1e-6f);
}

static void test_ema_moves_by_alpha(void)
{
	struct compass_ema ema = {0};
	struct vec3 s1 = v(0.0f, 0.0f, 0.0f);
	struct vec3 s2 = v(4.0f, -8.0f, 2.0f);

	ema_update(&ema, &s1, 0.25f);
	ema_update(&ema, &s2, 0.25f);
	CHECK_NEAR(ema.value.x, 1.0f, 1e-6f);
	CHECK_NEAR(ema.value.y, -2.0f, 1e-6f);
	CHECK_NEAR(ema.value.z, 0.5f, 1e-6f);
}

static void test_led_index_without_previous(void)
{
	CHECK(north_led_index(0.0f, COMPASS_LED_NONE, 5.0f) == 0);
	CHECK(north_led_index(90.0f, COMPASS_LED_NONE, 5.0f) == 6);
	CHECK(north_led_index(180.0f, COMPASS_LED_NONE, 5.0f) == 4);
	CHECK(north_led_index(270.0f, COMPASS_LED_NONE, 5.0f) == 2);
	CHECK(north_led_index(45.0f, COMPASS_LED_NONE, 5.0f) == 7);
	CHECK(north_led_index(350.0f, COMPASS_LED_NONE, 5.0f) == 0);
}

static void test_led_index_sector_boundary(void)
{
	/* North at 22.4 and 22.6 degrees clockwise from the N edge. */
	CHECK(north_led_index(337.6f, COMPASS_LED_NONE, 5.0f) == 0);
	CHECK(north_led_index(337.4f, COMPASS_LED_NONE, 5.0f) == 1);
}

static void test_led_index_hysteresis(void)
{
	/* North at 25 degrees: inside the hysteresis band of LED 0. */
	CHECK(north_led_index(335.0f, 0, 5.0f) == 0);
	/* North at 28 degrees: past the band, switch to LED 1. */
	CHECK(north_led_index(332.0f, 0, 5.0f) == 1);
	/* North at 20 degrees coming from LED 1: stay on LED 1. */
	CHECK(north_led_index(340.0f, 1, 5.0f) == 1);
	/* Wrap around: north at 355 degrees coming from LED 7 (315). */
	CHECK(north_led_index(5.0f, 7, 5.0f) == 0);
}

int main(void)
{
	test_cal_offsets_without_samples_fails();
	test_cal_offsets_are_midpoints();
	test_cal_small_span_is_rejected();
	test_cal_reset_clears_samples();
	test_heading_level();
	test_heading_tilted();
	test_heading_range();
	test_heading_degenerate_inputs();
	test_ema_first_sample_sets_value();
	test_ema_moves_by_alpha();
	test_led_index_without_previous();
	test_led_index_sector_boundary();
	test_led_index_hysteresis();

	if (failures != 0) {
		printf("%d check(s) failed\n", failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
