# Compass App Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Tasks 4-7 need the owner at the board (watching LEDs, moving the board), so run them inline, not in subagents.

**Goal:** Tilt-compensated compass on STM32F3 Discovery rev E: the LED pointing to magnetic north lights up, with button-triggered hard-iron calibration saved in flash.

**Architecture:** Portable math in `lib/compass` (no Zephyr headers), tested on the host with CMake/CTest. A Zephyr app in `app/` runs one 20 Hz loop with RUN and CALIBRATE modes and uses small modules for sensors, LEDs, and calibration storage.

**Tech Stack:** C11, Zephyr v4.4.2, Zephyr SDK 1.0.1, CMake/CTest (host), OpenOCD, GitHub Actions.

**Spec:** `docs/2026-09-25-compass-app-design.md`

## Global Constraints

- Board target: `stm32f3_disco@E`. Zephyr pinned to v4.4.2 by `west.yml`.
- `lib/compass` includes only C standard headers.
- Board axes: x toward the board's N edge, y toward the E edge, z down.
- Units: accel in m/s^2, magn in gauss, angles in degrees.
- Ring index 0..7 = N, NE, E, SE, S, SW, W, NW = LD3, LD5, LD7, LD9, LD10, LD8, LD6, LD4.
- Loop period 50 ms; calibration 15 s; min calibration span 0.2 gauss.
- Settings key `compass/mag_offset`; NVS on `storage_partition` (6 KB = 3 x 2 KB sectors).
- Flash patterns: 2 = uncalibrated at boot, 1 = calibration saved, 3 = calibration rejected, 5 = save failed, continuous = fatal.
- Zephyr C style: tabs, braces on every `if`, `/* */` comments, explicit comparisons.
- Commit messages: conventional commits. Do not push until the owner says so.

## Commands

Run from `~/ws/stm32f3-ws` with `source ~/ws/zp/zephyr-venv/bin/activate`.

- Host tests: `cmake -S stm32f3-zephyr-compass/tests/host -B stm32f3-zephyr-compass/build/host && cmake --build stm32f3-zephyr-compass/build/host && ctest --test-dir stm32f3-zephyr-compass/build/host --output-on-failure`
- Firmware build: `west build -p always -b stm32f3_disco@E -d build/compass stm32f3-zephyr-compass/app`
- Flash: `west flash -d build/compass`
- Serial console: `/dev/cu.usbmodem31103`, 115200 baud.

## File map

| File | Responsibility |
|---|---|
| `lib/compass/include/compass.h` | Types, constants, math API |
| `lib/compass/compass.c` | Calibration, heading, EMA, LED index |
| `tests/host/CMakeLists.txt` | Host test build |
| `tests/host/test_compass.c` | Host tests |
| `app/CMakeLists.txt` | Zephyr app build, OpenOCD default runner |
| `app/Kconfig` | `CONFIG_APP_RAW_STREAM` |
| `app/prj.conf` | Kernel config |
| `app/src/leds.[ch]` | 8-LED ring |
| `app/src/sensors.[ch]` | Sensor reads in board axes |
| `app/src/cal_store.[ch]` | Offset load/save with settings |
| `app/src/main.c` | Loop, modes, button |
| `README.md` | Build, flash, test, board checklist |
| `.github/workflows/ci.yml` | Host tests + firmware build |
| `.gitignore` | `build/` |

---

### Task 1: Host test harness and calibration math

**Files:**
- Create: `.gitignore`, `lib/compass/include/compass.h`, `lib/compass/compass.c`, `tests/host/CMakeLists.txt`, `tests/host/test_compass.c`

**Interfaces:**
- Produces: `struct vec3`, `struct compass_cal`, `cal_reset()`, `cal_update()`, `cal_offsets()`, error codes `COMPASS_OK`, `COMPASS_ERR_NO_DATA`, `COMPASS_ERR_SPAN`, `COMPASS_ERR_DEGENERATE`.

- [ ] **Step 1: Create `.gitignore`**

```
build/
```

- [ ] **Step 2: Create `tests/host/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20)
project(compass_host_tests C)

enable_testing()

add_executable(test_compass
  test_compass.c
  ../../lib/compass/compass.c
)
target_include_directories(test_compass PRIVATE ../../lib/compass/include)
set_target_properties(test_compass PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
target_compile_options(test_compass PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_compass PRIVATE m)

add_test(NAME compass COMMAND test_compass)
```

- [ ] **Step 3: Write the failing tests in `tests/host/test_compass.c`**

```c
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
```

- [ ] **Step 4: Run the host tests to see them fail**

Run the host test command from **Commands**.
Expected: build fails with `compass.h` not found.

- [ ] **Step 5: Create `lib/compass/include/compass.h`**

```c
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
```

- [ ] **Step 6: Create `lib/compass/compass.c`**

```c
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
```

- [ ] **Step 7: Run the host tests to see them pass**

Run the host test command. Expected: `all checks passed`, CTest `100% tests passed`, no compiler warnings.

- [ ] **Step 8: Commit**

```bash
git -C stm32f3-zephyr-compass add .gitignore lib tests
git -C stm32f3-zephyr-compass commit -m "feat: add hard-iron calibration math with host tests"
```

---

### Task 2: Tilt-compensated heading

**Files:**
- Modify: `lib/compass/include/compass.h`, `lib/compass/compass.c`, `tests/host/test_compass.c`

**Interfaces:**
- Consumes: `struct vec3`, error codes from Task 1.
- Produces: `int tilt_compensated_heading(const struct vec3 *accel, const struct vec3 *magn, float *heading_deg)`. `accel` is the accelerometer reading (at rest, level: z is about -9.81). Heading is the direction the N edge faces, clockwise from magnetic north, in [0, 360).

- [ ] **Step 1: Add the failing tests to `tests/host/test_compass.c`**

Insert after `test_cal_reset_clears_samples()`:

```c
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
```

Add to `main()` after `test_cal_reset_clears_samples();`:

```c
	test_heading_level();
	test_heading_tilted();
	test_heading_range();
	test_heading_degenerate_inputs();
```

- [ ] **Step 2: Run the host tests to see them fail**

Expected: build fails with implicit declaration of `tilt_compensated_heading` (treated as error).

- [ ] **Step 3: Add to `compass.h`**, after `#define COMPASS_CAL_MIN_SPAN_GAUSS 0.2f`:

```c
/* Below these magnitudes the heading is undefined. */
#define COMPASS_MIN_ACCEL_MS2 1.0f
#define COMPASS_MIN_HORIZ_GAUSS 0.01f
```

and before `#endif`:

```c
/*
 * Heading of the board's N edge, clockwise from magnetic north, in [0, 360).
 * accel: accelerometer reading in m/s^2 (level and at rest: z = -9.81).
 * magn: calibrated magnetometer reading in gauss.
 */
int tilt_compensated_heading(const struct vec3 *accel, const struct vec3 *magn,
			     float *heading_deg);
```

- [ ] **Step 4: Add to `compass.c`**, after `cal_offsets()`:

```c
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
```

- [ ] **Step 5: Run the host tests to see them pass**

Expected: `all checks passed`, no warnings.

- [ ] **Step 6: Commit**

```bash
git -C stm32f3-zephyr-compass add lib tests
git -C stm32f3-zephyr-compass commit -m "feat: add tilt-compensated heading"
```

---

### Task 3: Smoothing and north LED index

**Files:**
- Modify: `lib/compass/include/compass.h`, `lib/compass/compass.c`, `tests/host/test_compass.c`

**Interfaces:**
- Produces: `struct compass_ema`, `void ema_update(struct compass_ema *ema, const struct vec3 *sample, float alpha)`, `int north_led_index(float heading_deg, int prev_index, float hysteresis_deg)`, `COMPASS_LED_COUNT` (8), `COMPASS_LED_NONE` (-1).

- [ ] **Step 1: Add the failing tests to `tests/host/test_compass.c`**

Insert after `test_heading_degenerate_inputs()`:

```c
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
```

Add to `main()` after `test_heading_degenerate_inputs();`:

```c
	test_ema_first_sample_sets_value();
	test_ema_moves_by_alpha();
	test_led_index_without_previous();
	test_led_index_sector_boundary();
	test_led_index_hysteresis();
```

- [ ] **Step 2: Run the host tests to see them fail**

Expected: build fails with unknown type `struct compass_ema` and implicit declarations.

- [ ] **Step 3: Add to `compass.h`**, after `#define COMPASS_MIN_HORIZ_GAUSS 0.01f`:

```c
/* LED ring positions: 0 = N edge, then clockwise in 45 degree steps. */
#define COMPASS_LED_COUNT 8
#define COMPASS_LED_NONE (-1)
```

after `struct compass_cal`:

```c
struct compass_ema {
	struct vec3 value;
	bool initialized;
};
```

and before `#endif`:

```c
/* Exponential moving average; the first sample sets the value. */
void ema_update(struct compass_ema *ema, const struct vec3 *sample, float alpha);

/*
 * Ring position that points to magnetic north as seen from the board.
 * Keeps prev_index until north is more than hysteresis_deg past the
 * sector boundary. Pass COMPASS_LED_NONE when there is no previous index.
 */
int north_led_index(float heading_deg, int prev_index, float hysteresis_deg);
```

- [ ] **Step 4: Add to `compass.c`**, after `tilt_compensated_heading()`:

```c
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
```

- [ ] **Step 5: Run the host tests to see them pass**

Expected: `all checks passed`, no warnings.

- [ ] **Step 6: Commit**

```bash
git -C stm32f3-zephyr-compass add lib tests
git -C stm32f3-zephyr-compass commit -m "feat: add smoothing and north LED selection with hysteresis"
```

---

### Task 4: Zephyr app skeleton and LED ring

**Files:**
- Create: `app/CMakeLists.txt`, `app/prj.conf`, `app/src/leds.h`, `app/src/leds.c`, `app/src/main.c`

**Interfaces:**
- Consumes: `COMPASS_LED_COUNT`, `COMPASS_LED_NONE` from `compass.h`.
- Produces: `int leds_init(void)`, `int leds_show(int index)`, `int leds_set_all(bool on)`, `int leds_flash_all(int times)`.

- [ ] **Step 1: Create `app/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20.0)

# Flash with OpenOCD; the board default is STM32CubeProgrammer.
set(BOARD_FLASH_RUNNER openocd)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32f3_compass)

target_include_directories(app PRIVATE ../lib/compass/include)
target_sources(app PRIVATE
  src/main.c
  src/leds.c
  ../lib/compass/compass.c
)
```

- [ ] **Step 2: Create `app/prj.conf`**

```
CONFIG_GPIO=y
CONFIG_LOG=y
```

- [ ] **Step 3: Create `app/src/leds.h`**

```c
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
```

- [ ] **Step 4: Create `app/src/leds.c`**

```c
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
```

- [ ] **Step 5: Create `app/src/main.c` (LED ring check)**

```c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "compass.h"
#include "leds.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

int main(void)
{
	int err = leds_init();

	if (err != 0) {
		LOG_ERR("LEDs not ready: %d", err);
		return 0;
	}
	for (int i = 0;; i = (i + 1) % COMPASS_LED_COUNT) {
		err = leds_show(i);
		if (err != 0) {
			LOG_ERR("LED update failed: %d", err);
		}
		k_msleep(250);
	}
	return 0;
}
```

- [ ] **Step 6: Build and check the runner**

Run the firmware build command.
Expected: build succeeds with no warnings; `grep flash-runner build/compass/zephyr/runners.yaml` prints `flash-runner: openocd`.

- [ ] **Step 7: Flash and check on the board (owner)**

Run `west flash -d build/compass`.
Expected: one LED at a time, clockwise, starting at LD3 (N): LD3, LD5, LD7, LD9, LD10, LD8, LD6, LD4. The owner confirms the order.

- [ ] **Step 8: Run host tests, then commit**

```bash
git -C stm32f3-zephyr-compass add app
git -C stm32f3-zephyr-compass commit -m "feat: add Zephyr app with LED ring driver"
```

---

### Task 5: Sensor reads, raw stream, and axis mapping

**Files:**
- Create: `app/Kconfig`, `app/src/sensors.h`, `app/src/sensors.c`
- Modify: `app/CMakeLists.txt`, `app/prj.conf`, `app/src/main.c`

**Interfaces:**
- Consumes: `struct vec3` from `compass.h`; `leds_*` from Task 4.
- Produces: `int sensors_init(void)`, `int sensors_read(struct vec3 *accel, struct vec3 *magn)` in board axes; `CONFIG_APP_RAW_STREAM`.

- [ ] **Step 1: Create `app/Kconfig`**

```
config APP_RAW_STREAM
	bool "Print raw sensor data as CSV"
	help
	  Print RAW,ax,ay,az,mx,my,mz lines at the sample rate, in board
	  axes and before calibration: accel in m/s^2, magn in gauss.

source "Kconfig.zephyr"
```

- [ ] **Step 2: Replace `app/prj.conf`**

```
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_SENSOR=y
CONFIG_FPU=y
CONFIG_CBPRINTF_FP_SUPPORT=y
CONFIG_MAIN_STACK_SIZE=2048
```

- [ ] **Step 3: Create `app/src/sensors.h`**

```c
#ifndef SENSORS_H_
#define SENSORS_H_

#include "compass.h"

int sensors_init(void);

/* Accel in m/s^2 and magn in gauss, in board axes (x N edge, y E edge, z down). */
int sensors_read(struct vec3 *accel, struct vec3 *magn);

#endif /* SENSORS_H_ */
```

- [ ] **Step 4: Create `app/src/sensors.c` with an identity mapping**

```c
#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "sensors.h"

static const struct device *const accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct device *const magn_dev = DEVICE_DT_GET(DT_ALIAS(magn0));

/* Board axis n = sign * chip axis src. */
struct axis_map {
	uint8_t src;
	int8_t sign;
};

static const struct axis_map accel_map[3] = {{0, 1}, {1, 1}, {2, 1}};
static const struct axis_map magn_map[3] = {{0, 1}, {1, 1}, {2, 1}};

static struct vec3 to_board(const struct sensor_value raw[3], const struct axis_map map[3])
{
	float chip[3] = {
		sensor_value_to_float(&raw[0]),
		sensor_value_to_float(&raw[1]),
		sensor_value_to_float(&raw[2]),
	};
	struct vec3 out = {
		(float)map[0].sign * chip[map[0].src],
		(float)map[1].sign * chip[map[1].src],
		(float)map[2].sign * chip[map[2].src],
	};

	return out;
}

int sensors_init(void)
{
	if (!device_is_ready(accel_dev) || !device_is_ready(magn_dev)) {
		return -ENODEV;
	}
	return 0;
}

int sensors_read(struct vec3 *accel, struct vec3 *magn)
{
	struct sensor_value raw[3];
	int err;

	err = sensor_sample_fetch(accel_dev);
	if (err != 0) {
		return err;
	}
	err = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, raw);
	if (err != 0) {
		return err;
	}
	*accel = to_board(raw, accel_map);

	err = sensor_sample_fetch(magn_dev);
	if (err != 0) {
		return err;
	}
	err = sensor_channel_get(magn_dev, SENSOR_CHAN_MAGN_XYZ, raw);
	if (err != 0) {
		return err;
	}
	*magn = to_board(raw, magn_map);
	return 0;
}
```

- [ ] **Step 5: Add `src/sensors.c` to `target_sources` in `app/CMakeLists.txt`**, after `src/leds.c`.

- [ ] **Step 6: Replace `app/src/main.c` (raw stream)**

```c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "compass.h"
#include "leds.h"
#include "sensors.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define TICK_MS 50

static void print_raw(const struct vec3 *a, const struct vec3 *m)
{
	printk("RAW,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", (double)a->x, (double)a->y,
	       (double)a->z, (double)m->x, (double)m->y, (double)m->z);
}

int main(void)
{
	int err = leds_init();

	if (err != 0) {
		LOG_ERR("LEDs not ready: %d", err);
		return 0;
	}
	err = sensors_init();
	if (err != 0) {
		LOG_ERR("sensors not ready: %d", err);
		return 0;
	}
	for (;;) {
		struct vec3 a;
		struct vec3 m;

		k_msleep(TICK_MS);
		err = sensors_read(&a, &m);
		if (err != 0) {
			LOG_ERR("sensor read failed: %d", err);
			continue;
		}
		if (IS_ENABLED(CONFIG_APP_RAW_STREAM)) {
			print_raw(&a, &m);
		}
	}
	return 0;
}
```

- [ ] **Step 7: Build with the raw stream on, flash, capture**

```bash
west build -p always -b stm32f3_disco@E -d build/compass stm32f3-zephyr-compass/app -- -DCONFIG_APP_RAW_STREAM=y
west flash -d build/compass
```

Expected: build with no warnings; `RAW,...` lines at 20 Hz on the serial console.

- [ ] **Step 8: Determine the axis mapping (owner moves the board, controller reads the stream)**

For each pose, hold still for 2 s and average the RAW values. With the identity mapping, the printed columns are chip axes.

| Pose | Rule | Result sets |
|---|---|---|
| Flat, top up | Chip accel axis near ±9.8: board z = that axis with sign so board `az` is **negative** | `accel_map[2]` |
| Raise the N edge ~45° | Chip accel axis that changes most: board x = that axis with sign so board `ax` becomes **positive** | `accel_map[0]` |
| Raise the E edge ~45° | Chip accel axis that changes most: board y = that axis with sign so board `ay` becomes **positive** | `accel_map[1]` |
| Flat, N edge to magnetic north (phone compass) | Chip magn axis with largest horizontal value: board x = that axis with sign so board `mx` is **positive** | `magn_map[0]` |
| Flat, N edge to magnetic west | Board y = chip magn axis with sign so board `my` is **positive** | `magn_map[1]` |
| Flat, top up (Korea: field points down) | Remaining chip magn axis, sign so board `mz` is **positive** | `magn_map[2]` |

Each map must use each chip axis exactly once.

- [ ] **Step 9: Put the measured tables into `sensors.c`**, replacing the identity tables, with one comment line above each stating the poses used. Rebuild with the raw stream on, flash, and repeat the six poses.

Expected: all six rules hold in board axes (flat: `az` about -9.8, `mz` positive; N edge up: `ax` positive; E edge up: `ay` positive; facing north: `mx` positive and `my` near 0; facing west: `my` positive).

- [ ] **Step 10: Run host tests, then commit**

```bash
git -C stm32f3-zephyr-compass add app
git -C stm32f3-zephyr-compass commit -m "feat: read sensors in board axes with optional raw stream"
```

---

### Task 6: RUN mode (heading to LED)

**Files:**
- Modify: `app/src/main.c`

**Interfaces:**
- Consumes: `sensors_*` (Task 5), `leds_*` (Task 4), `ema_update`, `tilt_compensated_heading`, `north_led_index` (Tasks 2-3).

- [ ] **Step 1: Replace `app/src/main.c`**

```c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "compass.h"
#include "leds.h"
#include "sensors.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define TICK_MS 50
#define TICKS_PER_SECOND (1000 / TICK_MS)
#define EMA_ALPHA 0.2f
#define HYSTERESIS_DEG 5.0f
#define MAX_READ_ERRORS 20

static void fatal_blink(void)
{
	for (;;) {
		/* Nothing left to report an LED error to. */
		(void)leds_flash_all(1);
	}
}

static void print_raw(const struct vec3 *a, const struct vec3 *m)
{
	printk("RAW,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", (double)a->x, (double)a->y,
	       (double)a->z, (double)m->x, (double)m->y, (double)m->z);
}

int main(void)
{
	struct compass_ema accel_f = {0};
	struct compass_ema magn_f = {0};
	int led = COMPASS_LED_NONE;
	int read_errors = 0;
	uint32_t tick = 0U;
	int err;

	err = leds_init();
	if (err != 0) {
		LOG_ERR("LEDs not ready: %d", err);
		return 0;
	}
	err = sensors_init();
	if (err != 0) {
		LOG_ERR("sensors not ready: %d", err);
		fatal_blink();
	}

	for (;;) {
		struct vec3 a;
		struct vec3 m;
		float heading;

		k_msleep(TICK_MS);
		tick++;

		err = sensors_read(&a, &m);
		if (err != 0) {
			if (read_errors == 0) {
				LOG_ERR("sensor read failed: %d", err);
			}
			read_errors++;
			if (read_errors >= MAX_READ_ERRORS) {
				LOG_ERR("%d sensor reads failed in a row", read_errors);
				fatal_blink();
			}
			continue;
		}
		read_errors = 0;
		if (IS_ENABLED(CONFIG_APP_RAW_STREAM)) {
			print_raw(&a, &m);
		}

		ema_update(&accel_f, &a, EMA_ALPHA);
		ema_update(&magn_f, &m, EMA_ALPHA);
		if (tilt_compensated_heading(&accel_f.value, &magn_f.value, &heading) !=
		    COMPASS_OK) {
			continue;
		}
		led = north_led_index(heading, led, HYSTERESIS_DEG);
		err = leds_show(led);
		if (err != 0) {
			LOG_ERR("LED update failed: %d", err);
		}
		if ((tick % TICKS_PER_SECOND) == 0U) {
			LOG_INF("heading %.1f deg, north LED %d", (double)heading, led);
		}
	}
	return 0;
}
```

- [ ] **Step 2: Build (raw stream off), flash**

Expected: build with no warnings; log shows `heading ... north LED ...` once per second.

- [ ] **Step 3: Check on the board (owner)**

Expected: turning the board moves the lit LED the opposite way, one LED at a time, with no flicker when held still. Accuracy is not checked yet (no calibration); this checks direction and stability.

- [ ] **Step 4: Run host tests, then commit**

```bash
git -C stm32f3-zephyr-compass add app
git -C stm32f3-zephyr-compass commit -m "feat: light the LED that points to magnetic north"
```

---

### Task 7: Calibration mode and flash storage

**Files:**
- Create: `app/src/cal_store.h`, `app/src/cal_store.c`
- Modify: `app/CMakeLists.txt`, `app/prj.conf`, `app/src/main.c`

**Interfaces:**
- Consumes: `cal_reset`, `cal_update`, `cal_offsets` (Task 1) and everything from Task 6.
- Produces: `int cal_store_load(struct vec3 *offset)` (0, `-ENOENT` when nothing is saved, or a negative errno), `int cal_store_save(const struct vec3 *offset)`.

- [ ] **Step 1: Replace `app/prj.conf`**

```
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_SENSOR=y
CONFIG_FPU=y
CONFIG_CBPRINTF_FP_SUPPORT=y
CONFIG_MAIN_STACK_SIZE=2048

CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_NVS=y
CONFIG_SETTINGS=y
CONFIG_SETTINGS_NVS=y
# storage_partition is 6 KB: three 2 KB flash pages.
CONFIG_SETTINGS_NVS_SECTOR_COUNT=3
```

- [ ] **Step 2: Create `app/src/cal_store.h`**

```c
#ifndef CAL_STORE_H_
#define CAL_STORE_H_

#include "compass.h"

/* Returns 0, -ENOENT when no calibration is saved, or a negative errno. */
int cal_store_load(struct vec3 *offset);

int cal_store_save(const struct vec3 *offset);

#endif /* CAL_STORE_H_ */
```

- [ ] **Step 3: Create `app/src/cal_store.c`**

```c
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/settings/settings.h>

#include "cal_store.h"

#define KEY_LEAF "mag_offset"

static struct vec3 loaded_offset;
static bool found;

static int compass_settings_set(const char *key, size_t len, settings_read_cb read_cb,
				void *cb_arg)
{
	ssize_t n;

	if (strcmp(key, KEY_LEAF) != 0) {
		return -ENOENT;
	}
	if (len != sizeof(loaded_offset)) {
		return -EINVAL;
	}
	n = read_cb(cb_arg, &loaded_offset, sizeof(loaded_offset));
	if (n < 0) {
		return (int)n;
	}
	if (n != (ssize_t)sizeof(loaded_offset)) {
		return -EIO;
	}
	found = true;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(compass, "compass", NULL, compass_settings_set, NULL, NULL);

int cal_store_load(struct vec3 *offset)
{
	int err = settings_subsys_init();

	if (err != 0) {
		return err;
	}
	found = false;
	err = settings_load_subtree("compass");
	if (err != 0) {
		return err;
	}
	if (!found) {
		return -ENOENT;
	}
	*offset = loaded_offset;
	return 0;
}

int cal_store_save(const struct vec3 *offset)
{
	return settings_save_one("compass/" KEY_LEAF, offset, sizeof(*offset));
}
```

- [ ] **Step 4: Add `src/cal_store.c` to `target_sources` in `app/CMakeLists.txt`**, after `src/sensors.c`.

- [ ] **Step 5: Replace `app/src/main.c` (final)**

```c
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include "cal_store.h"
#include "compass.h"
#include "leds.h"
#include "sensors.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define TICK_MS 50
#define TICKS_PER_SECOND (1000 / TICK_MS)
#define EMA_ALPHA 0.2f
#define HYSTERESIS_DEG 5.0f
#define MAX_READ_ERRORS 20
#define CAL_DURATION_MS 15000
#define SPIN_TICKS_PER_STEP 2
#define DEBOUNCE_MS 200

#define FLASHES_UNCALIBRATED 2
#define FLASHES_CAL_SAVED 1
#define FLASHES_CAL_REJECTED 3
#define FLASHES_SAVE_FAILED 5

enum mode {
	MODE_RUN,
	MODE_CALIBRATE,
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb;
static atomic_t button_pressed;
static int64_t last_press_ms;

static void on_button(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	int64_t now = k_uptime_get();

	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if ((now - last_press_ms) < DEBOUNCE_MS) {
		return;
	}
	last_press_ms = now;
	atomic_set(&button_pressed, 1);
}

static int button_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&button)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		return err;
	}
	gpio_init_callback(&button_cb, on_button, BIT(button.pin));
	return gpio_add_callback(button.port, &button_cb);
}

static bool take_button_press(void)
{
	return atomic_cas(&button_pressed, 1, 0);
}

static void fatal_blink(void)
{
	for (;;) {
		/* Nothing left to report an LED error to. */
		(void)leds_flash_all(1);
	}
}

static void flash_all(int times)
{
	int err = leds_flash_all(times);

	if (err != 0) {
		LOG_ERR("LED flash failed: %d", err);
	}
}

static void print_raw(const struct vec3 *a, const struct vec3 *m)
{
	printk("RAW,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", (double)a->x, (double)a->y,
	       (double)a->z, (double)m->x, (double)m->y, (double)m->z);
}

static void load_offset(struct vec3 *offset)
{
	int err = cal_store_load(offset);

	if (err == 0) {
		LOG_INF("calibration loaded: %.3f %.3f %.3f", (double)offset->x,
			(double)offset->y, (double)offset->z);
		return;
	}
	if (err == -ENOENT) {
		LOG_WRN("no saved calibration; press USER to calibrate");
	} else {
		LOG_ERR("loading calibration failed: %d", err);
	}
	offset->x = 0.0f;
	offset->y = 0.0f;
	offset->z = 0.0f;
	flash_all(FLASHES_UNCALIBRATED);
}

static void finish_calibration(const struct compass_cal *cal, struct vec3 *offset)
{
	struct vec3 new_offset;
	int err = cal_offsets(cal, &new_offset);

	if (err != COMPASS_OK) {
		LOG_WRN("calibration rejected (%d): rotate through all orientations", err);
		flash_all(FLASHES_CAL_REJECTED);
		return;
	}
	*offset = new_offset;
	LOG_INF("calibration: %.3f %.3f %.3f", (double)offset->x, (double)offset->y,
		(double)offset->z);

	err = cal_store_save(offset);
	if (err != 0) {
		LOG_ERR("saving calibration failed: %d", err);
		flash_all(FLASHES_SAVE_FAILED);
		return;
	}
	flash_all(FLASHES_CAL_SAVED);
}

int main(void)
{
	struct vec3 offset;
	struct compass_cal cal;
	struct compass_ema accel_f = {0};
	struct compass_ema magn_f = {0};
	enum mode mode = MODE_RUN;
	int64_t cal_end_ms = 0;
	int led = COMPASS_LED_NONE;
	int read_errors = 0;
	uint32_t tick = 0U;
	int err;

	err = leds_init();
	if (err != 0) {
		LOG_ERR("LEDs not ready: %d", err);
		return 0;
	}
	err = sensors_init();
	if (err != 0) {
		LOG_ERR("sensors not ready: %d", err);
		fatal_blink();
	}
	err = button_init();
	if (err != 0) {
		LOG_ERR("button not ready: %d", err);
		fatal_blink();
	}
	load_offset(&offset);

	for (;;) {
		struct vec3 a;
		struct vec3 m;
		float heading;

		k_msleep(TICK_MS);
		tick++;

		if (take_button_press()) {
			if (mode == MODE_RUN) {
				cal_reset(&cal);
				cal_end_ms = k_uptime_get() + CAL_DURATION_MS;
				mode = MODE_CALIBRATE;
				LOG_INF("calibration started: rotate the board in all directions");
			} else {
				mode = MODE_RUN;
				LOG_INF("calibration canceled");
			}
			led = COMPASS_LED_NONE;
		}

		err = sensors_read(&a, &m);
		if (err != 0) {
			if (read_errors == 0) {
				LOG_ERR("sensor read failed: %d", err);
			}
			read_errors++;
			if (read_errors >= MAX_READ_ERRORS) {
				LOG_ERR("%d sensor reads failed in a row", read_errors);
				fatal_blink();
			}
			continue;
		}
		read_errors = 0;
		if (IS_ENABLED(CONFIG_APP_RAW_STREAM)) {
			print_raw(&a, &m);
		}

		if (mode == MODE_CALIBRATE) {
			cal_update(&cal, &m);
			err = leds_show((int)((tick / SPIN_TICKS_PER_STEP) % COMPASS_LED_COUNT));
			if (err != 0) {
				LOG_ERR("LED update failed: %d", err);
			}
			if (k_uptime_get() >= cal_end_ms) {
				finish_calibration(&cal, &offset);
				magn_f.initialized = false;
				mode = MODE_RUN;
			}
			continue;
		}

		m.x -= offset.x;
		m.y -= offset.y;
		m.z -= offset.z;
		ema_update(&accel_f, &a, EMA_ALPHA);
		ema_update(&magn_f, &m, EMA_ALPHA);
		if (tilt_compensated_heading(&accel_f.value, &magn_f.value, &heading) !=
		    COMPASS_OK) {
			continue;
		}
		led = north_led_index(heading, led, HYSTERESIS_DEG);
		err = leds_show(led);
		if (err != 0) {
			LOG_ERR("LED update failed: %d", err);
		}
		if ((tick % TICKS_PER_SECOND) == 0U) {
			LOG_INF("heading %.1f deg, north LED %d", (double)heading, led);
		}
	}
	return 0;
}
```

- [ ] **Step 6: Build, flash, check on the board (owner)**

Expected, in order:
1. First boot: all LEDs flash twice; log `no saved calibration`.
2. Press USER: one LED spins. Hold the board still for 15 s: 3 flashes, log `calibration rejected`.
3. Press USER, rotate the board through all orientations for 15 s: 1 flash, log `calibration: x y z`.
4. The lit LED points to magnetic north within one LED of a phone compass, also when tilted about 30°.
5. Press RESET: no double flash; log `calibration loaded` with the same values.
6. Press USER twice within 15 s: log `calibration canceled`, compass resumes.

- [ ] **Step 7: Run host tests, then commit**

```bash
git -C stm32f3-zephyr-compass add app
git -C stm32f3-zephyr-compass commit -m "feat: add button calibration mode saved to flash"
```

---

### Task 8: README and CI

**Files:**
- Modify: `README.md`
- Create: `.github/workflows/ci.yml`

- [ ] **Step 1: Replace `README.md`**

````markdown
# STM32F3 Discovery tilt-compensated compass (Zephyr)

The LED that points to magnetic north lights up on the STM32F3 Discovery
board (MB1035 revision E). The heading stays correct when the board is
tilted, and a button-triggered hard-iron calibration is saved in flash.

## Hardware

- STM32F3 Discovery, revision E (MB1035E): LSM303AGR accelerometer and
  magnetometer on I2C1, eight user LEDs LD3-LD10 in a ring, USER button.
- One USB cable to the USB ST-LINK port (flashing and serial console).

## Setup

```sh
mkdir stm32f3-ws && cd stm32f3-ws
west init -m https://github.com/alpentalsystems/stm32f3-zephyr-compass
west update
west sdk install -t arm-zephyr-eabi
```

Flashing uses OpenOCD (`brew install open-ocd` on macOS).

## Build and flash

```sh
west build -b stm32f3_disco@E -d build/compass stm32f3-zephyr-compass/app
west flash -d build/compass
```

Serial console: 115200 baud on the ST-LINK virtual COM port.

## Use

- Blue USER button: start calibration. One LED spins; rotate the board
  through all orientations for 15 seconds. Press again to cancel.
- LED flashes: 2 = no saved calibration at boot, 1 = calibration saved,
  3 = calibration rejected (not enough rotation), 5 = saving failed,
  continuous = fatal error (see the console log).
- Raw data for plots: build with `-- -DCONFIG_APP_RAW_STREAM=y` to print
  `RAW,ax,ay,az,mx,my,mz` lines at 20 Hz.

## Tests

The compass math in `lib/compass` has no Zephyr dependencies and is tested
on the host:

```sh
cmake -S tests/host -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

## Board checklist

- [ ] LEDs light clockwise from LD3 (N).
- [ ] Axis mapping (`app/src/sensors.c`): flat gives `az` about -9.8 and
      `mz` positive; raising the N edge makes `ax` positive; raising the E
      edge makes `ay` positive.
- [ ] After calibration, the lit LED matches a phone compass within one
      LED, flat and tilted about 30 degrees.
- [ ] Calibration survives a reset.
````

- [ ] **Step 2: Create `.github/workflows/ci.yml`**

```yaml
name: CI

on:
  push:
  pull_request:

jobs:
  host-tests:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - name: Build and run host tests
        run: |
          cmake -S tests/host -B build/host
          cmake --build build/host
          ctest --test-dir build/host --output-on-failure

  firmware:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
        with:
          path: stm32f3-zephyr-compass
      - uses: zephyrproject-rtos/action-zephyr-setup@v1
        with:
          app-path: stm32f3-zephyr-compass
          toolchains: arm-zephyr-eabi
      - name: Build firmware
        run: west build -b stm32f3_disco@E -d build/compass stm32f3-zephyr-compass/app
```

- [ ] **Step 3: Run host tests and a firmware build locally**

Expected: `all checks passed`; firmware builds with no warnings.

- [ ] **Step 4: Commit**

```bash
git -C stm32f3-zephyr-compass add README.md .github
git -C stm32f3-zephyr-compass commit -m "docs: add README and CI workflow"
```

- [ ] **Step 5: Push and check CI (only after the owner says to push)**

Expected: both jobs pass on GitHub Actions. If `action-zephyr-setup` inputs differ from the ones above, fix the workflow from the action's README and amend this commit before the owner reviews.
