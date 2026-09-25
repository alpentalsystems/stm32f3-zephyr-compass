# Tilt-compensated compass on STM32F3 Discovery — design

## Context

Demo project for Alpental Systems: a tilt-compensated compass on the STM32F3
Discovery board (MB1035 revision E, STM32F303VC) running Zephyr v4.4.2. It
backs a blog post on alpentalsystems.com, so the code must be easy to read and
explain.

Hardware facts confirmed on the board:

- Board target: `stm32f3_disco@E`.
- Accelerometer: LSM303AGR accel (`lis2dh` driver), I2C1, alias `accel0`.
- Magnetometer: LSM303AGR magn (`lis2mdl` driver), I2C1, alias `magn0`. No
  data-ready interrupt is wired in devicetree.
- User button: alias `sw0` (PA0).
- LEDs LD3..LD10 in a ring. Ring position to LED (from the silkscreen):
  N=LD3, NE=LD5, E=LD7, SE=LD9, S=LD10, SW=LD8, W=LD6, NW=LD4.
- Flash: 6 KB `storage_partition` at 0x3e800.
- Console: USART1 on the ST-LINK virtual COM port.
- Flashing: OpenOCD (`stm32cubeprogrammer` is the board default and is not
  installed).
- Hardware FPU available.

## Goals

- The lit LED always points to magnetic north as the board turns.
- Heading stays correct when the board is tilted (tilt compensation).
- Hard-iron calibration from a button-triggered mode, saved in flash.
- Raw data stream for before/after calibration plots.
- Math covered by host tests that run on macOS; CI builds firmware and runs
  tests.

## Non-goals

- Soft-iron (ellipsoid) calibration.
- Gyroscope use or sensor fusion.
- True-north (declination) correction.
- Sensor triggers, zbus, or multiple application threads.
- Power management.

## Repository layout

```
stm32f3-zephyr-compass/
  west.yml
  app/
    CMakeLists.txt      default flash runner: openocd
    prj.conf
    Kconfig             CONFIG_APP_RAW_STREAM
    src/main.c          20 Hz loop, RUN/CALIBRATE modes, button flag
    src/sensors.[ch]    read accel + magn, map chip axes to board axes
    src/leds.[ch]       8-LED ring: show one, spin, flash all
    src/cal_store.[ch]  load/save magnetometer offsets with settings
  lib/compass/
    include/compass.h   portable C, no Zephyr headers
    compass.c
  tests/host/
    CMakeLists.txt      CMake + CTest, host compiler
    test_compass.c
  .github/workflows/ci.yml
```

`app/` compiles `lib/compass/compass.c` as a normal source file.

## lib/compass (pure functions)

Types: `struct vec3 { float x, y, z; }` in board axes; accel in m/s^2, magn
in gauss.

- `cal_reset`, `cal_update(cal, magn)`: track per-axis min and max.
- `cal_offsets(cal, out)`: offset = (max + min) / 2 per axis. Returns an
  error if any axis span is below `COMPASS_CAL_MIN_SPAN_GAUSS` (0.2).
- `tilt_compensated_heading(accel, magn, out_deg)`: roll and pitch from
  gravity, magnetometer rotated to horizontal, heading = atan2(-Yh, Xh)
  normalized to [0, 360). Returns an error if |accel| or the horizontal
  magnetic magnitude is near zero.
- `ema_update(state, sample, alpha)`: exponential moving average on vec3.
- `north_led_index(heading_deg, prev_index, hysteresis_deg)`: ring position
  pointing to north as seen from the board, round((360 - heading) / 45) mod 8,
  keeping `prev_index` unless the new sector is entered by more than
  `hysteresis_deg`.

Functions return `int` (0 or negative error) where they can fail.

## app runtime

- Boot: init devices, load offsets (settings key `compass/mag_offset`, NVS
  backend on `storage_partition`). No saved calibration: flash all LEDs twice,
  run with zero offsets.
- RUN, every 50 ms: read sensors, subtract offsets, EMA, heading, LED index
  with hysteresis, light that LED. Log heading at 1 Hz.
- CALIBRATE, entered on button press: spin one LED around the ring, collect
  min/max for 15 s. Valid result: save, flash all LEDs once, back to RUN.
  Invalid: flash all LEDs 3 times, keep old offsets, back to RUN. A button
  press during calibration cancels it.
- `CONFIG_APP_RAW_STREAM=y`: print `RAW,ax,ay,az,mx,my,mz` at 20 Hz.
- Axis mapping from chip axes to board axes is a single table in
  `sensors.c`, determined on the board and documented in the README.

## Error handling

- Device not ready at boot: log error, blink all LEDs forever.
- Sensor read error: log (rate-limited), skip the tick; 20 consecutive
  errors: error blink forever.
- Heading error from `lib/compass`: keep the current LED for that tick.
- Settings load error: log, run uncalibrated with the uncalibrated hint.
- Settings save error: log, flash all LEDs 5 times, keep offsets in RAM.

## Testing

- Host tests (CTest), written first: calibration min/max/offsets and the
  span check; heading for flat N/E/S/W vectors; same heading after rotating
  inputs by known pitch and roll; `north_led_index` around the 22.5 degree
  boundaries; hysteresis; EMA; error returns for degenerate inputs.
- Manual board checklist in the README: axis mapping, north LED against a
  phone compass, calibration pass and fail, offsets survive reset.
- CI (GitHub Actions): host tests, and `west build -b stm32f3_disco@E app` in
  the Zephyr CI container. Build only; no hardware.
