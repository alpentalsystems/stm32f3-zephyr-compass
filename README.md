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

- Blue USER button: start calibration. One LED spins for 15 seconds. Hold
  the board in your hands (resting it face down presses the buttons):
  1. keep it flat and turn it one full circle, like a plate on a table;
  2. then flip it over, stand it on each edge, and tilt it around.

  Press USER again to cancel.
- LED flashes: 2 = no saved calibration at boot, 1 = calibration saved,
  3 = calibration rejected (not enough rotation, try again), 5 = saving
  failed, continuous = fatal error (see the console log).
- Raw data for plots: build with `-- -DCONFIG_APP_RAW_STREAM=y` to print
  `RAW,ax,ay,az,mx,my,mz` lines at 20 Hz, in board axes and before
  calibration.

Calibration is rejected unless every magnetometer axis spans at least
70% of the largest span. A full rotation gives each axis a span near twice
the field strength; flipping the board without turning it flat leaves the
horizontal axes short, and the offsets come out wrong.

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
- [ ] Axis mapping (`app/src/sensors.c`): flat gives `az` about -9.8;
      raising the N edge makes `ax` positive; raising the E edge makes
      `ay` positive.
- [ ] After calibration, the lit LED matches a phone compass within one
      LED, flat and tilted about 30 degrees.
- [ ] Calibration survives a reset.
