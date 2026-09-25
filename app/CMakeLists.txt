cmake_minimum_required(VERSION 3.20.0)

# Flash with OpenOCD; the board default is STM32CubeProgrammer.
set(BOARD_FLASH_RUNNER openocd)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32f3_compass)

target_include_directories(app PRIVATE ../lib/compass/include)
target_sources(app PRIVATE
  src/main.c
  src/leds.c
  src/sensors.c
  src/cal_store.c
  ../lib/compass/compass.c
)
