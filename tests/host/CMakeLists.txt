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
