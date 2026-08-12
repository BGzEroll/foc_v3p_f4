cmake_minimum_required(VERSION 3.22)

project(foc_math_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(foc_math_test
    foc_math_test.cpp
    ../../user_lib/drivers/foc/foc_math.cpp
)

add_executable(foc_monitor_test
    foc_monitor_test.cpp
    ../../user_lib/drivers/foc/foc_core.cpp
    ../../user_lib/drivers/foc/foc_math.cpp
)

add_executable(foc_control_test
    foc_control_test.cpp
    ../../user_lib/drivers/foc/foc_core.cpp
    ../../user_lib/drivers/foc/foc_math.cpp
)

target_include_directories(foc_math_test PRIVATE
    ../../user_lib
)

target_include_directories(foc_monitor_test PRIVATE
    stubs
    ../../user_lib
)

target_include_directories(foc_control_test PRIVATE
    stubs
    ../../user_lib
)

target_compile_options(foc_math_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

target_compile_options(foc_monitor_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

target_compile_options(foc_control_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

enable_testing()
add_test(NAME foc_math_test COMMAND foc_math_test)
add_test(NAME foc_monitor_test COMMAND foc_monitor_test)
add_test(NAME foc_control_test COMMAND foc_control_test)
