cmake_minimum_required(VERSION 3.15)
project(fir_filter_cpp
    VERSION 1.0.0
    DESCRIPTION "Real-Time Embedded-Ready FIR Digital Low-Pass Filter in C++17"
    LANGUAGES CXX
)

# Standard C++17 requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization and warning flags
if(MSVC)
    add_compile_options(/W4 /permissive- /O2)
else()
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -O3
    )
endif()

# ==============================================================================
# 1. FIR Filter Library Target
# ==============================================================================
add_library(fir_filter
    src/fir_filter.cpp
    src/fir_coefficients.cpp
    src/fir_filter_q15.cpp
)

target_include_directories(fir_filter
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# ==============================================================================
# 2. Demonstration Executable Target
# ==============================================================================
add_executable(fir_demo
    src/main.cpp
)

target_link_libraries(fir_demo
    PRIVATE
        fir_filter
)

# ==============================================================================
# 3. Automated Test Suite Target
# ==============================================================================
enable_testing()

add_executable(fir_test
    tests/test_fir.cpp
)

target_include_directories(fir_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

target_link_libraries(fir_test
    PRIVATE
        fir_filter
)

add_test(NAME FIRFilterUnitTests COMMAND fir_test)

# ==============================================================================
# 4. Benchmark Executable Target
# ==============================================================================
add_executable(fir_benchmark
    benchmarks/benchmark.cpp
)

target_link_libraries(fir_benchmark
    PRIVATE
        fir_filter
)
