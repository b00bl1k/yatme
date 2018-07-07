cmake_minimum_required(VERSION 3.11)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CPU "-mcpu=cortex-m0 -mthumb")
set(PROJECT_NAME yatme)

set(SOURCES
    src/board.c
    src/ds18b20.c
    src/main.c
    src/nrf24l01p.c
    src/owbus.c
    src/radio.c
    libs/cayenne-lpp/cayenne_lpp.c)
link_directories(${CMAKE_CURRENT_SOURCE_DIR})

# library libopencm3 (stm32f0 only)
set(LIBOPENCM3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libopencm3)
add_custom_target(
    libopencm3
    COMMAND make IRQ_DEFN_FILES=include/libopencm3/stm32/f0/irq.json TARGETS=stm32/f0 all
    WORKING_DIRECTORY ${LIBOPENCM3_DIR}
)
link_directories(${LIBOPENCM3_DIR}/lib)
link_directories(${LIBOPENCM3_DIR}/lib/stm32/f0)
include_directories(${LIBOPENCM3_DIR}/include)

# Flash via OpenOCD
add_custom_target(
    flash
    COMMAND openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd.cfg -c "program ${PROJECT_NAME} verify reset exit"
    DEPENDS ${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build
)

# Reset via OpenOCD
add_custom_target(
    reset
    COMMAND openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd.cfg -c "init" -c "reset run" -c "exit"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build
)

add_definitions(-DSTM32F0)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/cayenne-lpp)

set(CMAKE_EXE_LINKER_FLAGS "-Tstm32f030x4.ld ${CPU} -nostartfiles -Wl,--gc-sections")

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} c_nano)
target_link_libraries(${PROJECT_NAME} nosys)
target_link_libraries(${PROJECT_NAME} opencm3_stm32f0)

set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS "${CPU} -g3 -ggdb -Wall -ffunction-sections -fdata-sections -O2")

add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND arm-none-eabi-size --format=berkeley ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} libopencm3)
