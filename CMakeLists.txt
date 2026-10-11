cmake_minimum_required(VERSION 3.20)

# Firmware build:  cmake -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
# Host tests:      cmake -B build-host && cmake --build build-host && ctest --test-dir build-host
project(Surgekurrent2v C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SK2V_BOARD "SK2V_REV1" CACHE STRING "Board definition from src/config/board_config.h")

set(CORE_SOURCES
    src/app/app.c
    src/config/config.c
    src/foc/foc_math.c
    src/foc/pi_controller.c
    src/foc/foc_controller.c
    src/foc/pll.c
    src/foc/flux_observer.c
    src/foc/field_weakening.c
    src/control/motor_control.c
    src/control/motor_detect.c
    src/control/vehicle_control.c
    src/drivers/current_sense/current_sense.c
    src/drivers/voltage_sense/voltage_sense.c
    src/drivers/temperature/ntc.c
    src/drivers/hall/hall_sensor.c
    src/drivers/encoder/as5047p.c
    src/drivers/throttle/throttle.c
    src/drivers/brake/brake.c
    src/drivers/pas/pas.c
    src/drivers/gate_driver/drv8353.c
    src/drivers/gate_driver/gate_driver_simple.c
    src/safety/fault_manager.c
    src/safety/current_limits.c
    src/safety/safety_envelope.c
    src/control/ride_profiles.c
    src/comms/unlock.c
    src/util/sha256.c
    src/comms/uart_protocol.c
    src/comms/can_bus.c
    src/comms/telemetry.c
    src/storage/config_storage.c
    src/util/util_math.c
    src/util/crc.c
    src/util/ring_buffer.c
)

set(CORE_INCLUDES
    secrets src/app src/config src/foc src/control src/safety src/comms src/storage src/util src/hal
    src/drivers/current_sense src/drivers/voltage_sense src/drivers/temperature
    src/drivers/hall src/drivers/encoder src/drivers/throttle src/drivers/brake
    src/drivers/pas src/drivers/gate_driver
)

set(WARN_FLAGS -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter)

if(CMAKE_SYSTEM_NAME STREQUAL "Generic")
    # ------------------------------------------------------------ firmware
    set(CMSIS_DIR ${CMAKE_SOURCE_DIR}/third_party/cmsis)
    if(NOT EXISTS ${CMSIS_DIR}/device_g4/Include/stm32g4xx.h)
        message(FATAL_ERROR "CMSIS headers missing. Run tools/fetch_cmsis.sh first.")
    endif()

    add_executable(sk2v_fw.elf
        src/app/main.c
        ${CORE_SOURCES}
        src/hal/stm32g4/hal_stm32g4.c
        src/hal/stm32g4/startup_stm32g474.c
    )
    target_include_directories(sk2v_fw.elf PRIVATE
        ${CORE_INCLUDES}
        src/hal/stm32g4
        ${CMSIS_DIR}/core/Include
        ${CMSIS_DIR}/device_g4/Include
    )
    target_compile_definitions(sk2v_fw.elf PRIVATE STM32G474xx BOARD_${SK2V_BOARD})
    target_compile_options(sk2v_fw.elf PRIVATE ${WARN_FLAGS})
    target_link_options(sk2v_fw.elf PRIVATE
        -T${CMAKE_SOURCE_DIR}/ld/stm32g474re.ld
        -Wl,-Map=sk2v_fw.map -Wl,--print-memory-usage)
    add_custom_command(TARGET sk2v_fw.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary sk2v_fw.elf sk2v_fw.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex sk2v_fw.elf sk2v_fw.hex
        COMMAND ${CMAKE_SIZE} sk2v_fw.elf)
else()
    # ------------------------------------------------------------ host / sim
    enable_testing()
    add_library(sk2v_core STATIC ${CORE_SOURCES} src/hal/sim/sim_hal.c)
    target_include_directories(sk2v_core PUBLIC ${CORE_INCLUDES} src/hal/sim)
    target_compile_definitions(sk2v_core PUBLIC BOARD_${SK2V_BOARD} SK2V_SIM=1)
    target_compile_options(sk2v_core PRIVATE ${WARN_FLAGS} -Wno-double-promotion)
    target_link_libraries(sk2v_core PUBLIC m)

    add_executable(test_units tests/test_units.c)
    target_link_libraries(test_units PRIVATE sk2v_core)
    add_test(NAME units COMMAND test_units)

    add_executable(test_system tests/test_system.c)
    target_link_libraries(test_system PRIVATE sk2v_core)
    add_test(NAME system COMMAND test_system)
endif()
