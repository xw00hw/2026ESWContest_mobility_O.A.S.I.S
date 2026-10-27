cmake_minimum_required(VERSION 3.20)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN "C:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin" CACHE PATH "ARM GNU toolchain bin directory")
set(CMAKE_C_COMPILER "${TOOLCHAIN}/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN}/arm-none-eabi-gcc.exe")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(fsr_mux_h723 C ASM)
option(OASIS_PRESSURE_ONLY "Build pressure acquisition with motor outputs permanently LOW" OFF)
option(OASIS_FALLBACK_CONTROL "Build explicit-session fallback tracking; boot PWM OFF" OFF)
if(OASIS_PRESSURE_ONLY AND OASIS_FALLBACK_CONTROL)
  message(FATAL_ERROR "Pressure-only and fallback-control modes are mutually exclusive")
endif()
set(CUBE "$ENV{USERPROFILE}/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0" CACHE PATH "STM32CubeH7 firmware root")
set(HAL "${CUBE}/Drivers/STM32H7xx_HAL_Driver")
set(DEV "${CUBE}/Drivers/CMSIS/Device/ST/STM32H7xx")
set(CORE "${CUBE}/Drivers/CMSIS/Include")
set(LDSCRIPT "${CUBE}/Projects/NUCLEO-H723ZG/Examples/UART/UART_TwoBoards_ComPolling/STM32CubeIDE/STM32H723ZGTX_FLASH.ld")
if(OASIS_PRESSURE_ONLY)
  set(CONTROL_SOURCES Core/Src/pressure_only_control.c)
elseif(OASIS_FALLBACK_CONTROL)
  set(CONTROL_SOURCES Core/Src/fallback_control.c Core/Src/fallback_pwm.c Core/Src/fallback_geometry.c Core/Src/fallback_tuning.c Core/Src/joint_math.c)
else()
  set(CONTROL_SOURCES Core/Src/joint_control.c Core/Src/joint_math.c)
endif()

add_executable(fsr_mux_h723
  Core/Src/main.c
  Core/Src/stm32h7xx_it.c
  Core/Src/stm32h7xx_hal_msp.c
  Core/Src/fsr_single.c
  ${CONTROL_SOURCES}
  "${DEV}/Source/Templates/system_stm32h7xx.c"
  "${DEV}/Source/Templates/gcc/startup_stm32h723xx.s"
  "${HAL}/Src/stm32h7xx_hal.c"
  "${HAL}/Src/stm32h7xx_hal_cortex.c"
  "${HAL}/Src/stm32h7xx_hal_gpio.c"
  "${HAL}/Src/stm32h7xx_hal_rcc.c"
  "${HAL}/Src/stm32h7xx_hal_rcc_ex.c"
  "${HAL}/Src/stm32h7xx_hal_pwr.c"
  "${HAL}/Src/stm32h7xx_hal_pwr_ex.c"
  "${HAL}/Src/stm32h7xx_hal_flash.c"
  "${HAL}/Src/stm32h7xx_hal_flash_ex.c"
  "${HAL}/Src/stm32h7xx_hal_uart.c"
  "${HAL}/Src/stm32h7xx_hal_uart_ex.c"
  "${HAL}/Src/stm32h7xx_hal_tim.c"
  "${HAL}/Src/stm32h7xx_hal_tim_ex.c"
  "${HAL}/Src/stm32h7xx_hal_adc.c"
  "${HAL}/Src/stm32h7xx_hal_adc_ex.c"
  "${HAL}/Src/stm32h7xx_hal_dma.c"
  "${HAL}/Src/stm32h7xx_hal_dma_ex.c"
)
target_include_directories(fsr_mux_h723 PRIVATE Core/Inc "${HAL}/Inc" "${HAL}/Inc/Legacy" "${DEV}/Include" "${CORE}")
target_compile_definitions(fsr_mux_h723 PRIVATE USE_HAL_DRIVER STM32H723xx)
target_compile_options(fsr_mux_h723 PRIVATE -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -O2 -ffunction-sections -fdata-sections -Wall)
target_link_options(fsr_mux_h723 PRIVATE -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -T${LDSCRIPT} --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/fsr_mux_h723.map)
target_link_libraries(fsr_mux_h723 PRIVATE m)
set_target_properties(fsr_mux_h723 PROPERTIES SUFFIX ".elf")
if(OASIS_PRESSURE_ONLY)
  target_compile_definitions(fsr_mux_h723 PRIVATE OASIS_PRESSURE_ONLY=1)
  target_compile_options(fsr_mux_h723 PRIVATE -g3)
  set_target_properties(fsr_mux_h723 PROPERTIES OUTPUT_NAME "pressure_only_h723")
elseif(OASIS_FALLBACK_CONTROL)
  target_compile_definitions(fsr_mux_h723 PRIVATE OASIS_FALLBACK_CONTROL=1)
  target_compile_options(fsr_mux_h723 PRIVATE -g3)
  set_target_properties(fsr_mux_h723 PROPERTIES OUTPUT_NAME "fallback_control_h723")
endif()
add_custom_command(TARGET fsr_mux_h723 POST_BUILD
  COMMAND "${TOOLCHAIN}/arm-none-eabi-objcopy.exe" -O binary
          "$<TARGET_FILE:fsr_mux_h723>" "$<TARGET_FILE_DIR:fsr_mux_h723>/$<TARGET_FILE_BASE_NAME:fsr_mux_h723>.bin"
  VERBATIM)
