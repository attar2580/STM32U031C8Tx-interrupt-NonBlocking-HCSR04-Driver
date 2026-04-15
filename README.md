# STM32 Ultrasonic 3-Sensor Firmware (HC-SR04)

Firmware project for reading three HC-SR04 ultrasonic sensors on STM32 using
HAL + CMake, with UART serial reporting.

## Hardware Target

- Board: `NUCLEO-U083RC`
- MCU: `STM32U083RCT6` (`STM32U083RCTx` family target)
- Core clock: `HSI 16 MHz`
- Timer used for microsecond timing: `TIM3` (configured for ~1 MHz tick)
- Firmware package: `STM32Cube FW_U0 V1.3.0`

## Sensor Pin Mapping

Defined in `Core/Inc/ultrasonic.h`.

- Sensor 1
  - TRIG: `PA10` (D2)
  - ECHO: `PB3`  (D3 / EXTI3)
- Sensor 2
  - TRIG: `PB4`  (D5)
  - ECHO: `PB5`  (D4 / EXTI5)
- Sensor 3
  - TRIG: `PA8`  (D7)
  - ECHO: `PB10` (D6 / EXTI10)

All sensor grounds must be common with the board ground.

## Firmware Behavior

- Runtime loop takes measurements sequentially for S1 -> S2 -> S3.
- Trigger pulse is generated per sensor, echo is measured with microsecond
  timing and converted to millimeters.
- Serial output format:

```text
S1=<value|OOR> mm, S2=<value|OOR> mm, S3=<value|OOR> mm
```

- `OOR` means out-of-range/invalid measurement (distance not valid under the
  configured max range policy).
- Maximum valid display range is bounded by `ULTRASONIC_ERROR_DISTANCE_MM` in
  `Core/Inc/ultrasonic.h` (currently `4000.0F` mm).

## Project Structure

- `Core/Inc` and `Core/Src`: application and driver code
- `Drivers`: STM32 HAL/CMSIS/BSP files
- `cmake`: toolchain and CubeMX-generated CMake fragments
- `HCSR04_UO83RC.ioc`: STM32CubeMX project configuration
- `CMakeLists.txt`: top-level build entry

## Build Environment

Recommended setup:

- CMake `>= 3.22`
- `arm-none-eabi-gcc` toolchain
- Ninja (or compatible CMake generator)
- STM32CubeIDE / STM32CubeMX (for `.ioc` regeneration if needed)
- ST-LINK tools for flashing (`STM32CubeProgrammer` or IDE flash pipeline)

## Build Instructions

From repository root:

```powershell
cmake --preset Debug
cmake --build "build/Debug"
```

Output ELF is generated at:

- `build/Debug/HCSR04_UO83RC.elf`

## Flash and Monitor

1. Flash the ELF with your preferred ST-LINK workflow.
2. Open VCP serial monitor at:
   - Baud: `115200`
   - Data bits: `8`
   - Parity: `None`
   - Stop bits: `1`
3. Read continuous distance lines in the format shown above.

## Notes

- EXTI callbacks are still present in the ultrasonic module.
- Current application path uses blocking measurement calls for robust runtime
  behavior on this hardware setup.
- Keep user code updates inside CubeMX user sections when modifying generated
  files.
