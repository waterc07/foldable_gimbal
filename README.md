# Fodable Gimbal

Foldable gimbal test-bench firmware based on the G-Master/GSRL project layout.

## Hardware

- No yaw axis.
- Pitch is a Z-shaped foldable mechanism driven by two DM4310 motors.
- Lower DM4310: CAN ID `1`, master ID `4`.
- Upper DM4310: CAN ID `2`, master ID `5`.
- Friction wheels: two M3508 motors.
- All motors are connected to `CAN1`.
- DR16 receiver is connected to the C board UART.

## Structure

```text
.
|-- Chariot/      # Application controller and parameters
|-- Task/         # FreeRTOS tasks and ISR forwarding
|-- GSRL/         # G-Master standard library and device drivers
|-- CubeMX_BSP/   # STM32CubeMX BSP, startup, middleware
|-- CMakeLists.txt
`-- README.md
```

## Key Files

- `Chariot/inc/para_gimbal.hpp`: centralized motor IDs, CAN bus choices, PID gains, limits, polarity, IMU and DR16 settings.
- `Chariot/src/crt_gimbal.cpp`: foldable pitch control, friction wheel control, DR16 mode handling, CAN/UART initialization.
- `Task/src/tsk_gimbal.cpp`: PID, motor, IMU and `Gimbal` object assembly.
- `Task/src/tsk_isr.cpp`: DR16 UART callback and CAN RX callback forwarding.
- `Task/src/tsk_usb.cpp`: USB telemetry task.

## Current Control Mapping

- DR16 right switch down: no-force mode, pitch and friction motors output zero.
- DR16 right switch middle/up: manual pitch control.
- DR16 right stick Y: pitch target increment.
- DR16 left switch middle to up event: toggle friction wheels.

## Build

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Validated output: `build/Debug/Fodable_gimbal.elf`.
