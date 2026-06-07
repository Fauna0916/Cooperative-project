# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Intelligent line-following competition car based on STM32H7 (400 MHz). Differential-drive chassis with encoder odometry, BNO080 IMU, 8-channel gray sensor, dual millimeter-wave radar, HCSR04 ultrasonic, ST7735 128×160 TFT, LoRa telemetry, and DS3231 RTC.

## Build System

- **IDE**: Keil MDK (ARM Compiler 6)
- **Project file**: `MDK-ARM/TDPS.uvprojx`
- **CubeMX**: `TDPS.ioc` — regenerate `Core/` when pinout changes
- **Custom code**: `Code/Inc/` (headers) and `Code/Src/` (sources)
- **HAL code**: `Core/Inc/` and `Core/Src/` — CubeMX-generated, do not edit outside `USER CODE` blocks
- **Compiler**: GNU99 for C sources. Use `__NOP()` for tiny delays, not `HAL_Delay()`.

## Architecture — Data Flow

```
1 kHz TIM7 ISR (htim7)
  └─ Control_Update()          ← the central real-time tick
       ├─ Encoder_Update()     ← reads TIM2/TIM3 encoder counts
       ├─ Odometry_Update()    ← fuses encoders + BNO080 IMU → (x, y, θ)
       ├─ PID velocity loop    ← per-wheel speed control (pid_left/right_motor)
       └─ Motor_SetSpeed()     ← PWM outputs to DRV8876 drivers

Main loop (best-effort, ~1 kHz polling)
  ├─ GraySensor_Update()       ← 8-ch multiplexed comparator read, 1 kHz
  ├─ RobotTask_Update(gray)    ← mission FSM: line follow → junction → radar → search
  ├─ HCSR04_TriggerUpdate()    ← ultrasonic trigger every 60 ms
  ├─ BNO080_Update()           ← I2C1 DMA read of quaternion, updates yaw
  ├─ Radar_Update()            ← 8 Hz UART DMA parse (only when is_scanning)
  ├─ LoRa_ProcessArchTrigger() ← edge-triggered, one LoRa msg per arch passage
  └─ TDPS_DisplayDemo_Task()   ← TFT page rendering at ~10 Hz
```

## Key Modules

### Control Pipeline (`control.c`)
The central 1 kHz servo loop. Modes: `CTRL_STOP`, `CTRL_LINE_FOLLOWING`, `CTRL_IMU_HEADING`, `CTRL_SPEED_MODE`. The line-following mode takes a base speed + vision error and applies a PID to produce angular velocity, then inverse-kinematics to per-wheel RPM targets, then a per-wheel velocity PID to PWM. Velocity ramping (`MAX_ACCEL`) prevents jerk.

### Gray Sensor (`gray_sensor.c`)
8 reflective sensors multiplexed through a CD4051 (3 address lines AD0-AD2, 1 comparator output). `GraySensor_Update()` does blob analysis: finds connected components, picks the blob nearest to the last known position, applies rate-of-change limiting (±25/frame), and detects junctions when ≥2 direction branches are visible. The 16-channel variant in `gray_sensor_16ch.c` extends this with a 4th address line.

### Robot Task (`robot_task.c`)
Mission state machine with states: `MISSION_IDLE`, `MISSION_RUNNING`, `MISSION_FAULT_LOST_LINE`, `MISSION_FINISHED`. During `MISSION_RUNNING`, it tracks markers via odometry distance, handles junctions through a 20-frame sliding-window voting mechanism, and delegates to radar avoidance in the task-3 zone (MARKER_1_4). Turn execution locks on IMU yaw angle (`JUNC_UNLOCK_ANGLE = PI/3`).

### Radar (`radar.c`)
Two mm-wave radar modules on UART4 (left) and UART8 (right). DMA circular reception into DTCM (`0x24040000`). Frame format: `0x6E | state | dist_L | dist_H | 0x62`. Manual DMA pointer tracking (no HT/TC interrupts) to avoid STM32H7 DMA double-buffer bugs. A 30-sample sliding window filters detections; voting threshold determines avoidance direction.

### LoRa (`lora_tx.c`)
UART5 IT-based transmission. Frame: `[AddrH, AddrL, Channel, payload...]`. Team: "6" / "404 not found". `LoRa_ProcessArchTrigger()` implements rising-edge detection on HCSR04 arch sensor — sends exactly one message per passage, re-arms on falling edge, uses DS3231 RTC (I2C2) for wall-clock time.

### Odometry (`odometry.c`)
Fuses encoder ticks (1040 PPR, 45mm wheels, 148mm track) with BNO080 IMU absolute yaw. IMU provides primary heading with spike rejection (>0.5 rad → fallback to encoder relative). X/Y integration uses midpoint Runge-Kutta with `x = -dist*sin(θ)`, `y = dist*cos(θ)` convention.

### TFT Display (`tdps_display_demo.c`)
ST7735 128×160 over SPI1. Two pages with auto-switch state machine:
- **LINE page** (default): 8-block sensor bar, raw hex, error, flag, odometry distance/speed
- **RADAR page**: activates when `is_scanning` ≥ 500 ms, shows left/right distance bars, target indicators, vote counts, and decision direction. Holds 2 s after scanning stops.

## Pin Map (notable)

| Function       | Pin  | Peripheral |
|----------------|------|------------|
| Gray AD0-AD2   | PE7, PC5, PC9 | GPIO out |
| Gray OUT       | PE9  | GPIO in |
| Key1           | PC13 | EXTI15_10 (rising) |
| HCSR04 Trig    | PB14 | GPIO out |
| HCSR04 Echo    | PB12 | EXTI15_10 (both edges) |
| BNO080 INT     | PB5  | EXTI (falling) |
| BNO080 I2C     | PB6/7 | I2C1 |
| DS3231 I2C     | PB10/11 | I2C2 |
| Radar Left     | —    | UART8 |
| Radar Right    | —    | UART4 |
| LoRa           | —    | UART5 |
| TFT SPI        | —    | SPI1 |
| Motor PWM L    | PD12/13 | TIM4 |
| Motor PWM R    | PD14/15 | TIM4 |
| Encoder L      | PA0/1 | TIM2 |
| Encoder R      | PA6/7 | TIM3 |

## Concurrency & ISR Rules

- **TIM7** (1 kHz, priority 0,0): runs `Control_Update()` — keep short. Encoder reads, PID math, motor PWM writes only.
- **EXTI15_10** (priority 3,0): HCSR04 echo + Key1 button. Only sets flags/ticks, no blocking.
- **I2C1 events** (priority 0,0): BNO080 DMA completion. Transfer ~32 bytes, process in main loop.
- **UART DMA**: radar RX uses manual `NDTR` pointer tracking — never enable HT/TC interrupts on those streams (STM32H7 errata: double-buffer DMA can corrupt when interrupts fire on half-transfer).
- **Main loop**: non-critical work — sensor reads, display updates, LoRa TX. Must not block longer than ~1 ms.
- All `HAL_UART_Transmit_IT` and `HAL_I2C_Mem_Read` must be guarded against re-entry (check `State != BUSY_TX` / use flags like `is_lora_busy`).

## Adding New Code

1. Header → `Code/Inc/`, source → `Code/Src/`
2. Include the header in `Core/Src/main.c` inside `USER CODE Includes`
3. Add init call in `main()` inside `USER CODE 2` (after peripheral init, before `while(1)`)
4. If you create a `.c` file, add it to the Keil project under the `Code` group (edit `MDK-ARM/TDPS.uvprojx` or use the IDE)

## Git Workflow

- Base branch: `dev`
- Feature branches: `feature/<name>`
- PR to `dev` with reviewer
- Commit messages use conventional commits (`feat:`, `fix:`, etc.)
