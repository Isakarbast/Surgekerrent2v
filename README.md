# Surgekurrent2v

# This repo will be closed later, and have a new one with a new shorter and simpler firmware to understand.
while still having everything this one has. so yes a shorter and more efficient firmware. and no ai in it but from debuging or reviewing the code, which will be comfirmed by either me isakarbast or another surge team memeber.

# Unfinished do not use
Field-oriented control (FOC) firmware for e-scooter and e-bike hub/mid motors.
Target: **STM32G474** (Cortex-M4F, 170 MHz) with 3 low-side shunts and a DRV8353 gate driver.
The control code is MCU-independent and runs on a PC against a simulated motor, and the full test suite runs that way.

> ⚠️ **Read [docs/SAFETY.md](docs/SAFETY.md) before powering anything.** This firmware
> switches tens of amps from a battery that can start fires, on a vehicle that
> carries a person. It has been verified in simulation only. It is **not yet
> validated on hardware**. Follow [docs/BRINGUP.md](docs/BRINGUP.md) step by step.

## Features

| Area | What's there |
|---|---|
| Current control | Synchronous-frame PI with anti-windup, back-EMF/cross-coupling feed-forward, d-axis voltage priority, PWM delay compensation, dead-time compensation |
| Modulation | SVPWM (min/max injection), overmodulation clamping |
| Current sensing | 3-shunt with boot offset calibration, high-duty phase reconstruction, Kirchhoff plausibility check, broken-amplifier detection |
| Position | Hall (interpolated, auto-mapped), sensorless flux observer (active-flux, works with salient motors) + PLL, AS5047P SPI encoder, hall→observer handover with lock and agreement checks |
| Motor detection | R (two-point), Ld/Lq (HF injection), flux linkage, hall table, encoder offset, auto current-loop gains |
| Torque | MTPA for IPM motors, optional field weakening |
| Vehicle | Torque/speed/duty throttle modes, pedal assist (PAS), regen braking (switch or analog lever), legal speed limit fade, throttle expo |
| Ride profiles | Locked ECO / SPORT / RACE presets, plus a CUSTOM "VESC mode" that is only unlockable through a signed challenge-response from your app ([docs/PROFILES_AND_UNLOCK.md](docs/PROFILES_AND_UNLOCK.md)) |
| Safety envelope | Hard ceilings (150 A phase, 160 A trip, 100 A battery...). Out-of-range settings are **refused**, checked at compile time against the current-sensor range, and re-clamped at boot |
| Protection | SW + HW overcurrent (TIM1 break), OV/UV, battery current limits (drive & regen), FET/motor thermal derating, stall, hall/encoder/observer/throttle wiring faults, gate-driver faults, loop-overrun, dual-ISR watchdog, BMS CAN limits |
| Comms | UART binary protocol with CRC (config, telemetry, detection, fault log), CAN 2.0B status + display/BMS input, Python CLI |
| Storage | A/B flash slots with sequence numbers and CRC32, safe if power is lost mid-write |

## Layout

```
src/
  app/            boot sequence, main loop, watchdog policy
  config/         board_config.h (hardware constants), config.c (runtime config + validation)
  foc/            transforms, SVPWM, PI, current loop, flux observer, PLL, field weakening/MTPA
  control/        motor_control (20 kHz fast loop), vehicle_control (1 kHz), motor_detect, ride_profiles
  drivers/
    current_sense/  3-shunt sensing + reconstruction
    voltage_sense/  bus voltage
    gate_driver/    generic interface, DRV8353 (SPI), simple enable-pin drivers
    hall/ encoder/  position sensors
    throttle/ brake/ pas/ temperature/
  safety/         fault manager, current derating, safety envelope
  comms/          UART protocol, CAN, telemetry, unlock (HMAC challenge-response)
  storage/        power-loss-safe config storage
  hal/            hal.h interface; stm32g4/ (register-level port); sim/ (PC simulator + motor model)
tests/            unit tests + full-system simulation tests
tools/            sk2v_cli.py (PC tool), gen_unlock_key.py, fetch_cmsis.sh
secrets/          unlock key (generated, git-ignored)
docs/             SAFETY, BRINGUP, TUNING, ARCHITECTURE, PROFILES_AND_UNLOCK
ld/ cmake/        linker script, ARM toolchain file
```

## Build

**Host tests (no hardware needed):**
```bash
cmake -B build-host && cmake --build build-host -j && ctest --test-dir build-host --output-on-failure
```

**Firmware** (needs `arm-none-eabi-gcc` 12+):
```bash
./tools/fetch_cmsis.sh
./tools/gen_unlock_key.py        # once per product line; without it a PUBLIC dev key is used
cmake -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build-fw -j
# flash: st-flash write build-fw/sk2v_fw.bin 0x08000000   (or openocd / STM32CubeProgrammer)
```

## First use

1. Power up and check that `ping` and `telemetry` work (`tools/sk2v_cli.py -p /dev/ttyUSB0 ...`).
2. Set limits and battery voltages for **your** pack in the config (defaults are a 13S / 48 V scooter).
3. Lift the wheel and run `detect`. It measures R, L and flux, maps the halls, then saves.
4. Test on a stand at low current limits, then increase them step by step. See [docs/TUNING.md](docs/TUNING.md).

The LED shows status: solid means ready, a 1 Hz blink means no motor parameters yet, and N quick blinks means fault number N.

## Porting to another board or MCU

- New board with the same MCU: add a section to `src/config/board_config.h` and adjust `src/hal/stm32g4/board_pins.h`.
- New MCU: implement `src/hal/hal.h`. Nothing else touches hardware.
