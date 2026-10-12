# Safety

This firmware drives a vehicle that carries a person, powered by a lithium pack that can deliver
hundreds of amps. Read this whole page.

## Status

- The control code is verified by unit tests and a closed-loop motor simulation (`tests/`).
- The STM32G4 HAL (`src/hal/stm32g4`) is written against the reference manual but **has not been
  validated on hardware**. Pin mux, ADC trigger timing, dead time and break polarity must be
  checked with a scope during bring-up (docs/BRINGUP.md).
- Road legality is your responsibility. The default speed limit is 25 km/h (EU pedelec / many scooter
  rules). Local laws on power, speed and throttle differ.

## Protection layers (outermost first)

| Layer | Reaction time | Mechanism |
|---|---|---|
| Gate-driver VDS monitoring | ~4 µs | DRV8353 shuts down all FETs, pulls nFAULT → TIM1 BKIN |
| TIM1 break input | < 1 µs after BKIN | hardware clears MOE, outputs forced off, latched |
| Software overcurrent | 50 µs | every PWM sample vs `abs_current_max` |
| Overvoltage trip | 50 µs | bus voltage every sample |
| Current-sense plausibility | ms | offset range at boot, ia+ib+ic ≈ 0 while running |
| Derating (soft limits) | 1 ms | temperature, under/overvoltage, battery current, speed, eRPM |
| Sensor faults | 1–100 ms | hall codes, hall skips, encoder parity, observer flux, throttle range |
| Fast-loop supervision | 2 ms | medium loop checks the ADC ISR is running and within budget |
| Watchdog | 50 ms | kicked only if both ISR loops made progress |
| Debug freeze | immediate | TIM1 outputs go idle when halted at a breakpoint |

## Design decisions you should know about

**Coast on fault, don't brake.** A fault switches every FET off, so the vehicle freewheels. Sudden
regenerative braking at speed can throw the rider, and an active short can overheat the motor.

**Back-EMF above battery voltage.** If the motor spins fast enough that its back-EMF exceeds the pack
voltage (field weakening, or rolling downhill with a disconnected BMS), the body diodes rectify
energy into the bus even with every FET off. Mitigations:
- `fw_max_current` defaults to 0 (field weakening off)
- `erpm_max` limit
- the overvoltage trip turns the bridge off (this does not stop diode rectification!)
- **Hardware:** fit a TVS diode / brake chopper sized for this case, and bus capacitors rated for
  `BOARD_MAX_BUS_VOLTAGE`.

**Regen with an open BMS contactor.** If the BMS disconnects the pack, regen current has nowhere to go
and the bus voltage climbs fast. With CAN BMS messages present, regen goes to 0 when they stop. Without
a CAN BMS, rely on `battery_voltage_max` / `overvoltage_trip` and the TVS.

**Throttle interlocks.**
- At power-on the throttle output stays 0 until the throttle has read closed for 200 ms.
- Out-of-range voltage (broken ground → 0 V, or a signal shorted to 5 V) is a fault with zero output.
  An unprotected throttle shorted to 5 V means full power.
- After any brake application, drive stays off until the throttle is released.
- Latched faults can only be cleared while the throttle is released.

**Brake always wins.** The brake switch cuts drive torque within 1 ms, whatever the throttle does.
Regen fades to zero near standstill so the motor never drives backwards.

**Motor temperature sensor failure.** An open or shorted configured motor NTC raises `temp_sensor`,
sets both current allowances to zero, and stops PWM (including field weakening and motor detection).
Normal temperature derating resumes when valid readings return. Setting `motor.ntc_r25 = 0` explicitly
disables this sensor; an absent sensor is not treated as a failed configured sensor.

**Detection spins the motor.** `START_DETECTION` is only accepted while stopped and idle. **The wheel must
be off the ground.** Use test current `min(0.3 · motor_current_max, 15 A)`.

## Ride profiles and the safety envelope

Riders pick ECO/SPORT/RACE, which are fixed fractions of the factory limits. The CUSTOM profile is fully
tunable after an authenticated unlock in the app, but it can never exceed the hard envelope in
`board_config.h` (150 A phase / 160 A trip on SK2V-rev1). Requests above it are refused, not clamped.
Details: [PROFILES_AND_UNLOCK.md](PROFILES_AND_UNLOCK.md).

## Hardware requirements (non-negotiable for road use)

- Gate-driver inputs must have **pull-downs** so FETs stay off during reset and while firmware is not running.
- A hardware overcurrent path into TIM1 BKIN (gate driver VDS OCP and/or a shunt comparator).
- A pre-charge circuit for the bus capacitors, and a fuse sized for the wiring.
- FET and motor NTCs mounted where they actually measure the hot spot.
- A physical kill switch or key the rider can reach.
- Mechanical brakes that can stop the vehicle on their own. Regen is extra braking, not the brake.
