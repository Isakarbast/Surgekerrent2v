# Hardware bring-up

Do these steps **in order**. Use a **current-limited bench supply** (start at 24 V / 1 A) until step 8.
Never use a battery before the break input and current sensing have been checked.

## 0. Before power
- [ ] Check `board_config.h` against the schematic: shunt value, CSA gain, divider resistors, NTC parts.
- [ ] Check `board_pins.h` against the schematic, pin by pin.
- [ ] Gate-driver inputs have pull-downs. Measure resistance from each INHx/INLx to GND.

## 1. MCU alive (motor disconnected, gate driver EN held low)
- [ ] Flash the firmware, then `sk2v_cli.py ping` → `SK2V firmware vX.Y.Z`.
- [ ] `telemetry`: Vbus within 2 % of a multimeter reading. If not, fix `BOARD_VBUS_R_*`.
- [ ] FET temperature reads room temperature. Throttle reads ~0.8 V closed, ~4.2 V open.

## 2. PWM and dead time (motor disconnected, **FET power stage unpowered**, gate driver logic powered)
- [ ] Put a scope on INHA/INLA. Temporarily force RUN via the debugger (or a test build). Check:
  - frequency = `BOARD_PWM_FREQ_HZ`, center-aligned
  - **dead time ≥ `BOARD_DEADTIME_NS` on both edges, and high/low never overlap**
- [ ] Halt in the debugger: all gate inputs go low (debug freeze).

## 3. Break input
- [ ] Pull BKIN (PB12) low while PWM is running: outputs stop in < 1 µs and `overcurrent_hw` latches.
- [ ] `clear` refuses while the throttle is open and works once it is closed.

## 4. ADC trigger timing
- [ ] Scope a spare GPIO toggled at the start of `ADC1_2_IRQHandler` against INLA. The sample (ISR start
      minus conversion time ≈ 1 µs) must fall inside the low-side on-time, near its middle.
- [ ] `loop_us_max` in telemetry must be < 40 µs at 20 kHz (80 % budget). Otherwise `loop_overrun` trips.

## 5. Current-sense offsets (power stage on, motor disconnected)
- [ ] After boot the state goes calibrating → idle with no `current_sense` fault.
- [ ] Offsets (read with the debugger: `g_mc.cs.offset`) all within ±3 % of 2048.

## 6. Current-sense polarity and gain (motor connected, 24 V / 3 A supply)
- [ ] Set `motor_current_max = 5`, `abs_current_max = 15`.
- [ ] Run `detect` with the wheel off the ground. If R/L come out wildly wrong or `current_sense` trips,
      the CSA polarity (`BOARD_CSA_INVERTED`) or gain is wrong.
- [ ] Compare the detected R with a 4-wire measurement (line-to-line / 2).

## 7. Closed loop on the stand
- [ ] Hall mode first (`sensor_mode = HALL`). Throttle gently: smooth, quiet rotation, `iq` follows
      `iq_target`.
- [ ] Switch to `HALL_OBSERVER`. Above the crossover `angle_src` becomes `observer` with no
      audible change. If it never hands over, the observer and halls disagree (bad flux or R).
- [ ] Brake lever: iq goes negative, the wheel slows, `ibatt` is negative.

## 8. Battery, still on the stand
- [ ] Connect through a fuse and pre-charge. Raise limits in steps (10 A → 20 A → target) and watch FET
      temperature at each step with sustained load (hold the wheel with a brake).
- [ ] Test brownout: drop the supply until undervoltage foldback, then the undervoltage fault. Drive must fade,
      not cut abruptly.

## 9. Road
- [ ] Empty car park, low limits, helmet. Test brake cut-off, the throttle-release interlock,
      and pulling the throttle connector while riding (must fault to zero, not full throttle).
