# Tuning

Most parameters are derived automatically by motor detection. Change them by hand only if
something below is out of spec.

## Current loop
Detection sets `kp = L·ωc` and `ki = R·ωc` (pole-zero cancellation) with ωc = 2π·`current_bandwidth_hz`.
- Default 1000 Hz bandwidth suits 20 kHz PWM. Do not exceed PWM/20.
- Audible hiss or current ripple: lower the bandwidth to 500–700 Hz.
- Sluggish torque response (rare): raise it to 1500 Hz.

## Observer
- `observer_gain` (1/s): convergence rate of the flux magnitude. 1000–3000 is typical. It must stay well
  below PWM frequency / 2.
- `pll_kp`/`pll_ki`: bandwidth of angle/speed smoothing. Higher values track hard acceleration better but
  pass more noise.
- The observer depends mostly on accurate **R** (at low speed) and **flux** (everywhere). A hot motor has
  ~30 % higher R. If handover fails when hot, raise `hall_observer_crossover_erpm`.

## Hall ↔ observer handover
- `hall_observer_crossover_erpm`: pick a speed where back-EMF ≳ 5 % of pack voltage:
  `erpm ≈ 0.05 · Vbus / flux · 60 / 2π`.
- Handover requires the observer to be locked for 50 ms and to agree with the halls within 20°.

## Throttle feel
- `throttle_expo` 0 is linear and 1 is cubic. 0.3–0.5 feels natural on scooters.
- `current_ramp_a_per_s` sets how quickly torque builds. Release is always 5× faster. Lower it (100 A/s) for
  gentler starts on hub motors with little rider weight.
- `CONTROL_MODE_SPEED` behaves like cruise control. `TORQUE` is the most predictable choice and the default.

## Limits: what each one protects
| Parameter | Protects |
|---|---|
| `motor_current_max` | motor windings / magnets (demagnetisation!) and FETs |
| `battery_current_max` | battery and BMS, fuse, wiring |
| `battery_current_regen_max` | battery charge rate (cold cells: keep low) |
| `abs_current_max` | hard trip, set ~1.5–2× `motor_current_max` |
| `battery_voltage_min` / `cutback_start` | cells: set ~3.0 V/cell and ~3.3 V/cell |
| `battery_voltage_max` | set to full pack voltage + 1 V. Regen fades out 2 V below this |
| `erpm_max` | bearing / magnet retention / back-EMF above the pack |
