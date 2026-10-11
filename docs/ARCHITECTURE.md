# Architecture

## Execution contexts

```
priority   context          rate    work
  0        ADC1_2 IRQ       20 kHz  mc_fast_loop(): sense → protect → angle → FOC → PWM
  4        SysTick          1 kHz   vehicle_medium_loop(): inputs → limits → torque cmd; slow protections
  6        USART2 IRQ       bytes   ring buffers only
  -        main loop        free    protocol, CAN, LED, watchdog kick
```

The fast loop is the only code that writes the PWM timer. Other contexts communicate through
single-word volatile fields (`iq_cmd`, `run_req`, `detect_req`), which are atomic on Cortex-M.

## Fast loop timing (TIM1 center-aligned, 20 kHz)

```
CNT   /\        /\        /\
     /  \      /  \      /  \
    /    \    /    \    /    \
         ^         ^            ADC trigger at peak (all low sides on → shunts see phase current)
          [ISR k]
              v                 new CCRs load at underflow (½ period later)
```
- Phase-current sample k reflects the duty written by ISR k-1 → `duty_applied` is used for reconstruction.
- The output angle is advanced by `BOARD_PWM_OUTPUT_DELAY_PERIODS · ω · T`.
- The observer integrates the average of the previous two commanded voltage vectors, which is what the
  bridge actually applied between two samples.

## Data flow

```
             ┌──────────── vehicle_control (1 kHz) ────────────┐
throttle ──► │ throttle/brake/PAS → request → limits (derate)  │── iq_cmd ─┐
brake ─────► │ slow protections → fault_manager                │           │
             └─────────────────────────────────────────────────┘           ▼
             ┌──────────── motor_control (20 kHz) ─────────────────────────────┐
ADC ───────► │ current_sense → fast protections → hall/encoder/observer        │
             │ → angle select → field weakening/MTPA → foc_controller → SVPWM  │──► TIM1
             └─────────────────────────────────────────────────────────────────┘
```

## Testing strategy

`src/hal/sim` implements the full HAL with a dq-frame PMSM model. The model includes dead time,
the half-period PWM update delay, battery resistance and 120° hall sensors. `tests/test_system.c`
runs the actual firmware against it:
boot → current-offset calibration → motor detection over the UART protocol → ride to the speed limit
with observer handover → regen braking → hardware-break fault and clearing → undervoltage foldback →
config persistence across reboot.

What simulation cannot catch: pin mux, ADC trigger alignment, analog noise, EMI, thermal behaviour,
and real inverter non-linearities. Those are covered in docs/BRINGUP.md.
