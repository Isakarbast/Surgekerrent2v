/*
 * board_config.h - compile-time hardware description.
 *
 * Select a board with -DBOARD_<NAME> (see CMakeLists.txt). Values here
 * describe the PCB and must match the schematic exactly: wrong shunt or
 * divider values produce wrong current limits, which on a vehicle means
 * burnt FETs or a fire. Measure, don't guess.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#if defined(BOARD_SK2V_REV1) || !defined(BOARD_NAME)

#ifndef BOARD_NAME
#define BOARD_NAME "SK2V-rev1"
#endif

/* ---- inverter ---- */
#define BOARD_PWM_FREQ_HZ           20000.0f
#define BOARD_DEADTIME_NS           300.0f
/* Duty written in the ADC ISR is loaded at the next counter underflow (half a
 * period after the sample) and held for one period, so the voltage is centred
 * one full period after the sample. The FOC output angle is advanced by
 * this many periods * omega. */
#define BOARD_PWM_OUTPUT_DELAY_PERIODS 1.0f
#define BOARD_MAX_BUS_VOLTAGE       84.0f   /* absolute max: FET/cap rating derated */
#define BOARD_MIN_BUS_VOLTAGE       18.0f
#define BOARD_MAX_PHASE_CURRENT     160.0f  /* hardware limit, peak A (must be < ADC range) */

/* ---- current sensing: 3 low-side shunts + gate-driver CSA ---- */
#define BOARD_SHUNT_OHM             0.0005f
#define BOARD_CSA_GAIN              20.0f   /* DRV8353 CSA gain (5/10/20/40) */
#define BOARD_CSA_INVERTED          1       /* DRV835x CSA output: Vref/2 - I*R*G */
#define BOARD_ADC_BITS              12
#define BOARD_ADC_VREF              3.3f
/* Current at which the ADC clips: (Vref/2) / (R * G) = 165 A here. */

/* Low-side on-time needed for a valid shunt sample: settling of CSA +
 * ADC sample window. Phases whose duty exceeds this are reconstructed. */
#define BOARD_MIN_LOWSIDE_SAMPLE_NS 1500.0f

/* ---- bus voltage divider ---- */
#define BOARD_VBUS_R_TOP            56000.0f
#define BOARD_VBUS_R_BOTTOM         2200.0f

/* ---- NTC thermistors (to GND, pull-up to Vref) ---- */
#define BOARD_NTC_FET_PULLUP        10000.0f
#define BOARD_NTC_FET_R25           10000.0f
#define BOARD_NTC_FET_BETA          3380.0f
#define BOARD_NTC_MOTOR_PULLUP      10000.0f
/* Motor NTC R25/beta are in runtime config: motors vary (10k/3950 is common). */

/* ---- throttle / brake analog inputs are 5 V hall sensors behind a divider ---- */
#define BOARD_THROTTLE_DIVIDER      (3.3f / 5.0f)  /* 5V sensor -> 3.3V ADC */

/* ---- SAFETY ENVELOPE ----
 * Hard ceilings that NOTHING can exceed: not the app, not an unlocked
 * CUSTOM profile, not a tampered flash config. Requests above these are
 * refused by the protocol; values loaded from flash are clamped to them.
 * Only a firmware rebuild can change them - set them from what the power
 * stage (FETs, copper, cooling, caps) is actually rated for, with margin.
 * safety_envelope.c has compile-time checks that they fit the current
 * sensor range. */
#define ENV_PHASE_CURRENT_MAX       150.0f  /* motor_current_max / regen [A peak] */
#define ENV_ABS_TRIP_MAX            160.0f  /* software overcurrent trip [A peak] */
#define ENV_BATTERY_CURRENT_MAX     100.0f  /* DC drive current [A] */
#define ENV_BATTERY_REGEN_MAX       40.0f   /* DC charge current [A] */
#define ENV_FW_CURRENT_MAX          50.0f   /* field-weakening |Id| [A] */
#define ENV_ERPM_MAX                60000.0f
#define ENV_RAMP_MAX_A_PER_S        1000.0f
#define ENV_SPEED_LIMIT_MAX_KMH     100.0f
#define ENV_CURRENT_BW_MAX_HZ       2500.0f /* current loop bandwidth, <= PWM / 8 */

/* ---- gate driver ---- */
#define BOARD_GATE_DRIVER_DRV8353   1

/* ---- optional position sensors ---- */
#define BOARD_HAS_HALL              1
#define BOARD_HAS_SPI_ENCODER       1   /* AS5047P footprint on SPI */

#else
#error "Unknown board. Add a section to board_config.h"
#endif

/* Derived */
#define BOARD_ADC_MAX_COUNTS        ((float)((1u << BOARD_ADC_BITS) - 1u))
#define BOARD_AMPS_PER_VOLT         (1.0f / (BOARD_SHUNT_OHM * BOARD_CSA_GAIN))
#define BOARD_CURRENT_SENSE_RANGE_A (0.5f * BOARD_ADC_VREF * BOARD_AMPS_PER_VOLT) /* ADC clips here */
#define BOARD_VBUS_SCALE            ((BOARD_VBUS_R_TOP + BOARD_VBUS_R_BOTTOM) / BOARD_VBUS_R_BOTTOM)
#define BOARD_FAST_LOOP_DT          (1.0f / BOARD_PWM_FREQ_HZ)
#define BOARD_MEDIUM_LOOP_HZ        1000.0f
#define BOARD_MEDIUM_LOOP_DT        (1.0f / BOARD_MEDIUM_LOOP_HZ)

#endif
