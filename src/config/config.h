/*
 * config.h - runtime configuration persisted in flash.
 *
 * Bump CONFIG_VERSION whenever the struct layout changes; storage will
 * refuse old blobs and fall back to defaults (the rider then has to
 * re-run motor detection, which is safer than using garbage values).
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_MAGIC   0x534B3256u /* "SK2V" */
#define CONFIG_VERSION 4u

typedef enum {
    SENSOR_MODE_SENSORLESS = 0,   /* flux observer only (needs push start or HFI-free open-loop start) */
    SENSOR_MODE_HALL,             /* hall only, interpolated */
    SENSOR_MODE_HALL_OBSERVER,    /* hall at low speed, observer above crossover (recommended) */
    SENSOR_MODE_ENCODER,          /* SPI absolute encoder */
} sensor_mode_t;

typedef enum {
    CONTROL_MODE_TORQUE = 0,      /* throttle -> phase current (feels natural, default) */
    CONTROL_MODE_SPEED,           /* throttle -> target speed (cruise-like) */
    CONTROL_MODE_DUTY,            /* throttle -> voltage (legacy "feel"), still current-limited */
} control_mode_t;

typedef enum {
    VEHICLE_ESCOOTER = 0,
    VEHICLE_EBIKE_PAS,            /* pedelec: motor only with pedalling, speed-capped */
    VEHICLE_EBIKE_THROTTLE,
} vehicle_type_t;

typedef struct {
    /* electrical */
    float rs_ohm;                 /* phase resistance (line-neutral) */
    float ld_h, lq_h;             /* d/q inductance */
    float flux_wb;                /* PM flux linkage (V*s/rad electrical) */
    uint8_t pole_pairs;
    int8_t direction;             /* +1 / -1: swaps phase B/C in software */
    float hall_angle_rad[8];      /* hall code -> sector centre angle; < 0 = invalid code */
    float encoder_offset_rad;
    /* thermal */
    float ntc_r25, ntc_beta;      /* motor NTC; r25 = 0 disables motor temp */
    float temp_motor_start_c, temp_motor_max_c;
} motor_config_t;

typedef struct {
    /* current loop (auto-computed from R/L by detection, editable) */
    float current_kp, current_ki;
    float current_bandwidth_hz;
    /* speed loop */
    float speed_kp, speed_ki;
    /* observer */
    float observer_gain;          /* magnitude convergence rate [1/s]; gamma = gain / flux^2 */
    float pll_kp, pll_ki;
    float hall_observer_crossover_erpm;
    float hall_observer_hysteresis_erpm;
    /* field weakening */
    float fw_max_current;         /* max negative Id [A]; 0 disables */
    float fw_start_duty;          /* modulation index where FW begins (0.85..0.95) */
    float max_modulation;         /* <= 0.95 of linear SVPWM range */
    sensor_mode_t sensor_mode;
    control_mode_t control_mode;
} foc_config_t;

typedef struct {
    float motor_current_max;      /* peak phase current, drive [A] */
    float motor_current_regen_max;/* peak phase current, braking [A] (positive number) */
    float battery_current_max;    /* DC current drawn from pack [A] */
    float battery_current_regen_max; /* DC current pushed into pack [A] (positive number) */
    float abs_current_max;        /* software overcurrent trip [A], > motor_current_max */
    float battery_voltage_max;    /* regen cut-off (full pack + margin) */
    float battery_voltage_min;    /* hard undervoltage cut */
    float battery_voltage_cutback_start; /* begin current foldback */
    float overvoltage_trip;       /* absolute trip [V] */
    float temp_fet_start_c, temp_fet_max_c;
    float erpm_max;               /* hard motor speed limit */
    float current_ramp_a_per_s;   /* torque slew limit */
} limits_config_t;

typedef struct {
    vehicle_type_t type;
    float wheel_diameter_m;
    float gear_ratio;             /* motor turns per wheel turn (1.0 for hub motors) */
    float speed_limit_kmh;        /* legal limit; 0 = off (private land only!) */
    float speed_limit_margin_kmh; /* current fades out over this band below the limit */
    uint8_t battery_cells_series;
    /* throttle (volts at the connector, 5 V hall throttle typical) */
    float throttle_v_min, throttle_v_max;       /* active range */
    float throttle_v_fault_low, throttle_v_fault_high; /* outside = wire fault */
    float throttle_expo;          /* 0 = linear, 1 = full cubic */
    /* Flags are uint8_t, not bool: this struct comes from flash and from the
     * app, and a bool holding anything but 0/1 is undefined behaviour. */
    uint8_t throttle_enabled;
    /* brake */
    uint8_t brake_switch_enabled;
    uint8_t brake_analog_enabled;
    float brake_v_min, brake_v_max;
    float regen_on_brake_fraction; /* fraction of regen max applied when brake switch pressed */
    /* pedal assist */
    uint8_t pas_magnets;
    uint8_t pas_levels;
    float pas_assist_current[6];  /* per level, [A] phase; index 0 = off */
    float pas_start_cadence_rpm;
    uint16_t pas_stop_timeout_ms;
    /* comms */
    uint8_t can_node_id;
    uint32_t can_bitrate;
} vehicle_config_t;

typedef enum {
    PROFILE_ECO = 0,
    PROFILE_SPORT,
    PROFILE_RACE,
    PROFILE_CUSTOM,              /* "VESC mode": user-tunable, needs authenticated unlock */
    PROFILE_COUNT
} ride_profile_t;

/* Everything a ride profile may override. Presets derive these from the
 * factory limits; CUSTOM stores its own copy. All are bounded by the
 * safety envelope in board_config.h. */
typedef struct {
    float motor_current_max;
    float motor_current_regen_max;
    float battery_current_max;
    float battery_current_regen_max;
    float abs_current_max;
    float erpm_max;
    float current_ramp_a_per_s;
    float fw_max_current;
    float speed_limit_kmh;        /* 0 = no limit (CUSTOM only) */
    float throttle_expo;
} profile_params_t;

typedef struct {
    uint8_t boot_profile;         /* ECO/SPORT/RACE; never boots into CUSTOM */
    uint8_t advanced_unlocked;    /* set only by a verified unlock response */
    uint16_t terms_version;       /* terms version the rider agreed to in the app */
    profile_params_t custom;
} ride_config_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    motor_config_t motor;
    foc_config_t foc;
    limits_config_t limits;
    vehicle_config_t vehicle;
    ride_config_t ride;
    uint32_t crc;                 /* CRC32 over all preceding bytes; must stay last */
} app_config_t;

extern app_config_t g_config;

void config_load_defaults(app_config_t *cfg);
/* Sanity-checks values against the board's hardware limits and clamps.
 * Returns false if something had to be changed. */
bool config_validate(app_config_t *cfg);

#endif
