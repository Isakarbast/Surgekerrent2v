#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 0

typedef struct __attribute__((packed)) {
    uint32_t time_ms;
    uint8_t mc_state;
    uint8_t angle_src;
    uint8_t pas_level;
    uint8_t flags;           /* bit0 brake, bit1 throttle armed, bit2 pedalling, bit3 outputs on, bit4 advanced unlocked */
    uint32_t faults;
    float vbus;
    float ibatt;
    float id, iq;
    float iq_target;
    float erpm;
    float speed_kmh;
    float modulation;
    float temp_fet, temp_motor;
    float throttle;
    float odometer_km;
    float trip_wh;
    float iq_limit_drive, iq_limit_regen;
    uint16_t loop_us_max;
    uint8_t hall_code;
    uint8_t profile;         /* ride_profile_t */
} telemetry_t;

void telemetry_fill(telemetry_t *t);

#endif
