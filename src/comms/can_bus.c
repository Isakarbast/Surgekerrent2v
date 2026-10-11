#include "can_bus.h"

#include <string.h>

#include "fault_manager.h"
#include "hal.h"
#include "motor_control.h"
#include "ride_profiles.h"
#include "vehicle_control.h"

can_state_t g_can;
static uint8_t s_node;
static uint32_t s_last_fast, s_last_slow;

void can_bus_init(uint8_t node_id)
{
    memset(&g_can, 0, sizeof(g_can));
    s_node = node_id;
}

static void put_i16(uint8_t *p, int16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)((uint16_t)v >> 8); }
static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static int16_t sat16(float x) { return (int16_t)(x > 32767.0f ? 32767 : (x < -32768.0f ? -32768 : x)); }
static int8_t sat8(float x) { return (int8_t)(x > 127.0f ? 127 : (x < -128.0f ? -128 : x)); }

static void send_status(uint32_t now)
{
    hal_can_frame_t f = {0};
    if (now - s_last_fast >= 20u) {
        s_last_fast = now;
        f.id = 0x100u + s_node;
        f.dlc = 8;
        put_u32(&f.data[0], (uint32_t)(int32_t)g_mc.erpm_filt);
        put_i16(&f.data[4], sat16(g_mc.iq_filt * 10.0f));
        put_i16(&f.data[6], sat16(g_mc.foc.modulation * 1000.0f));
        hal_can_send(&f);
    }
    if (now - s_last_slow >= 100u) {
        s_last_slow = now;
        f.id = 0x110u + s_node;
        f.dlc = 8;
        put_u16(&f.data[0], (uint16_t)(g_mc.vs.vbus_filt * 10.0f));
        put_i16(&f.data[2], sat16(g_mc.ibatt_filt * 10.0f));
        f.data[4] = (uint8_t)sat8(g_vehicle.ntc_fet.temp_c);
        f.data[5] = (uint8_t)sat8(g_vehicle.ntc_motor.temp_c);
        f.data[6] = (uint8_t)fault_first_active();
        f.data[7] = (uint8_t)g_mc.state;
        hal_can_send(&f);

        f.id = 0x120u + s_node;
        put_u16(&f.data[0], (uint16_t)(g_vehicle.speed_kmh < 0 ? 0 : g_vehicle.speed_kmh * 10.0f));
        put_u32(&f.data[2], (uint32_t)(g_vehicle.odometer_m / 10.0f));
        f.data[6] = g_vehicle.pas_level;
        f.data[7] = (uint8_t)(g_vehicle.brake.switch_active ? 1u : 0u);
        hal_can_send(&f);
    }
}

void can_bus_poll(void)
{
    uint32_t now = hal_millis();
    hal_can_frame_t f;
    while (hal_can_receive(&f)) {
        if (f.extended) continue;
        if (f.id == 0x300u + s_node && f.dlc >= 2) {
            if (f.data[0] <= 5) g_vehicle.pas_level = f.data[0];
            g_can.lights = (f.data[1] & 1u) != 0;
            hal_aux_output_set(g_can.lights);
            /* Displays repeat this frame continuously: act on changes only. */
            static uint8_t last_req = 0xFF;
            if (f.dlc >= 3 && f.data[2] != 0xFF && f.data[2] != last_req) {
                last_req = f.data[2];
                (void)profile_select((ride_profile_t)f.data[2]);
            }
        } else if (f.id == 0x310u + s_node && f.dlc >= 3) {
            g_can.bms_seen = true;
            g_can.bms_last_ms = now;
            g_can.bms_discharge_a = f.data[0];
            g_can.bms_charge_a = f.data[1];
            g_can.bms_stop = (f.data[2] & 1u) != 0;
        }
    }
    g_can.bms_timeout = g_can.bms_seen && (now - g_can.bms_last_ms > 500u);
    send_status(now);
}
