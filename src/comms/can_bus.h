/*
 * can_bus.h - CAN 2.0B status broadcast + display/BMS input.
 *
 * TX (node id n):
 *   0x100+n  50 Hz  [erpm i32][iq*10 i16][modulation*1000 i16]
 *   0x110+n  10 Hz  [vbus*10 u16][ibatt*10 i16][t_fet i8][t_motor i8][fault u8][state u8]
 *   0x120+n  10 Hz  [speed_kmh*10 u16][odometer_m/10 u32][pas u8][flags u8]
 * RX:
 *   0x300+n  display: [pas_level u8][flags u8: bit0 lights][profile u8, 0xFF = no change]
 *            (CUSTOM is honoured only if unlocked and stopped, same rules as UART)
 *   0x310+n  BMS:     [discharge_limit_A u8][charge_limit_A u8][flags u8: bit0 stop]
 * If a BMS frame was ever received and then goes silent > 500 ms, the BMS
 * limits fall back to 0 regen / 50 % drive (BMS may have disconnected).
 */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool bms_seen;
    uint32_t bms_last_ms;
    float bms_discharge_a, bms_charge_a;
    bool bms_stop;
    bool bms_timeout;
    bool lights;
} can_state_t;

extern can_state_t g_can;

void can_bus_init(uint8_t node_id);
void can_bus_poll(void); /* main loop, ~1 kHz or faster */

#endif
