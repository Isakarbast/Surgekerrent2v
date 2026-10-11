#include "as5047p.h"

#include "hal.h"
#include "util_math.h"

#define AS5047_REG_ANGLECOM 0x3FFFu
#define AS5047_READ         0x4000u
#define AS5047_EF           0x4000u

static uint16_t parity_even(uint16_t v)
{
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1u;
}

static uint16_t cmd_frame(uint16_t addr)
{
    uint16_t f = AS5047_READ | (addr & 0x3FFFu);
    return (uint16_t)(f | (parity_even(f) << 15));
}

void as5047p_init(as5047p_t *e, uint8_t pole_pairs, int8_t direction, float offset)
{
    e->pole_pairs = pole_pairs;
    e->direction = direction;
    e->offset = offset;
    e->mech_angle = 0.0f;
    e->elec_angle = 0.0f;
    e->error_count = 0;
    e->fault = false;
    pll_init(&e->pll, 2000.0f, 400000.0f);
    (void)hal_spi_transfer16(HAL_SPI_DEV_ENCODER, cmd_frame(AS5047_REG_ANGLECOM));
}

bool as5047p_update(as5047p_t *e, float dt)
{
    uint16_t rx = hal_spi_transfer16(HAL_SPI_DEV_ENCODER, cmd_frame(AS5047_REG_ANGLECOM));
    if (parity_even(rx) != 0u || (rx & AS5047_EF)) {
        if (e->error_count < 0xFFFF) e->error_count++;
        e->fault = e->error_count > 100;
        return false;
    }
    if (e->error_count) e->error_count--;

    float mech = (float)(rx & 0x3FFFu) * (M_2PI_F / 16384.0f);
    if (e->direction < 0) mech = M_2PI_F - mech;
    e->mech_angle = mech;
    e->elec_angle = wrap_2pi(mech * (float)e->pole_pairs + e->offset);
    pll_update(&e->pll, e->elec_angle, dt);
    return true;
}
