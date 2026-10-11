/*
 * Lock-free single-producer / single-consumer byte ring buffer.
 * Safe between one ISR and the main loop on Cortex-M (aligned 32-bit
 * index writes are atomic). Size must be a power of two.
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *buf;
    uint32_t mask;
    volatile uint32_t head; /* written by producer */
    volatile uint32_t tail; /* written by consumer */
} ring_buffer_t;

void rb_init(ring_buffer_t *rb, uint8_t *storage, uint32_t size_pow2);
bool rb_put(ring_buffer_t *rb, uint8_t byte);
bool rb_get(ring_buffer_t *rb, uint8_t *byte);
uint32_t rb_count(const ring_buffer_t *rb);
uint32_t rb_free(const ring_buffer_t *rb);

#endif
