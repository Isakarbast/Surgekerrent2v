#include "ring_buffer.h"

void rb_init(ring_buffer_t *rb, uint8_t *storage, uint32_t size_pow2)
{
    rb->buf = storage;
    rb->mask = size_pow2 - 1u;
    rb->head = 0;
    rb->tail = 0;
}

uint32_t rb_count(const ring_buffer_t *rb) { return rb->head - rb->tail; }
uint32_t rb_free(const ring_buffer_t *rb) { return (rb->mask + 1u) - rb_count(rb); }

bool rb_put(ring_buffer_t *rb, uint8_t byte)
{
    if (rb_free(rb) == 0) return false;
    rb->buf[rb->head & rb->mask] = byte;
    rb->head++;
    return true;
}

bool rb_get(ring_buffer_t *rb, uint8_t *byte)
{
    if (rb_count(rb) == 0) return false;
    *byte = rb->buf[rb->tail & rb->mask];
    rb->tail++;
    return true;
}
