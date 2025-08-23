#ifndef __CIRCULARBUFFER__
#define __CIRCULARBUFFER__

#include <stdint.h>
#include <stddef.h>   // for size_t
#include <string.h>   // for memcpy
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif
struct CircularBuffer{    
    uint8_t* buffer;
    uint8_t head;
    uint8_t tail;
    uint8_t rows;
    uint8_t row_size;
};

void init_buffer(struct CircularBuffer* cb, uint8_t rows, uint8_t row_size);
void put_data(struct CircularBuffer *cb, const void* data);
void pull_data(struct CircularBuffer *cb, void* data);
bool check_empty(struct CircularBuffer* cb);
bool check_full(struct CircularBuffer* cb);
void free_buffer(struct CircularBuffer* cb);
void test_buffer(struct CircularBuffer* cb);

#ifdef __cplusplus
}
#endif

#endif