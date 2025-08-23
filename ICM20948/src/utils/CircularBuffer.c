#include "CircularBuffer.h"
#include <stdio.h>


void init_buffer(struct CircularBuffer* cb, uint8_t rows, uint8_t row_size){
    cb->buffer=(char*)malloc(rows*row_size);
    cb->rows=rows;
    cb->row_size=row_size;
    cb->head=0x00;
    cb->tail=0x00;
    printf("created cb->buffer of size{%u}, cb->head={%hhu}, cb->tail={%hhu}\n",rows * row_size, cb->head, cb->tail);
}

void put_data(struct CircularBuffer* cb, const void* data){
    printf("putting data at cb->head: {%hhu}\n",cb->head); 
    memcpy((cb->buffer+cb->head*cb->row_size),data,cb->row_size);
    cb->head=(cb->head+1)%cb->rows;
}
void pull_data(struct CircularBuffer *cb, void* data){
    printf("pulling data at cb->tail: {%hhu}\n",cb->tail);
    memcpy(data,(cb->buffer+cb->tail*cb->row_size),cb->row_size);
    cb->tail=(cb->tail+1)%cb->rows;
}

bool check_empty(struct CircularBuffer* cb){
   return cb->tail==cb->head;
}
bool check_full(struct CircularBuffer* cb){
    return (cb->head+1)%cb->rows==cb->tail;
}
void free_buffer(struct CircularBuffer* cb) {
    free(cb->buffer);
    cb->buffer = NULL;
}
void test_circular_buffer(struct CircularBuffer* cb) {

    init_buffer(cb, 5, sizeof(float[4])); // 5 rows, each row = float[4]

    printf("Initialized buffer: rows=%u, row_size=%u, head=%u, tail=%u\n",
           cb->rows, cb->row_size, cb->head, cb->tail);

    // Put some rows
    for (int i = 0; i < 5; i++) {
        float data[4] = {(float)i*(float)1.0, (float)i*(float)2.0, (float)i*(float)3.0, (float)i*(float)4.0};
        put_data(cb, data);
        printf("After put %d: head=%u, tail=%u, data={%.1f, %.1f, %.1f, %.1f}\n",
               i, cb->head, cb->tail, data[0], data[1], data[2], data[3]);
    }

    // Pull all rows
    for (int i = 0; i < 5; i++) {
        float out[4];
        pull_data(cb, out);
        printf("After pull %d: head=%u, tail=%u, data={%.1f, %.1f, %.1f, %.1f}\n",
               i, cb->head, cb->tail, out[0], out[1], out[2], out[3]);
    }

    free_buffer(cb);
    printf("Buffer freed.\n");
}