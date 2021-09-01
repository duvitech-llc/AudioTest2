#ifndef __FIFO_BUFFER_H_
#define __FIFO_BUFFER_H_

#include <stdint.h>

typedef struct {
	 int * buf;
     int head;
     int tail;
     int size;
} fifo_t;

void fifo_init(fifo_t * f, int * buf, int size);
int fifo_read(fifo_t * f, void * buf, int nbytes);
int fifo_write(fifo_t * f, const void * buf, int nbytes);
int fifo_get_read_index(fifo_t* f);
int fifo_get_write_index(fifo_t* f);
int fifo_lookback(fifo_t* f, int* buf, int offset);

#endif // __FIFO_BUFFER_H_
