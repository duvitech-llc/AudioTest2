#include <string.h>

#include "fifo_buff.h"


//This initializes the FIFO structure with the given buffer and size
void fifo_init(fifo_t * f, int * buf, int size){
     f->head = 0;
     f->tail = 0;
     f->size = size;
     f->buf = buf;
}

//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
int fifo_read(fifo_t * f, void * buf, int nbytes){
     int i;
     int * p;
     p = buf;
     for(i=0; i < nbytes; i++){
          if( f->tail != f->head ){ //see if any data is available
               *p++ = f->buf[f->tail];  //grab a byte from the buffer
               f->tail++;  //increment the tail
               if( f->tail == f->size ){  //check for wrap-around
                    f->tail = 0;
               }
          } else {
               return i; //number of bytes read
          }
     }
     return nbytes;
}

//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned
int fifo_write(fifo_t * f, const void * buf, int nbytes){
     int i;
     const int* p;
     p = buf;
     for(i=0; i < nbytes; i++){
           //first check to see if there is space in the buffer
           if( (f->head + 1 == f->tail) ||
                ( (f->head + 1 == f->size) && (f->tail == 0) ))
					 {
                 return i; //no more room
           } else {
               f->buf[f->head] = *p++;
               f->head++;  //increment the head
               if( f->head == f->size ){  //check for wrap-around
                    f->head = 0;
               }
           }
     }
     return nbytes;
}


int fifo_lookback(fifo_t* f, int* buf, int offset) {
	int * p;
    p = buf;
    int position = f->head - offset;
    if (position < 0) {
        position = f->size + position;
    }

    if (position <= f->tail) {
        *p = f->buf[f->tail];
        return f->tail;
    }

    *p = f->buf[position];
    return position;
}

int fifo_get_read_index(fifo_t* f) {
    return f->tail;
}

int fifo_get_write_index(fifo_t* f) {
    return f->head;
}
