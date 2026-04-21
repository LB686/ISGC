#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#include "stdint.h"
//环形存储
struct ring_buffer
{
	uint8_t     *buffer;	  	//缓存区地址
	uint32_t    size;	    	//缓存区大小
	uint32_t    out;	      	//读取索引
	uint32_t    in;	      		//写入索引
	uint8_t     is_full;	  	//缓存已满
	uint8_t     modifying;		//正在写入一个字节
};

void fifo_init(struct ring_buffer* fifo, uint8_t *buffer, uint32_t size);
void fifo_reset(struct ring_buffer* fifo);
uint8_t fifo_push_byte(struct ring_buffer* fifo, uint8_t data);
uint32_t fifo_push(struct ring_buffer* fifo, const uint8_t *data, uint8_t length);
uint32_t fifo_take_all(struct ring_buffer* fifo, uint8_t *buffer);
uint8_t fifo_is_full(struct ring_buffer* fifo);
uint8_t fifo_pop_byte(struct ring_buffer* fifo, uint8_t *data);
uint32_t fifo_pop(struct ring_buffer* fifo, uint8_t *buffer, uint32_t length);
uint32_t fifo_free_size(struct ring_buffer* fifo);
uint32_t fifo_occuppied(struct ring_buffer* fifo);
uint8_t fifo_is_empty(struct ring_buffer* fifo);

#endif

