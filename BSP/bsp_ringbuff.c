#include "bsp_ringbuff.h"
#include "string.h"


/**************************************************************
==> 功  能：初始化循环缓存
***************************************************************/
void fifo_init(struct ring_buffer* fifo, uint8_t *buffer, uint32_t size) 
{
	fifo->buffer = buffer;
	fifo->size = size;
	fifo->in = 0;
	fifo->out = 0;
	fifo->is_full = 0;
}


/**************************************************************
==> 功  能：复位循环缓存
***************************************************************/
void fifo_reset(struct ring_buffer* fifo) 
{
	memset(fifo->buffer, 0, fifo->size);
	fifo->in = 0;
	fifo->out = 0;
	fifo->is_full = 0;
}


/**************************************************************
==> 功  能：判定循环缓存是否空
***************************************************************/
uint8_t fifo_is_empty(struct ring_buffer* fifo) 
{
	return (fifo->in == fifo->out) && !fifo->is_full;
}


/**************************************************************
==> 功  能：判定循环缓存是否满
***************************************************************/
uint8_t fifo_is_full(struct ring_buffer* fifo) 
{
	return fifo->is_full;
}


/**************************************************************
==> 功  能：向循环缓存出入一个字节
==> 说  明：
            fifo 循环缓存
            data 要存入的字节
==> 返  回：存入失败返回0，否则返回1或非零值
***************************************************************/
uint8_t fifo_push_byte(struct ring_buffer* fifo, uint8_t data) 
{
	if (fifo->is_full) 
	{
		return 0;
	}

	fifo->buffer[fifo->in] = data;
	
	if (fifo->in + 1 == fifo->size) 
	{
		fifo->in = 0;
	} 
	else 
	{
		fifo->in++;
	}
	
	if (fifo->in == fifo->out) 
	{
		fifo->is_full = 1;
	}
	
	return 1;
}


/**************************************************************
==> 功  能：将数据存入循环缓存
==> 说  明：
            fifo 循环缓存
            data 要存储的数据
            length 要存储的数据长度
==> 返  回：实际存入的数据长度
***************************************************************/
uint32_t fifo_push(struct ring_buffer* fifo, const uint8_t *data, uint8_t length) 
{
	uint32_t bytes_pushed = 0;
	
	for (int i = 0; i < length; i++) 
	{
		if (fifo_push_byte(fifo, data[i])) 
		{
			bytes_pushed++;
		} else 
		{
		  break;
		}
	}
	return bytes_pushed;
}


/**************************************************************
==> 功  能：从循环缓存中读取一个字节
==> 说  明：
            fifo 要读取的缓存
            data 读取后存储字节
==> 返  回：读取失败返回0，否则返回1或者非0值
***************************************************************/
uint8_t fifo_pop_byte(struct ring_buffer* fifo, uint8_t *data) 
{
	if (fifo_is_empty(fifo)) 
	{
		return 0;
	}
	
	*data = fifo->buffer[fifo->out];

	if (fifo->out + 1 == fifo->size) 
	{
		fifo->out = 0;
	} else 
	{
		fifo->out++;
	}	
	
	if (fifo->is_full) 
	{
		fifo->is_full = 0;
	}
	
	return 1;
}

/**************************************************************
==> 功  能：从循环缓存中批量读取数据
==> 说  明：
            fifo    循环缓存指针
            buffer  存放读取数据的缓冲区
            length  期望读取的长度
==> 返  回：实际读取的字节数
***************************************************************/
uint32_t fifo_pop(struct ring_buffer* fifo, uint8_t *buffer, uint32_t length) {
    uint32_t occupied = fifo_occuppied(fifo);
    uint32_t bytes_to_read = (length < occupied) ? length : occupied;
    if (bytes_to_read == 0) return 0;

    // 处理环形缓冲区回绕，分段拷贝
    if (fifo->out < fifo->in) {
        // 数据连续存放在 [out, in-1]
        memcpy(buffer, fifo->buffer + fifo->out, bytes_to_read);
    } else if (fifo->out > fifo->in) {
        // 数据分为两段：out 到 buffer 尾，以及 buffer 头到 in-1
        uint32_t part1 = fifo->size - fifo->out;
        if (bytes_to_read <= part1) {
            memcpy(buffer, fifo->buffer + fifo->out, bytes_to_read);
        } else {
            memcpy(buffer, fifo->buffer + fifo->out, part1);
            memcpy(buffer + part1, fifo->buffer, bytes_to_read - part1);
        }
    } else { // fifo->out == fifo->in
        if (fifo->is_full) {
            // 缓冲区满，数据占据整个缓冲区
            uint32_t part1 = fifo->size - fifo->out;
            if (bytes_to_read <= part1) {
                memcpy(buffer, fifo->buffer + fifo->out, bytes_to_read);
            } else {
                memcpy(buffer, fifo->buffer + fifo->out, part1);
                memcpy(buffer + part1, fifo->buffer, bytes_to_read - part1);
            }
        } else {
            return 0; // 缓冲区为空
        }
    }

    // 更新读指针并清除满标志（只要读走数据，就一定不再是满状态）
    fifo->out = (fifo->out + bytes_to_read) % fifo->size;
    fifo->is_full = 0;
    return bytes_to_read;
}

/**************************************************************
==> 功  能：从循环缓存中读取全部数据
==> 说  明：
            fifo 循环缓存
            data 存放读取出数据的缓存区
            length 要存储的数据长度
==> 返  回：实际读取到的数据长度
***************************************************************/
uint32_t fifo_take_all(struct ring_buffer* fifo, uint8_t *buffer) 
{
	int32_t bytes_taken = fifo_occuppied(fifo);
	
	if (fifo->out < fifo->in)
	{
		memcpy(buffer, fifo->buffer + fifo->out, bytes_taken);		
	} 
	else if (fifo->out > fifo->in) 
	{
		memcpy(buffer, fifo->buffer + fifo->out, fifo->size - fifo->out);
		memcpy(buffer + fifo->size - fifo->out, fifo->buffer,  bytes_taken + fifo->out - fifo->size);
	} else 
	{
		if (fifo->is_full) 
		{
			memcpy(buffer, fifo->buffer + fifo->out, fifo->size - fifo->out);
			memcpy(buffer + fifo->size - fifo->out, fifo->buffer,  bytes_taken + fifo->out - fifo->size);
			fifo->is_full = 0;
		}
	}
	
	fifo->out = fifo->in;
	fifo->is_full = 0; 
	
	return bytes_taken;
}



/**************************************************************
==> 功  能：获取fifo的剩余空间
==> 说  明：
            fifo 要操作的缓存
==> 返  回：返回缓存的剩余空间
***************************************************************/
uint32_t fifo_free_size(struct ring_buffer* fifo) 
{
	return (int64_t)fifo->size - (int64_t)fifo_occuppied(fifo);
}


/**************************************************************
==> 功  能：获取fifo的已用空间
==> 说  明：
            fifo 要操作的缓存
==> 返  回：返回已用缓存字节数
***************************************************************/
uint32_t fifo_occuppied(struct ring_buffer* fifo) 
{
	int64_t occupied = 0;
	
	occupied = (int64_t)fifo->in - (int64_t)fifo->out;
	
	if (occupied < 0) 
	{
		occupied += fifo->size;
	}
	
	if (occupied == 0 && fifo->is_full) 
	{
		occupied = fifo->size;
	}
	return occupied;
}
