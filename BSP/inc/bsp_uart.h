/*
*********************************************************************************************************
*
*	模块名称 : 串口+FIFO驱动模块
*	文件名称 : bsp_uart.h
*	版    本 : V1.0
*	说    明 : 
*	修改记录 :
*		版本号  日期         作者      说明
*		V1.0    2025-03-19   libing   驱动创建
*
*
*********************************************************************************************************
*/

#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include "../../bsp/bsp.h"

#define UART3_EN                    1        //串口3使能
#define USART3_TX_DMA_EN            1        //使能串口3发送DMA使能

//串口数据发送缓存区大小，改动需要对c文件中的内存强制对齐函数进行确定大小  
#define USART3_TX_FIFO_SIZE 1024
#define USART3_RX_FIFO_SIZE 1024           

/* 串口接收单字节回调类型（用户注册后，驱动在每次收到字节时直接调用） */
typedef void (*USART3_RxByteCallback_t)(uint8_t byte);

//串口数据发送
void usart3_send(const uint8_t *buff, uint16_t len);

//串口数据解析（轮询方式，若使用回调可不再调用）
void usart_dataparse(void);

//串口配置
void usart_configure(void);

//查看串口发送忙状态
void read_usart_state(void);

//注册串口接收回调（NULL表示注销）
void usart3_register_rx_callback(USART3_RxByteCallback_t cb);

#endif
/***************************** (END OF FILE) *********************************/
