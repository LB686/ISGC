/**
 * @brief    电机串口通讯协议
 * @file     app_motor_uart_parse.h
 * @details  
 * @author   LB
 * @version  V1.0
 * @date     2026/04/14
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

#ifndef __APP_MOTOR_UART_PARSE_H_
#define __APP_MOTOR_UART_PARSE_H_

/* Includes ------------------------------------------------------------------*/
#include "../../app/app_motor_vf_demo.h"
#include "../../bsp/inc/bsp_uart.h"

/* Exported variables --------------------------------------------------------*/
extern uint8_t uart_debug_buff[32];

/* Exported functions --------------------------------------------------------*/
void uart_MotorDataSend(void);
void uart_MotorDataParse(void);
void usart3_frame_assemble(uint8_t b);

#endif /* __APP_MOTOR_UART_PARSE_H_ */

/***************************** (END OF FILE) *********************************/

