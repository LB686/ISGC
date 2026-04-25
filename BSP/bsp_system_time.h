/**
 * @brief    定时器模块，用于提供时基
 * @file     bsp_system_time.c
 * @details  
 * @mainpage 
 * @author   LB
 * @email 
 * @version  V1.0
 * @date     2024/06/23
 * @license  Copyright (c) 2024-2024 浙江华奕航空科技有限公司.All rights reserved.
 */
#ifndef __BSP_SYSTEM_TIME_H
#define __BSP_SYSTEM_TIME_H

#include "stdint.h"
#include "main.h"

///////////////使用方法////////////////////////
//1、配置htim为1ms触发一次，向上计数，每1us记一次（ARR=1000）
//2、htim的中断处理里system_timer_tick
//3、定义const TIM_HandleTypeDef *htim_system
//4、system_time_ms即为毫秒单位的系统时间
//5、调用get_system_time_us获取us单位的系统时间

extern const TIM_HandleTypeDef *hitm_system;
extern uint64_t system_time_ms;

inline uint64_t get_system_time_us(void);
inline uint64_t get_system_time_ms(void);
void delay_us(uint64_t us);
void delay_ms(uint64_t ms);
extern void system_timer_tick(void);

#endif
