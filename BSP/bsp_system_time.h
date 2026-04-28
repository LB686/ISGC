/**
 * @brief    ��ʱ��ģ�飬�����ṩʱ��
 * @file     bsp_system_time.c
 * @details  
 * @mainpage 
 * @author   LB
 * @email 
 * @version  V1.0
 * @date     2024/06/23
 * @license  Copyright (c) 2024-2024 �㽭���Ⱥ��տƼ����޹�˾.All rights reserved.
 */
#ifndef __BSP_SYSTEM_TIME_H
#define __BSP_SYSTEM_TIME_H

#include "stdint.h"
#include "main.h"

///////////////ʹ�÷���////////////////////////
//1������htimΪ1ms����һ�Σ����ϼ�����ÿ1us��һ�Σ�ARR=1000��
//2��htim���жϴ�����system_timer_tick
//3������const TIM_HandleTypeDef *htim_system
//4��system_time_ms��Ϊ���뵥λ��ϵͳʱ��
//5������get_system_time_us��ȡus��λ��ϵͳʱ��

extern const TIM_HandleTypeDef *htim_system;
extern uint64_t system_time_ms;

uint64_t get_system_time_us(void);
uint64_t get_system_time_ms(void);
void delay_us(uint64_t us);
void delay_ms(uint64_t ms);
extern void system_timer_tick(void);

#endif
