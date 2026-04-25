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
#include "../../bsp/bsp_system_time.h"

extern const TIM_HandleTypeDef *htim_system;
uint64_t timer_tick_count = 0;

/**************************************************************
==> 功  能：1ms计数
***************************************************************/
void system_timer_tick(void)
{
  timer_tick_count++;
}

/**************************************************************
==> 功  能：获取us计数值
***************************************************************/
uint64_t get_system_time_us(void)
{
  return timer_tick_count * (htim_system->Instance->ARR + 1) + htim_system->Instance->CNT;
}

/**************************************************************
==> 功  能：获取ms计数值
***************************************************************/
uint64_t get_system_time_ms(void)
{
  return get_system_time_us()/1000;
}

/**************************************************************
==> 功  能：us延时函数
***************************************************************/
void delay_us(uint64_t us)
{
	uint64_t start_us = get_system_time_us();
	uint64_t stop_us = start_us + us;
	while(get_system_time_us() < stop_us)
	{
	
	}
}

/**************************************************************
==> 功  能：ms延时函数
***************************************************************/
void delay_ms(uint64_t ms)
{
  delay_us(1000 * ms);
}

