/**
 * @brief    BSP模块(For STM32H7)
 * @file     bsp.h
 * @details  这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 来包含所有的外设驱动模块。
 *			 bsp = Borad surport packet 板级支持包
 * @mainpage 
 * @author   LB
 * @email 
 * @version  V1.0
 * @date     2024/06/23
 * @license  Copyright (c) 2024-2024 浙江华奕航空科技有限公司.All rights reserved.
 */

#ifndef __BSP_H_
#define __BSP_H_

#define  USE_FreeRTOS      0
#define  CUBEMX_CONFIG     1                //cubemx配置宏，通过预编译指令避免cubemx更改用户代码,1表示开启防更改功能


#if USE_FreeRTOS == 1
	#include "FreeRTOS.h"
	#include "task.h"
    #include "cmsis_os.h"
	#define DISABLE_INT()    taskENTER_CRITICAL()
	#define ENABLE_INT()     taskEXIT_CRITICAL()
#else
	/* 开关全局中断的宏 */
	#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */
#endif

#include "stm32F4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 通过取消注释或者添加注释的方式控制是否包含底层驱动模块 */
#include "../../bsp/inc/bsp_led.h"
#include "../../bsp/inc/bsp_KEY.h"
#include "../../bsp/inc/bsp_uart.h"
//#include "../../bsp/inc/bsp_uart_config.h"
#include "../../bsp/bsp_ringbuff.h"
#include "../../bsp/inc/bsp_user_lib.h"





/* 提供给其他C文件调用的函数 */
void bsp_Init(void);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
