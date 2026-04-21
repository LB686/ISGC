/*
*********************************************************************************************************
*   模块名称 : LED指示灯驱动模块
*   文件名称 : bsp_led.h
*   版    本 : V2.0
*   说    明 : 头文件
*********************************************************************************************************
*/
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "stdint.h"
#include "gpio.h" 

/* Private define ------------------------------------------------------------*/
/* 支持的最大LED数量，可通过修改此值轻松扩展 */
#define LED_MAX_COUNT   2

/* 每个LED的GPIO定义（便于移植，只需修改下方数组） */
#define LED1_PORT       GPIOC
#define LED1_PIN        GPIO_PIN_6
#define LED1_ACTIVE     GPIO_PIN_RESET    /* 低电平点亮 */

#define LED2_PORT       GPIOC
#define LED2_PIN        GPIO_PIN_4
#define LED2_ACTIVE     GPIO_PIN_RESET    /* 低电平点亮 */

/* 统一使能所有LED涉及的GPIO时钟 */
#define LED_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()

/* Private typedef -----------------------------------------------------------*/
/* 单个LED硬件描述结构体 */
typedef struct {
    GPIO_TypeDef    *port;          /* GPIO端口 */
    uint16_t         pin;           /* 引脚编号 */
    GPIO_PinState    activeLevel;   /* 有效电平（点亮时的电平） */
} LED_Hardware_t;

/* LED操作句柄结构体（函数指针封装） */
typedef struct {
    void (*On)  (uint8_t ledId);     /* 点亮 */
    void (*Off) (uint8_t ledId);     /* 熄灭 */
    void (*Flip)(uint8_t ledId);     /* 翻转 */
} LED_Driver_t;

/* 全局LED驱动实例（仅一个，操作所有LED） */
extern LED_Driver_t LED;

/* 初始化所有LED硬件 */
void BSP_LED_Init(void);

#endif /* __BSP_LED_H__ */
