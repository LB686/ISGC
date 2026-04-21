/**
 * @brief    LED指示灯驱动模块
 * @file     bsp_led.c
 * @details  高可移植/可扩展的LED驱动
 * @mainpage
 * @author   LB
 * @version  V2.0
 * @date     2026/04/14
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "../../bsp/inc/bsp_led.h"
#include "main.h"
  

/* Private variables ---------------------------------------------------------*/
/* LED硬件配置表（只需维护此数组，新增LED仅在此添加一行） */
static const LED_Hardware_t ledTable[LED_MAX_COUNT] = {
    {LED1_PORT, LED1_PIN, LED1_ACTIVE},
    {LED2_PORT, LED2_PIN, LED2_ACTIVE},
};

/* Private function prototypes -----------------------------------------------*/
static void LED_On(uint8_t ledId);
static void LED_Off(uint8_t ledId);
static void LED_Flip(uint8_t ledId);
static void LED_HardwareInit(void);

/* Public variables ----------------------------------------------------------*/
/* 对外提供的统一操作接口 */
LED_Driver_t LED = {
    .On   = LED_On,
    .Off  = LED_Off,
    .Flip = LED_Flip
};

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  硬件初始化：配置所有LED的GPIO为推挽输出
 */
static void LED_HardwareInit(void)
{
    GPIO_InitTypeDef gpioInit = {0};

    /* 使能GPIO时钟 */
    LED_GPIO_CLK_ENABLE();

    /* 通用配置：推挽输出、无上下拉、低速 */
    gpioInit.Mode  = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull  = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;

    /* 遍历所有LED，逐个初始化GPIO */
    for (uint8_t i = 0; i < LED_MAX_COUNT; i++) {
        gpioInit.Pin = ledTable[i].pin;
        HAL_GPIO_Init(ledTable[i].port, &gpioInit);
        /* 默认全部熄灭（写入无效电平） */
        HAL_GPIO_WritePin(ledTable[i].port,
                          ledTable[i].pin,
                          (ledTable[i].activeLevel == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/**
 * @brief  点亮指定LED（LED编号从0开始）
 * @param  ledId: LED索引 (0 ~ LED_MAX_COUNT-1)
 */
static void LED_On(uint8_t ledId)
{
    if (ledId >= LED_MAX_COUNT) {
        return;
    }
    HAL_GPIO_WritePin(ledTable[ledId].port,
                      ledTable[ledId].pin,
                      ledTable[ledId].activeLevel);
}

/**
 * @brief  熄灭指定LED
 * @param  ledId: LED索引 (0 ~ LED_MAX_COUNT-1)
 */
static void LED_Off(uint8_t ledId)
{
    if (ledId >= LED_MAX_COUNT) {
        return;
    }
    GPIO_PinState offLevel = (ledTable[ledId].activeLevel == GPIO_PIN_SET) ?
                             GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(ledTable[ledId].port,
                      ledTable[ledId].pin,
                      offLevel);
}

/**
 * @brief  翻转指定LED电平
 * @param  ledId: LED索引 (0 ~ LED_MAX_COUNT-1)
 */
static void LED_Flip(uint8_t ledId)
{
    if (ledId >= LED_MAX_COUNT) {
        return;
    }
    HAL_GPIO_TogglePin(ledTable[ledId].port,
                       ledTable[ledId].pin);
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  初始化LED驱动（上层调用入口）
 */
void BSP_LED_Init(void)
{
    LED_HardwareInit();
}
