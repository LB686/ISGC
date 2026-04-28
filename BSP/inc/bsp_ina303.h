/*
*********************************************************************************************************
*   模块名称 : INA303/BEMF/Vbus/Tempr 采集驱动模块（注入组版本）
*   文件名称 : bsp_ina303.h
*   版    本 : V2.1
*   说    明 : - ADC1 注入组：三相电流，TIM1_CC4 触发，JEOC 中断
*              - ADC2 注入组：三相 BEMF，TIM1_CC4 触发，JEOC 中断
*              - ADC3 规则组：Vbus + Temp，软件轮询 1kHz
*********************************************************************************************************
*/

#ifndef __BSP_INA303_H__
#define __BSP_INA303_H__

#include "../../bsp/inc/bsp_ina303_cfg.h"

/* =======================================================================================================
 * 数据类型定义
 * ======================================================================================================= */

typedef enum {
    INA303_TYPE_CURRENT = 0,
    INA303_TYPE_BEMF    = 1,
    INA303_TYPE_VOLTAGE = 2,
    INA303_TYPE_TEMP    = 3
} INA303_ChannelType_t;

typedef enum {
    INA303_CH_IU = 0,
    INA303_CH_IV,
    INA303_CH_IW,
    INA303_CH_BEMF_A,
    INA303_CH_BEMF_B,
    INA303_CH_BEMF_C,
    INA303_CH_VBUS,
    INA303_CH_TEMP,
    INA303_CH_NUM
} INA303_ChannelIndex_t;

typedef enum {
    INA303_FAULT_NONE       = 0x00,
    INA303_FAULT_OVER_CUR   = 0x01,
    INA303_FAULT_UNDER_CUR  = 0x02,
    INA303_FAULT_OVER_VOLT  = 0x04,
    INA303_FAULT_UNDER_VOLT = 0x08,
    INA303_FAULT_ADC_ERR    = 0x10
} INA303_FaultFlag_t;

typedef enum {
    INA303_CAL_IDLE = 0,
    INA303_CAL_RUNNING,
    INA303_CAL_DONE
} INA303_CalState_t;

/* =======================================================================================================
 * API 函数声明
 * ======================================================================================================= */

/* 初始化：启动 TIM1 CH4、启动 ADC1/ADC2 注入组中断模式、初始化 ADC3 */
void BSP_INA303_Init(void);

/* ADC1 JEOC 中断回调（20kHz，在 ADC_IRQHandler 中调用） */
void BSP_INA303_ADC1_JEOC_IRQHandler(void);

/* ADC2 JEOC 中断回调（20kHz，在 ADC_IRQHandler 中调用） */
void BSP_INA303_ADC2_JEOC_IRQHandler(void);

/* 检查注入组是否已完成（用于在 ISR 中触发 FOC）
 * 若 INA303_USE_ADC2_BEMF=0，只检查 ADC1；若=1，需 ADC1+ADC2 都完成 */
uint8_t BSP_INA303_IsInjectedDone(void);

/* 启动 ADC2 注入组（在需要 BEMF 时调用，如发电模式检测前） */
void BSP_INA303_ADC2_Start(void);

/* 快速循环任务（1kHz）：滤波、物理量计算、保护检测、ADC3 轮询 */
void BSP_INA303_FastTask(void);

/* ---------- 获取接口 ---------- */
float BSP_INA303_GetCurrentU(void);
float BSP_INA303_GetCurrentV(void);
float BSP_INA303_GetCurrentW(void);
float BSP_INA303_GetBemfA(void);
float BSP_INA303_GetBemfB(void);
float BSP_INA303_GetBemfC(void);
float BSP_INA303_GetBusVoltage(void);
float BSP_INA303_GetTemperature(void);

uint16_t BSP_INA303_GetRaw(INA303_ChannelIndex_t ch);
uint16_t BSP_INA303_GetFiltered(INA303_ChannelIndex_t ch);

/* ---------- 校准接口 ---------- */
void BSP_INA303_CalibrateZeroStart(void);
INA303_CalState_t BSP_INA303_CalibrateZeroPoll(void);
void BSP_INA303_SetGainCal(INA303_ChannelIndex_t ch, float gainCal);
float BSP_INA303_GetGainCal(INA303_ChannelIndex_t ch);

/* ---------- 故障检测 ---------- */
uint8_t BSP_INA303_GetFaultFlags(void);
void BSP_INA303_ClearFaults(void);
uint8_t BSP_INA303_IsOverCurrent(void);
uint8_t BSP_INA303_IsUnderCurrent(void);
uint8_t BSP_INA303_IsOverVoltage(void);
uint8_t BSP_INA303_IsUnderVoltage(void);

/* 动态过流阈值设置（用于启动阶段分阶段限流） */
void BSP_INA303_SetOCThreshold(float thresholdA);
float BSP_INA303_GetOCThreshold(void);

#endif /* __BSP_INA303_H__ */
/***************************** (END OF FILE) *********************************/
