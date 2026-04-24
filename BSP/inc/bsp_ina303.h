/*
*********************************************************************************************************
*   模块名称 : INA303电流/电压采集驱动模块
*   文件名称 : bsp_ina303.h
*   版    本 : V1.0
*   说    明 : 高可靠性、高可移植性INA303A1驱动头文件
*              支持：3路双向电流 + 1路母线电压，ADC DMA连续采集，软件滤波，过流/过压保护
*********************************************************************************************************
*/

#ifndef __BSP_INA303_H__
#define __BSP_INA303_H__

#include "../../bsp/inc/bsp_ina303_cfg.h"

/* =======================================================================================================
 * 数据类型定义
 * ======================================================================================================= */

/* 通道类型 */
typedef enum {
    INA303_TYPE_CURRENT = 0,    /* 电流通道 */
    INA303_TYPE_VOLTAGE = 1     /* 电压通道 */
} INA303_ChannelType_t;

/* 通道索引（与配置文件中SCAN_IDX顺序无关，固定为逻辑通道） */
typedef enum {
    INA303_CH_IU = 0,           /* U相电流 */
    INA303_CH_IV,               /* V相电流 */
    INA303_CH_IW,               /* W相电流 */
    INA303_CH_VBUS,             /* 母线电压 */
    INA303_CH_NUM               /* 通道总数 */
} INA303_ChannelIndex_t;

/* 故障标志位 */
typedef enum {
    INA303_FAULT_NONE       = 0x00,
    INA303_FAULT_OVER_CUR   = 0x01,     /* 过流 */
    INA303_FAULT_UNDER_CUR  = 0x02,     /* 欠流（负向过流） */
    INA303_FAULT_OVER_VOLT  = 0x04,     /* 母线过压 */
    INA303_FAULT_UNDER_VOLT = 0x08,     /* 母线欠压 */
    INA303_FAULT_ADC_ERR    = 0x10      /* ADC数值异常（超出12位范围） */
} INA303_FaultFlag_t;

/* 校准状态 */
typedef enum {
    INA303_CAL_IDLE = 0,        /* 空闲 */
    INA303_CAL_RUNNING,         /* 校准中 */
    INA303_CAL_DONE             /* 完成 */
} INA303_CalState_t;

/* =======================================================================================================
 * API函数声明
 * ======================================================================================================= */

/* 初始化与启动 */
void BSP_INA303_Init(void);

/* DMA中断回调（必须在对应的中断服务函数中调用） */
void BSP_INA303_DMAHalfCplt(void);
void BSP_INA303_DMACplt(void);

/* 快速循环任务（建议在1kHz~10kHz定时中断中调用，执行滤波和物理量计算） */
void BSP_INA303_FastTask(void);

/* 获取三相电流（单位：A，双向，发电为负） */
float BSP_INA303_GetCurrentU(void);
float BSP_INA303_GetCurrentV(void);
float BSP_INA303_GetCurrentW(void);

/* 获取母线电压（单位：V） */
float BSP_INA303_GetBusVoltage(void);

/* 获取原始ADC值（0~4095） */
uint16_t BSP_INA303_GetRawU(void);
uint16_t BSP_INA303_GetRawV(void);
uint16_t BSP_INA303_GetRawW(void);
uint16_t BSP_INA303_GetRawVbus(void);

/* 获取滤波后的ADC值（0~4095） */
uint16_t BSP_INA303_GetFilteredU(void);
uint16_t BSP_INA303_GetFilteredV(void);
uint16_t BSP_INA303_GetFilteredW(void);
uint16_t BSP_INA303_GetFilteredVbus(void);

/* 校准接口 */
void BSP_INA303_CalibrateZeroStart(void);       /* 启动零偏校准（调用前确保电流为0A） */
INA303_CalState_t BSP_INA303_CalibrateZeroPoll(void); /* 轮询校准状态，返回DONE表示完成 */
void BSP_INA303_SetGainCal(INA303_ChannelIndex_t ch, float gainCal);
float BSP_INA303_GetGainCal(INA303_ChannelIndex_t ch);

/* 故障检测 */
uint8_t BSP_INA303_GetFaultFlags(void);
void BSP_INA303_ClearFaults(void);
uint8_t BSP_INA303_IsOverCurrent(void);         /* 任意相过流 */
uint8_t BSP_INA303_IsUnderCurrent(void);        /* 任意相欠流 */
uint8_t BSP_INA303_IsOverVoltage(void);
uint8_t BSP_INA303_IsUnderVoltage(void);

/* 硬件比较器中断回调（如后续连接Alert引脚，在对应EXTI中断中调用） */
void BSP_INA303_Alert1_IRQHandler(void);
void BSP_INA303_Alert2_IRQHandler(void);

#endif /* __BSP_INA303_H__ */
/***************************** (END OF FILE) *********************************/
