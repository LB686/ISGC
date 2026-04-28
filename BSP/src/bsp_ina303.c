/*
*********************************************************************************************************
*   模块名称 : INA303/BEMF/Vbus/Tempr 采集驱动模块（注入组版本）
*   文件名称 : bsp_ina303.c
*   版    本 : V2.1
*   说    明 : - ADC1 注入组：三相电流（Iu/Iv/Iw），TIM1_CC4 RISING 触发，JEOC 中断
*              - ADC2 注入组：三相 BEMF（A/B/C），TIM1_CC4 RISING 触发，JEOC 中断
*              - ADC3 规则组：Vbus + Temp，软件轮询 1kHz
*              - 滑动平均滤波，零偏校准，过流/过压保护（消抖+迟滞）
*********************************************************************************************************
*/

#include "../../bsp/inc/bsp_ina303.h"
#include "../../bsp/bsp.h"
#include <string.h>

/* =======================================================================================================
 * 内部宏定义
 * ======================================================================================================= */

/* 电流转换系数：A/LSB = Vref / 4096 / (Rsense * Gain) */
#define INA303_CURR_SCALE_A_PER_LSB     ((INA303_VREF_V) / (INA303_ADC_RESOLUTION) / ((INA303_RSENSE_OHM) * (INA303_GAIN_VV)))

/* BEMF 转换系数：V/LSB = Vref / 4096 / BEMF_DIV_RATIO */
#define INA303_BEMF_SCALE_V_PER_LSB     ((INA303_VREF_V) / (INA303_ADC_RESOLUTION) / (INA303_BEMF_DIV_RATIO))

/* 母线电压转换系数：V/LSB = Vref / 4096 / VBUS_DIV_RATIO */
#define INA303_VBUS_SCALE_V_PER_LSB     ((INA303_VREF_V) / (INA303_ADC_RESOLUTION) / (INA303_VBUS_DIV_RATIO))

/* 温度转换系数：°C/LSB */
#define INA303_TEMP_SCALE_C_PER_LSB     (INA303_TEMP_SCALE_C_PER_LSB)

#define INA303_ADC_MAX_VALUE            4095

/* 零偏校准采样次数（2^n） */
#define INA303_CAL_SAMPLE_SHIFT         6
#define INA303_CAL_SAMPLE_NUM           (1U << INA303_CAL_SAMPLE_SHIFT)

/* 临界区保护（如 bsp.h 未定义，使用底层指令） */
#ifndef DISABLE_INT
    #define DISABLE_INT()               __disable_irq()
    #define ENABLE_INT()                __enable_irq()
#endif

/* =======================================================================================================
 * 内部数据结构
 * ======================================================================================================= */

typedef struct {
    uint16_t rawAdc;
    uint16_t filteredAdc;
    float    physicalValue;
    int16_t  offsetAdc;
    float    gainCal;
    uint16_t filterBuf[INA303_FILTER_SIZE];
    uint32_t filterSum;
    uint8_t  filterIdx;
    uint8_t  filterReady;
    uint16_t protDebounceOC;
    uint16_t protDebounceUC;
    uint16_t protDebounceOV;
    uint16_t protDebounceUV;
    uint8_t  protFlagOC;
    uint8_t  protFlagUC;
    uint8_t  protFlagOV;
    uint8_t  protFlagUV;
} INA303_ChannelData_t;

typedef struct {
    uint8_t  type;              /* 0=电流, 1=BEMF, 2=电压, 3=温度 */
    int16_t  defaultOffset;
    float    scaleFactor;
} INA303_ChannelConfig_t;

static const INA303_ChannelConfig_t s_chConfig[INA303_CH_NUM] = {
    {INA303_TYPE_CURRENT, INA303_DEFAULT_OFFSET_ADC, INA303_CURR_SCALE_A_PER_LSB},  /* IU     */
    {INA303_TYPE_CURRENT, INA303_DEFAULT_OFFSET_ADC, INA303_CURR_SCALE_A_PER_LSB},  /* IV     */
    {INA303_TYPE_CURRENT, INA303_DEFAULT_OFFSET_ADC, INA303_CURR_SCALE_A_PER_LSB},  /* IW     */
    {INA303_TYPE_BEMF,    0,                         INA303_BEMF_SCALE_V_PER_LSB},   /* BEMF_A */
    {INA303_TYPE_BEMF,    0,                         INA303_BEMF_SCALE_V_PER_LSB},   /* BEMF_B */
    {INA303_TYPE_BEMF,    0,                         INA303_BEMF_SCALE_V_PER_LSB},   /* BEMF_C */
    {INA303_TYPE_VOLTAGE, 0,                         INA303_VBUS_SCALE_V_PER_LSB},   /* VBUS   */
    {INA303_TYPE_TEMP,    INA303_TEMP_OFFSET_ADC,    INA303_TEMP_SCALE_C_PER_LSB}    /* TEMP   */
};

/* =======================================================================================================
 * 内部变量
 * ======================================================================================================= */

/* 注入组原始值（JEOC 中断中更新，volatile 确保主循环可见） */
static volatile uint16_t s_adc1InjRaw[INA303_ADC1_INJ_NUM];
static volatile uint16_t s_adc2InjRaw[INA303_ADC2_INJ_NUM];

/* JEOC 完成标志（用于双 ADC 同步） */
static volatile uint8_t s_adc1JeocDone = 0;
static volatile uint8_t s_adc2JeocDone = 0;

/* 通道运行时数据 */
static INA303_ChannelData_t s_chData[INA303_CH_NUM];

/* 全局故障标志 */
static volatile uint8_t s_faultFlags = 0;

/* 动态过流阈值（运行时可调，默认使用宏定义值） */
static float s_ocThresholdA = INA303_OC_THRESHOLD_A;

/* 初始化完成标志 */
static uint8_t s_initDone = 0;

/* 零偏校准状态 */
static INA303_CalState_t s_calState = INA303_CAL_IDLE;
static uint8_t  s_calChIndex = 0;
static uint16_t s_calSampleCnt = 0;
static int32_t  s_calAccSum = 0;

/* =======================================================================================================
 * 内部函数声明
 * ======================================================================================================= */
static void INA303_ProcessADC1InjData(void);
static void INA303_ProcessADC2InjData(void);
static void INA303_PollADC3(void);
static void INA303_UpdateFilter(INA303_ChannelIndex_t ch);
static void INA303_CalculatePhysical(INA303_ChannelIndex_t ch);
static void INA303_CheckProtection(void);

/* =======================================================================================================
 * 函数实现
 * ======================================================================================================= */

/**
 * @brief  初始化 INA303 驱动
 * @note   调用前 CUBEMX 必须已完成：
 *         - ADC1/ADC2 注入组 3 通道，TIM1_CC4 RISING 触发，JEOC 中断使能
 *         - ADC3 规则组 2 通道（Vbus+Temp），软件触发
 *         - TIM1 CH4 配置为 PWM Mode 2 No Output，CCR4=4050
 *         - 注入组采样时间 = 3 Cycles（关键！）
 */
void BSP_INA303_Init(void)
{
    uint8_t i, j;

    memset(s_chData, 0, sizeof(s_chData));
    memset((void *)s_adc1InjRaw, 0, sizeof(s_adc1InjRaw));
    memset((void *)s_adc2InjRaw, 0, sizeof(s_adc2InjRaw));
    s_adc1JeocDone = 0;
    s_adc2JeocDone = 0;
    s_faultFlags = 0;
    s_initDone = 0;
    s_calState = INA303_CAL_IDLE;

    for (i = 0; i < INA303_CH_NUM; i++) {
        s_chData[i].offsetAdc = s_chConfig[i].defaultOffset;
        s_chData[i].gainCal   = INA303_DEFAULT_GAIN_CAL;

        uint16_t initVal = (uint16_t)s_chConfig[i].defaultOffset;
        for (j = 0; j < INA303_FILTER_SIZE; j++) {
            s_chData[i].filterBuf[j] = initVal;
        }
        s_chData[i].filterSum = (uint32_t)initVal * INA303_FILTER_SIZE;
        s_chData[i].filterIdx = 0;
        s_chData[i].filterReady = 0;
    }

    /* TIM1 CH4 触发点 */
    TIM1->CCR4 = INA303_TIM1_CCR4_VALUE;

    /* 启动 TIM1 CH4（No Output，仅产生 OC4REF 事件） */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 启动 ADC1 注入组中断模式（等待 TIM1_CC4 硬件触发） */
    HAL_ADCEx_InjectedStart_IT(&INA303_ADC1_HANDLE);

    /* ADC2 根据宏决定是否启动（启动阶段通常禁用 BEMF） */
#if INA303_USE_ADC2_BEMF
    HAL_ADCEx_InjectedStart_IT(&INA303_ADC2_HANDLE);
#endif

    s_initDone = 1;
}

/**
 * @brief  ADC1 JEOC 中断回调（20kHz）
 * @note   极致轻量：只读 JDR 寄存器，不做任何计算
 */
void BSP_INA303_ADC1_JEOC_IRQHandler(void)
{
    if (!s_initDone) {
        return;
    }
    s_adc1InjRaw[INA303_ADC1_INJ_RANK_IU] = (uint16_t)(INA303_ADC1_HANDLE.Instance->JDR1);
    s_adc1InjRaw[INA303_ADC1_INJ_RANK_IV] = (uint16_t)(INA303_ADC1_HANDLE.Instance->JDR2);
    s_adc1InjRaw[INA303_ADC1_INJ_RANK_IW] = (uint16_t)(INA303_ADC1_HANDLE.Instance->JDR3);
    s_adc1JeocDone = 1;
}

/**
 * @brief  ADC2 JEOC 中断回调（20kHz）
 * @note   极致轻量：只读 JDR 寄存器，不做任何计算
 */
void BSP_INA303_ADC2_JEOC_IRQHandler(void)
{
    if (!s_initDone) {
        return;
    }
    s_adc2InjRaw[INA303_ADC2_INJ_RANK_BEMF_A] = (uint16_t)(INA303_ADC2_HANDLE.Instance->JDR1);
    s_adc2InjRaw[INA303_ADC2_INJ_RANK_BEMF_B] = (uint16_t)(INA303_ADC2_HANDLE.Instance->JDR2);
    s_adc2InjRaw[INA303_ADC2_INJ_RANK_BEMF_C] = (uint16_t)(INA303_ADC2_HANDLE.Instance->JDR3);
    s_adc2JeocDone = 1;
}

/**
 * @brief  检查注入组是否已完成转换
 * @retval 1: 已完成（内部标志自动清零），0: 尚未完成
 * @note   若 INA303_USE_ADC2_BEMF=0，只检查 ADC1；
 *         若=1，需 ADC1+ADC2 都完成后才返回 1。
 */
uint8_t BSP_INA303_IsInjectedDone(void)
{
    uint8_t ret = 0;
    DISABLE_INT();
#if INA303_USE_ADC2_BEMF
    if (s_adc1JeocDone && s_adc2JeocDone) {
        s_adc1JeocDone = 0;
        s_adc2JeocDone = 0;
        ret = 1;
    }
#else
    if (s_adc1JeocDone) {
        s_adc1JeocDone = 0;
        ret = 1;
    }
#endif
    ENABLE_INT();
    return ret;
}

/**
 * @brief  运行时启动 ADC2 注入组（用于发电模式检测前启用 BEMF）
 * @note   调用后 ADC2 开始响应 TIM1_CC4 触发，JEOC 中断生效
 */
void BSP_INA303_ADC2_Start(void)
{
    HAL_ADCEx_InjectedStart_IT(&INA303_ADC2_HANDLE);
}

/**
 * @brief  快速循环任务（建议在 1kHz 定时中断或主循环中调用）
 * @note   执行：提取注入组数据 -> ADC3 轮询 -> 滤波 -> 物理量计算 -> 保护检测
 */
void BSP_INA303_FastTask(void)
{
    uint8_t i;

    if (!s_initDone) {
        return;
    }

    /* 提取注入组新数据 */
    INA303_ProcessADC1InjData();
    INA303_ProcessADC2InjData();

    /* ADC3 规则组软件轮询（Vbus + Temp） */
    INA303_PollADC3();

    /* 逐通道滤波和物理量计算 */
    for (i = 0; i < INA303_CH_NUM; i++) {
        INA303_UpdateFilter((INA303_ChannelIndex_t)i);
        INA303_CalculatePhysical((INA303_ChannelIndex_t)i);
    }

    /* 保护检测 */
    INA303_CheckProtection();

    /* 零偏校准轮询（仅对电流通道） */
    if (s_calState == INA303_CAL_RUNNING) {
        (void)BSP_INA303_CalibrateZeroPoll();
    }
}

/* =======================================================================================================
 * 数据提取
 * ======================================================================================================= */

static void INA303_ProcessADC1InjData(void)
{
    uint16_t sample;

    sample = s_adc1InjRaw[INA303_ADC1_INJ_RANK_IU];
    if (sample <= INA303_ADC_MAX_VALUE) {
        s_chData[INA303_CH_IU].rawAdc = sample;
    } else {
        s_faultFlags |= INA303_FAULT_ADC_ERR;
    }

    sample = s_adc1InjRaw[INA303_ADC1_INJ_RANK_IV];
    if (sample <= INA303_ADC_MAX_VALUE) {
        s_chData[INA303_CH_IV].rawAdc = sample;
    } else {
        s_faultFlags |= INA303_FAULT_ADC_ERR;
    }

    sample = s_adc1InjRaw[INA303_ADC1_INJ_RANK_IW];
    if (sample <= INA303_ADC_MAX_VALUE) {
        s_chData[INA303_CH_IW].rawAdc = sample;
    } else {
        s_faultFlags |= INA303_FAULT_ADC_ERR;
    }
}

static void INA303_ProcessADC2InjData(void)
{
    uint16_t sample;

    sample = s_adc2InjRaw[INA303_ADC2_INJ_RANK_BEMF_A];
    if (sample <= INA303_ADC_MAX_VALUE) {
        s_chData[INA303_CH_BEMF_A].rawAdc = sample;
    } else {
        s_faultFlags |= INA303_FAULT_ADC_ERR;
    }

    sample = s_adc2InjRaw[INA303_ADC2_INJ_RANK_BEMF_B];
    if (sample <= INA303_ADC_MAX_VALUE) {
        s_chData[INA303_CH_BEMF_B].rawAdc = sample;
    } else {
        s_faultFlags |= INA303_FAULT_ADC_ERR;
    }

    sample = s_adc2InjRaw[INA303_ADC2_INJ_RANK_BEMF_C];
    if (sample <= INA303_ADC_MAX_VALUE) {
        s_chData[INA303_CH_BEMF_C].rawAdc = sample;
    } else {
        s_faultFlags |= INA303_FAULT_ADC_ERR;
    }
}

/**
 * @brief  ADC3 规则组软件轮询（1kHz，非阻塞，适合中断上下文）
 * @note   直接查询 EOC 标志，避免 HAL_PollForConversion 的 ms 级阻塞
 */
static void INA303_PollADC3(void)
{
    uint16_t val;
    uint32_t timeout;

    /* 启动 ADC3 规则组扫描 */
    if (HAL_ADC_Start(&INA303_ADC3_HANDLE) != HAL_OK) {
        return;
    }

    /* 等待 Rank1 (Vbus) EOC —— 非阻塞软件超时 */
    timeout = 1000;
    while (!(__HAL_ADC_GET_FLAG(&INA303_ADC3_HANDLE, ADC_FLAG_EOC)) && timeout--) {}
    if (timeout) {
        val = (uint16_t)HAL_ADC_GetValue(&INA303_ADC3_HANDLE);
        if (val <= INA303_ADC_MAX_VALUE) {
            s_chData[INA303_CH_VBUS].rawAdc = val;
        } else {
            s_faultFlags |= INA303_FAULT_ADC_ERR;
        }
    }

    /* 等待 Rank2 (Temp) EOC —— 非阻塞软件超时 */
    timeout = 1000;
    while (!(__HAL_ADC_GET_FLAG(&INA303_ADC3_HANDLE, ADC_FLAG_EOC)) && timeout--) {}
    if (timeout) {
        val = (uint16_t)HAL_ADC_GetValue(&INA303_ADC3_HANDLE);
        if (val <= INA303_ADC_MAX_VALUE) {
            s_chData[INA303_CH_TEMP].rawAdc = val;
        } else {
            s_faultFlags |= INA303_FAULT_ADC_ERR;
        }
    }

    HAL_ADC_Stop(&INA303_ADC3_HANDLE);
}

/* =======================================================================================================
 * 滤波与物理量计算
 * ======================================================================================================= */

static void INA303_UpdateFilter(INA303_ChannelIndex_t ch)
{
    INA303_ChannelData_t *pCh = &s_chData[ch];
    uint16_t newSample = pCh->rawAdc;

    pCh->filterSum -= pCh->filterBuf[pCh->filterIdx];
    pCh->filterBuf[pCh->filterIdx] = newSample;
    pCh->filterSum += newSample;

    pCh->filterIdx++;
    if (pCh->filterIdx >= INA303_FILTER_SIZE) {
        pCh->filterIdx = 0;
        pCh->filterReady = 1;
    }

    pCh->filteredAdc = (uint16_t)(pCh->filterSum >> INA303_FILTER_SHIFT);
}

static void INA303_CalculatePhysical(INA303_ChannelIndex_t ch)
{
    INA303_ChannelData_t *pCh = &s_chData[ch];
    int32_t adcDiff;
    float result;

    if (!pCh->filterReady) {
        return;
    }

    adcDiff = (int32_t)pCh->filteredAdc - pCh->offsetAdc;

    switch (s_chConfig[ch].type) {
        case INA303_TYPE_CURRENT:
            result = (float)adcDiff * s_chConfig[ch].scaleFactor * pCh->gainCal;
            break;

        case INA303_TYPE_BEMF:
            /* BEMF：单向电压，不允许负值 */
            if (adcDiff < 0) {
                adcDiff = 0;
            }
            result = (float)adcDiff * s_chConfig[ch].scaleFactor * pCh->gainCal;
            break;

        case INA303_TYPE_VOLTAGE:
            /* Vbus：单向电压，不允许负值 */
            if (adcDiff < 0) {
                adcDiff = 0;
            }
            result = (float)adcDiff * s_chConfig[ch].scaleFactor * pCh->gainCal;
            break;

        case INA303_TYPE_TEMP:
        default:
            result = (float)adcDiff * s_chConfig[ch].scaleFactor * pCh->gainCal;
            break;
    }

    pCh->physicalValue = result;
}

/* =======================================================================================================
 * 保护检测
 * ======================================================================================================= */

static void INA303_CheckProtection(void)
{
    INA303_ChannelData_t *pCh;
    float val;
    float ocRelease, ucRelease;
    uint8_t i;

    /* 三相电流：过流 / 欠流 */
    for (i = 0; i < 3; i++) {
        pCh = &s_chData[i];
        if (!pCh->filterReady) {
            continue;
        }
        val = pCh->physicalValue;

        ocRelease = INA303_OC_THRESHOLD_A * INA303_PROT_HYSTERESIS;
        if (val > INA303_OC_THRESHOLD_A) {
            if (pCh->protDebounceOC < INA303_PROT_DEBOUNCE_CNT) {
                pCh->protDebounceOC++;
            } else {
                pCh->protFlagOC = 1;
                s_faultFlags |= INA303_FAULT_OVER_CUR;
            }
        } else if (val < ocRelease) {
            if (pCh->protDebounceOC > 0) {
                pCh->protDebounceOC--;
            }
            if (pCh->protDebounceOC == 0) {
                pCh->protFlagOC = 0;
            }
        }

        ucRelease = INA303_UC_THRESHOLD_A * INA303_PROT_HYSTERESIS;
        if (val < INA303_UC_THRESHOLD_A) {
            if (pCh->protDebounceUC < INA303_PROT_DEBOUNCE_CNT) {
                pCh->protDebounceUC++;
            } else {
                pCh->protFlagUC = 1;
                s_faultFlags |= INA303_FAULT_UNDER_CUR;
            }
        } else if (val > ucRelease) {
            if (pCh->protDebounceUC > 0) {
                pCh->protDebounceUC--;
            }
            if (pCh->protDebounceUC == 0) {
                pCh->protFlagUC = 0;
            }
        }
    }

    /* 母线电压：过压 / 欠压 */
    pCh = &s_chData[INA303_CH_VBUS];
    if (!pCh->filterReady) {
        return;
    }
    val = pCh->physicalValue;

    if (val > INA303_OV_THRESHOLD_V) {
        if (pCh->protDebounceOV < INA303_PROT_DEBOUNCE_CNT) {
            pCh->protDebounceOV++;
        } else {
            pCh->protFlagOV = 1;
            s_faultFlags |= INA303_FAULT_OVER_VOLT;
        }
    } else if (val < (INA303_OV_THRESHOLD_V * INA303_PROT_HYSTERESIS)) {
        if (pCh->protDebounceOV > 0) {
            pCh->protDebounceOV--;
        }
        if (pCh->protDebounceOV == 0) {
            pCh->protFlagOV = 0;
        }
    }

    if (val < INA303_UV_THRESHOLD_V) {
        if (pCh->protDebounceUV < INA303_PROT_DEBOUNCE_CNT) {
            pCh->protDebounceUV++;
        } else {
            pCh->protFlagUV = 1;
            s_faultFlags |= INA303_FAULT_UNDER_VOLT;
        }
    } else if (val > (INA303_UV_THRESHOLD_V / INA303_PROT_HYSTERESIS)) {
        if (pCh->protDebounceUV > 0) {
            pCh->protDebounceUV--;
        }
        if (pCh->protDebounceUV == 0) {
            pCh->protFlagUV = 0;
        }
    }
}

/* =======================================================================================================
 * 获取接口
 * ======================================================================================================= */

float BSP_INA303_GetCurrentU(void)    { return s_chData[INA303_CH_IU].physicalValue; }
float BSP_INA303_GetCurrentV(void)    { return s_chData[INA303_CH_IV].physicalValue; }
float BSP_INA303_GetCurrentW(void)    { return s_chData[INA303_CH_IW].physicalValue; }
float BSP_INA303_GetBemfA(void)       { return s_chData[INA303_CH_BEMF_A].physicalValue; }
float BSP_INA303_GetBemfB(void)       { return s_chData[INA303_CH_BEMF_B].physicalValue; }
float BSP_INA303_GetBemfC(void)       { return s_chData[INA303_CH_BEMF_C].physicalValue; }
float BSP_INA303_GetBusVoltage(void)  { return s_chData[INA303_CH_VBUS].physicalValue; }
float BSP_INA303_GetTemperature(void) { return s_chData[INA303_CH_TEMP].physicalValue; }

uint16_t BSP_INA303_GetRaw(INA303_ChannelIndex_t ch)
{
    if (ch >= INA303_CH_NUM) {
        return 0;
    }
    return s_chData[ch].rawAdc;
}

uint16_t BSP_INA303_GetFiltered(INA303_ChannelIndex_t ch)
{
    if (ch >= INA303_CH_NUM) {
        return 0;
    }
    return s_chData[ch].filteredAdc;
}

/* =======================================================================================================
 * 校准接口
 * ======================================================================================================= */

void BSP_INA303_CalibrateZeroStart(void)
{
    if (s_calState != INA303_CAL_IDLE) {
        return;
    }
    s_calState = INA303_CAL_RUNNING;
    s_calChIndex = 0;
    s_calSampleCnt = 0;
    s_calAccSum = 0;
}

INA303_CalState_t BSP_INA303_CalibrateZeroPoll(void)
{
    uint8_t ch;

    if (s_calState != INA303_CAL_RUNNING) {
        return s_calState;
    }

    for (ch = s_calChIndex; ch < INA303_CH_NUM; ch++) {
        if (s_chConfig[ch].type != INA303_TYPE_CURRENT) {
            continue;
        }
        if (s_chData[ch].filterReady) {
            s_calAccSum += s_chData[ch].filteredAdc;
            s_calSampleCnt++;
            if (s_calSampleCnt >= INA303_CAL_SAMPLE_NUM) {
                s_chData[ch].offsetAdc = (int16_t)(s_calAccSum >> INA303_CAL_SAMPLE_SHIFT);
                s_calAccSum = 0;
                s_calSampleCnt = 0;
                s_calChIndex = ch + 1;
            }
            break;
        }
    }

    if (s_calChIndex >= INA303_CH_NUM) {
        s_calState = INA303_CAL_DONE;
    }
    return s_calState;
}

void BSP_INA303_SetGainCal(INA303_ChannelIndex_t ch, float gainCal)
{
    if (ch >= INA303_CH_NUM) {
        return;
    }
    if (gainCal < 0.5f || gainCal > 1.5f) {
        return;
    }
    s_chData[ch].gainCal = gainCal;
}

float BSP_INA303_GetGainCal(INA303_ChannelIndex_t ch)
{
    if (ch >= INA303_CH_NUM) {
        return 1.0f;
    }
    return s_chData[ch].gainCal;
}

/* =======================================================================================================
 * 故障接口
 * ======================================================================================================= */

uint8_t BSP_INA303_GetFaultFlags(void)
{
    return s_faultFlags;
}

void BSP_INA303_ClearFaults(void)
{
    uint8_t i;
    s_faultFlags = 0;
    for (i = 0; i < INA303_CH_NUM; i++) {
        s_chData[i].protFlagOC = 0;
        s_chData[i].protFlagUC = 0;
        s_chData[i].protFlagOV = 0;
        s_chData[i].protFlagUV = 0;
        s_chData[i].protDebounceOC = 0;
        s_chData[i].protDebounceUC = 0;
        s_chData[i].protDebounceOV = 0;
        s_chData[i].protDebounceUV = 0;
    }
}

uint8_t BSP_INA303_IsOverCurrent(void)
{
    return (s_faultFlags & INA303_FAULT_OVER_CUR) ? 1 : 0;
}

uint8_t BSP_INA303_IsUnderCurrent(void)
{
    return (s_faultFlags & INA303_FAULT_UNDER_CUR) ? 1 : 0;
}

uint8_t BSP_INA303_IsOverVoltage(void)
{
    return (s_faultFlags & INA303_FAULT_OVER_VOLT) ? 1 : 0;
}

uint8_t BSP_INA303_IsUnderVoltage(void)
{
    return (s_faultFlags & INA303_FAULT_UNDER_VOLT) ? 1 : 0;
}

/* =======================================================================================================
 * 动态过流阈值接口
 * ======================================================================================================= */

void BSP_INA303_SetOCThreshold(float thresholdA)
{
    if (thresholdA < 1.0f || thresholdA > 500.0f) {
        return;     /* 异常值拒绝写入 */
    }
    s_ocThresholdA = thresholdA;
}

float BSP_INA303_GetOCThreshold(void)
{
    return s_ocThresholdA;
}

/***************************** (END OF FILE) *********************************/
