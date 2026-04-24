/*
*********************************************************************************************************
*   模块名称 : INA303电流/电压采集驱动模块
*   文件名称 : bsp_ina303.c
*   版    本 : V1.0
*   说    明 : 高可靠性、高可移植性INA303A1驱动实现
*              - 3路双向电流（U/V/W相）+ 1路母线电压
*              - ADC DMA连续采集，双缓冲处理
*              - 滑动平均滤波，零偏校准，增益校准
*              - 软件过流/欠流/过压/欠压保护（消抖+迟滞）
*              - 发电阶段反电动势兼容（双向电流检测）
*********************************************************************************************************
*/

#include "../../bsp/inc/bsp_ina303.h"
#include "../../bsp/bsp.h"
#include <string.h>
#include <math.h>

/* =======================================================================================================
 * 内部宏定义
 * ======================================================================================================= */

/* 电流转换系数：A/LSB = Vref / 4096 / (Rsense * Gain) */
#define INA303_CURR_SCALE_A_PER_LSB     ((INA303_VREF_V) / (INA303_ADC_RESOLUTION) / ((INA303_RSENSE_OHM) * (INA303_GAIN_VV)))

/* 电压转换系数：V/LSB = Vref / 4096 / 分压比 */
#define INA303_VOLT_SCALE_V_PER_LSB     ((INA303_VREF_V) / (INA303_ADC_RESOLUTION) / (INA303_BUS_DIV_RATIO))

/* ADC最大有效值（12位） */
#define INA303_ADC_MAX_VALUE            4095

/* 零偏校准采样次数（2^n，用于求平均） */
#define INA303_CAL_SAMPLE_SHIFT         6           /* 64次采样 */
#define INA303_CAL_SAMPLE_NUM           (1U << INA303_CAL_SAMPLE_SHIFT)

/* =======================================================================================================
 * 内部数据结构
 * ======================================================================================================= */

/* 单通道运行时数据 */
typedef struct {
    uint16_t rawAdc;                /* 最新原始ADC值 */
    uint16_t filteredAdc;           /* 滤波后ADC值 */
    float    physicalValue;         /* 物理量（电流A或电压V） */
    int16_t  offsetAdc;             /* 零偏校准值（0A或0V对应的ADC值） */
    float    gainCal;               /* 增益校准系数 */

    /* 滑动平均滤波器 */
    uint16_t filterBuf[INA303_FILTER_SIZE];
    uint32_t filterSum;
    uint8_t  filterIdx;
    uint8_t  filterReady;           /* 滤波器已填满标志 */

    /* 保护状态 */
    uint16_t protDebounceOC;        /* 过流消抖计数 */
    uint16_t protDebounceUC;        /* 欠流消抖计数 */
    uint16_t protDebounceOV;        /* 过压消抖计数 */
    uint16_t protDebounceUV;        /* 欠压消抖计数 */
    uint8_t  protFlagOC;            /* 过流标志 */
    uint8_t  protFlagUC;            /* 欠流标志 */
    uint8_t  protFlagOV;            /* 过压标志 */
    uint8_t  protFlagUV;            /* 欠压标志 */
} INA303_ChannelData_t;

/* 通道静态配置表（索引顺序与INA303_ChannelIndex_t一致） */
typedef struct {
    uint8_t  adcRank;               /* 在ADC扫描序列/DMA缓冲区中的位置 */
    uint8_t  type;                  /* 0=电流, 1=电压 */
    int16_t  defaultOffset;         /* 默认零偏 */
    float    scaleFactor;           /* 转换系数 */
} INA303_ChannelConfig_t;

static const INA303_ChannelConfig_t s_chConfig[INA303_CH_NUM] = {
    /* U相电流 */
    {
        .adcRank        = INA303_SCAN_IDX_IU,
        .type           = INA303_TYPE_CURRENT,
        .defaultOffset  = INA303_DEFAULT_OFFSET_ADC,
        .scaleFactor    = INA303_CURR_SCALE_A_PER_LSB
    },
    /* V相电流 */
    {
        .adcRank        = INA303_SCAN_IDX_IV,
        .type           = INA303_TYPE_CURRENT,
        .defaultOffset  = INA303_DEFAULT_OFFSET_ADC,
        .scaleFactor    = INA303_CURR_SCALE_A_PER_LSB
    },
    /* W相电流 */
    {
        .adcRank        = INA303_SCAN_IDX_IW,
        .type           = INA303_TYPE_CURRENT,
        .defaultOffset  = INA303_DEFAULT_OFFSET_ADC,
        .scaleFactor    = INA303_CURR_SCALE_A_PER_LSB
    },
    /* 母线电压 */
    {
        .adcRank        = INA303_SCAN_IDX_VBUS,
        .type           = INA303_TYPE_VOLTAGE,
        .defaultOffset  = 0,        /* 0V时ADC理论值为0 */
        .scaleFactor    = INA303_VOLT_SCALE_V_PER_LSB
    }
};

/* =======================================================================================================
 * 内部变量
 * ======================================================================================================= */

/* DMA缓冲区（双缓冲，必须对齐到32位以适配DMA） */
#if INA303_DMA_BUFFER_DEPTH >= 2
static uint16_t s_dmaBuffer[INA303_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
#else
static uint16_t s_dmaBuffer[INA303_DMA_BUFFER_SIZE];
#endif

/* 通道运行时数据 */
static INA303_ChannelData_t s_chData[INA303_CH_NUM];

/* DMA状态标志（volatile，在中断和主循环间同步） */
static volatile uint8_t s_dmaHalfReady = 0;
static volatile uint8_t s_dmaFullReady = 0;

/* 全局故障标志 */
static volatile uint8_t s_faultFlags = 0;

/* 初始化完成标志 */
static uint8_t s_initDone = 0;

/* 零偏校准状态 */
static INA303_CalState_t s_calState = INA303_CAL_IDLE;
static uint8_t s_calChIndex = 0;
static uint16_t s_calSampleCnt = 0;
static int32_t s_calAccSum = 0;

/* =======================================================================================================
 * 内部函数声明
 * ======================================================================================================= */
static void INA303_ProcessBuffer(const uint16_t *pBuffer);
static void INA303_UpdateFilter(INA303_ChannelIndex_t ch);
static void INA303_CalculatePhysical(INA303_ChannelIndex_t ch);
static void INA303_CheckProtection(void);

/* =======================================================================================================
 * 函数实现
 * ======================================================================================================= */

/**
 * @brief  初始化INA303驱动
 * @note   调用本函数前，必须已在CUBEMX中配置好ADC和DMA，并生成代码。
 *         本函数会启动ADC DMA循环传输。
 */
void BSP_INA303_Init(void)
{
    uint8_t i, j;

    /* 清零所有运行时数据 */
    memset(s_chData, 0, sizeof(s_chData));
    memset(s_dmaBuffer, 0, sizeof(s_dmaBuffer));
    s_dmaHalfReady = 0;
    s_dmaFullReady = 0;
    s_faultFlags = 0;
    s_initDone = 0;
    s_calState = INA303_CAL_IDLE;

    /* 初始化各通道默认值 */
    for (i = 0; i < INA303_CH_NUM; i++) {
        s_chData[i].offsetAdc = s_chConfig[i].defaultOffset;
        s_chData[i].gainCal   = INA303_DEFAULT_GAIN_CAL;

        /* 预填充滤波器（避免启动时输出跳变） */
        uint16_t initVal = (s_chConfig[i].type == INA303_TYPE_CURRENT)
                           ? (uint16_t)s_chConfig[i].defaultOffset
                           : 0U;
        for (j = 0; j < INA303_FILTER_SIZE; j++) {
            s_chData[i].filterBuf[j] = initVal;
        }
        s_chData[i].filterSum = (uint32_t)initVal * INA303_FILTER_SIZE;
        s_chData[i].filterIdx = 0;
        s_chData[i].filterReady = 0;
    }

    /* 启动ADC DMA循环传输 */
    HAL_ADC_Start_DMA(&INA303_ADC_HANDLE, (uint32_t *)s_dmaBuffer, INA303_DMA_BUFFER_SIZE);

    s_initDone = 1;
}

/**
 * @brief  DMA半传输完成回调（需在HAL_ADC_ConvHalfCpltCallback中调用）
 */
void BSP_INA303_DMAHalfCplt(void)
{
    if (!s_initDone) {
        return;
    }
    INA303_ProcessBuffer(&s_dmaBuffer[0]);
    s_dmaHalfReady = 1;
}

/**
 * @brief  DMA传输完成回调（需在HAL_ADC_ConvCpltCallback中调用）
 */
void BSP_INA303_DMACplt(void)
{
    if (!s_initDone) {
        return;
    }
    INA303_ProcessBuffer(&s_dmaBuffer[INA303_ADC_CHANNEL_NUM]);
    s_dmaFullReady = 1;
}

/**
 * @brief  处理DMA缓冲区数据（提取各通道原始值）
 * @param  pBuffer: 缓冲区首地址（长度为INA303_ADC_CHANNEL_NUM）
 */
static void INA303_ProcessBuffer(const uint16_t *pBuffer)
{
    uint8_t i;
    uint8_t rank;
    uint16_t sample;

    for (i = 0; i < INA303_CH_NUM; i++) {
        rank = s_chConfig[i].adcRank;
        sample = pBuffer[rank];

        /* ADC数值有效性检查（12位ADC不应超过4095） */
        if (sample > INA303_ADC_MAX_VALUE) {
            s_faultFlags |= INA303_FAULT_ADC_ERR;
            continue;
        }

        s_chData[i].rawAdc = sample;
    }
}

/**
 * @brief  快速循环任务（建议在1kHz~10kHz中断中调用）
 * @note   本函数执行滤波、物理量计算和保护检测。调用频率越高，消抖响应越快。
 */
void BSP_INA303_FastTask(void)
{
    uint8_t i;
    uint8_t newData = 0;

    if (!s_initDone) {
        return;
    }

    /* 检查是否有新DMA数据（原子操作读取标志） */
    DISABLE_INT();
    if (s_dmaHalfReady || s_dmaFullReady) {
        s_dmaHalfReady = 0;
        s_dmaFullReady = 0;
        newData = 1;
    }
    ENABLE_INT();

    /* 无新数据则跳过 */
    if (!newData) {
        return;
    }

    /* 逐通道更新滤波器和计算物理量 */
    for (i = 0; i < INA303_CH_NUM; i++) {
        INA303_UpdateFilter((INA303_ChannelIndex_t)i);
        INA303_CalculatePhysical((INA303_ChannelIndex_t)i);
    }

    /* 保护检测 */
    INA303_CheckProtection();

    /* 零偏校准轮询（如果处于校准模式） */
    if (s_calState == INA303_CAL_RUNNING) {
        (void)BSP_INA303_CalibrateZeroPoll();
    }
}

/**
 * @brief  更新滑动平均滤波器
 */
static void INA303_UpdateFilter(INA303_ChannelIndex_t ch)
{
    INA303_ChannelData_t *pCh = &s_chData[ch];
    uint16_t newSample = pCh->rawAdc;

    /* 移除最老的值，加入最新值 */
    pCh->filterSum -= pCh->filterBuf[pCh->filterIdx];
    pCh->filterBuf[pCh->filterIdx] = newSample;
    pCh->filterSum += newSample;

    pCh->filterIdx++;
    if (pCh->filterIdx >= INA303_FILTER_SIZE) {
        pCh->filterIdx = 0;
        pCh->filterReady = 1;   /* 滤波器已满，输出有效 */
    }

    pCh->filteredAdc = (uint16_t)(pCh->filterSum >> INA303_FILTER_SHIFT);
}

/**
 * @brief  计算物理量（电流A或电压V）
 */
static void INA303_CalculatePhysical(INA303_ChannelIndex_t ch)
{
    INA303_ChannelData_t *pCh = &s_chData[ch];
    int32_t adcDiff;
    float result;

    if (!pCh->filterReady) {
        return;     /* 滤波器未填满，暂不计算 */
    }

    adcDiff = (int32_t)pCh->filteredAdc - pCh->offsetAdc;

    if (s_chConfig[ch].type == INA303_TYPE_CURRENT) {
        /* 电流 = (ADC - offset) × 转换系数 × 增益校准 */
        result = (float)adcDiff * s_chConfig[ch].scaleFactor * pCh->gainCal;
    } else {
        /* 电压：不允许负值（分压电路不可能输出负电压） */
        if (adcDiff < 0) {
            adcDiff = 0;
        }
        result = (float)adcDiff * s_chConfig[ch].scaleFactor * pCh->gainCal;
    }

    pCh->physicalValue = result;
}

/**
 * @brief  保护检测（过流/欠流/过压/欠压）
 * @note   采用独立消抖计数 + 迟滞释放，避免边界抖动导致误动作
 */
static void INA303_CheckProtection(void)
{
    INA303_ChannelData_t *pCh;
    float val;
    float ocRelease;
    float ucRelease;
    uint8_t i;

    /* 检测三相电流（过流/欠流） */
    for (i = 0; i < 3; i++) {   /* IU, IV, IW */
        pCh = &s_chData[i];
        if (!pCh->filterReady) {
            continue;
        }
        val = pCh->physicalValue;

        /* 过流检测 */
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

        /* 欠流检测（负向过流，发电工况） */
        ucRelease = INA303_UC_THRESHOLD_A * INA303_PROT_HYSTERESIS; /* 负值×0.9 = 更接近0 */
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

    /* 母线电压检测（过压/欠压） */
    pCh = &s_chData[INA303_CH_VBUS];
    if (!pCh->filterReady) {
        return;
    }
    val = pCh->physicalValue;

    /* 过压 */
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

    /* 欠压 */
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
 * 获取接口实现
 * ======================================================================================================= */

float BSP_INA303_GetCurrentU(void)  { return s_chData[INA303_CH_IU].physicalValue; }
float BSP_INA303_GetCurrentV(void)  { return s_chData[INA303_CH_IV].physicalValue; }
float BSP_INA303_GetCurrentW(void)  { return s_chData[INA303_CH_IW].physicalValue; }
float BSP_INA303_GetBusVoltage(void){ return s_chData[INA303_CH_VBUS].physicalValue; }

uint16_t BSP_INA303_GetRawU(void)   { return s_chData[INA303_CH_IU].rawAdc; }
uint16_t BSP_INA303_GetRawV(void)   { return s_chData[INA303_CH_IV].rawAdc; }
uint16_t BSP_INA303_GetRawW(void)   { return s_chData[INA303_CH_IW].rawAdc; }
uint16_t BSP_INA303_GetRawVbus(void){ return s_chData[INA303_CH_VBUS].rawAdc; }

uint16_t BSP_INA303_GetFilteredU(void)    { return s_chData[INA303_CH_IU].filteredAdc; }
uint16_t BSP_INA303_GetFilteredV(void)    { return s_chData[INA303_CH_IV].filteredAdc; }
uint16_t BSP_INA303_GetFilteredW(void)    { return s_chData[INA303_CH_IW].filteredAdc; }
uint16_t BSP_INA303_GetFilteredVbus(void) { return s_chData[INA303_CH_VBUS].filteredAdc; }

/* =======================================================================================================
 * 校准接口实现
 * ======================================================================================================= */

/**
 * @brief  启动零偏校准
 * @note   调用前必须确保三相电流均为0A（电机停转、无负载），否则校准结果错误。
 *         校准完成后，offset会自动写入各电流通道。
 */
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

/**
 * @brief  轮询零偏校准状态（非阻塞，在FastTask中自动调用，也可在主循环手动轮询）
 * @retval INA303_CAL_IDLE     空闲
 * @retval INA303_CAL_RUNNING  校准中
 * @retval INA303_CAL_DONE     校准完成
 */
INA303_CalState_t BSP_INA303_CalibrateZeroPoll(void)
{
    uint8_t ch;

    if (s_calState != INA303_CAL_RUNNING) {
        return s_calState;
    }

    /* 只对电流通道进行校准 */
    for (ch = s_calChIndex; ch < INA303_CH_NUM; ch++) {
        if (s_chConfig[ch].type != INA303_TYPE_CURRENT) {
            continue;
        }

        /* 累加当前通道的滤波后ADC值 */
        if (s_chData[ch].filterReady) {
            s_calAccSum += s_chData[ch].filteredAdc;
            s_calSampleCnt++;

            if (s_calSampleCnt >= INA303_CAL_SAMPLE_NUM) {
                /* 本通道完成，计算平均值 */
                s_chData[ch].offsetAdc = (int16_t)(s_calAccSum >> INA303_CAL_SAMPLE_SHIFT);

                /* 切换下一通道 */
                s_calAccSum = 0;
                s_calSampleCnt = 0;
                s_calChIndex = ch + 1;
            }
            break;  /* 每次只处理一个通道的一帧数据 */
        }
    }

    /* 检查是否所有电流通道都已完成 */
    if (s_calChIndex >= INA303_CH_NUM) {
        s_calState = INA303_CAL_DONE;
    }

    return s_calState;
}

/**
 * @brief  设置指定通道的增益校准系数
 * @param  ch: 通道索引
 * @param  gainCal: 校准系数（1.0为无修正，0.98表示实际增益比理论小2%）
 */
void BSP_INA303_SetGainCal(INA303_ChannelIndex_t ch, float gainCal)
{
    if (ch >= INA303_CH_NUM) {
        return;
    }
    if (gainCal < 0.5f || gainCal > 1.5f) {
        /* 增益系数异常，拒绝写入（防止误操作） */
        return;
    }
    s_chData[ch].gainCal = gainCal;
}

/**
 * @brief  获取指定通道的增益校准系数
 */
float BSP_INA303_GetGainCal(INA303_ChannelIndex_t ch)
{
    if (ch >= INA303_CH_NUM) {
        return 1.0f;
    }
    return s_chData[ch].gainCal;
}

/* =======================================================================================================
 * 故障接口实现
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
 * 硬件比较器预留（Alert引脚中断回调）
 * ======================================================================================================= */

/**
 * @brief  Alert1中断回调（如接入硬件比较器输出，在对应EXTI中断服务函数中调用）
 * @note   可用于硬件级快速过流保护（响应时间约1us）
 */
void BSP_INA303_Alert1_IRQHandler(void)
{
    /* 硬件快速保护：直接置位过流标志 */
    s_faultFlags |= INA303_FAULT_OVER_CUR;
}

/**
 * @brief  Alert2中断回调（如接入硬件比较器输出，在对应EXTI中断服务函数中调用）
 */
void BSP_INA303_Alert2_IRQHandler(void)
{
    /* 根据实际硬件连接定义保护类型，此处预留 */
    /* s_faultFlags |= INA303_FAULT_UNDER_CUR; */
}

/***************************** (END OF FILE) *********************************/
