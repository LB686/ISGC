/**
 * @brief    启发一体电机控制器状态机核心实现
 * @file     app_motor_ctrl.c
 * @details  状态机驱动框架 + 各状态钩子函数骨架
 * @author   LB
 * @version  V1.0
 * @date     2026/04/17
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

#include "../../app/app_motor_ctrl.h"

/* Private define ------------------------------------------------------------*/
#define EVENT_QUEUE_SIZE    8

/* Private variables ---------------------------------------------------------*/
static MotorEvent_E s_EventQueue[EVENT_QUEUE_SIZE];
static uint8_t s_EventHead = 0;
static uint8_t s_EventTail = 0;
static uint8_t s_OffsetCalibCnt = 0;
static uint32_t s_OffsetSumIa = 0;
static uint32_t s_OffsetSumIb = 0;
static uint32_t s_OffsetSumIc = 0;

/* Public variables ----------------------------------------------------------*/
MotorCtrl_T g_MotorCtrl = {0};

/* 状态函数表（用于通用调用） */
static const StateFuncPtr StateOnEnterTable[MST_NUM] = {
    MST_Init_OnEnter,
    MST_Standby_OnEnter,
    MST_IfStart_OnEnter,
    MST_FocRun_OnEnter,
    MST_GenReady_OnEnter,
    MST_GenRun_OnEnter,
    MST_Fault_OnEnter
};

static const StateFuncPtr StateOnRunTable[MST_NUM] = {
    MST_Init_OnRun,
    MST_Standby_OnRun,
    MST_IfStart_OnRun,
    MST_FocRun_OnRun,
    MST_GenReady_OnRun,
    MST_GenRun_OnRun,
    MST_Fault_OnRun
};

static const StateFuncPtr StateOnExitTable[MST_NUM] = {
    MST_Init_OnExit,
    MST_Standby_OnExit,
    MST_IfStart_OnExit,
    MST_FocRun_OnExit,
    MST_GenReady_OnExit,
    MST_GenRun_OnExit,
    MST_Fault_OnExit
};

/**
 * @brief 状态转移表
 * @note  行：当前状态，列：事件，值：目标状态
 *        0xFF 表示不转移（忽略该事件）
 */
static const uint8_t StateTransitionTable[MST_NUM][EVT_NUM] = {
    /*                    NONE  INIT_D  START  STOP   IF_TMO SMO_C  STR_T  IGN_O  BEMF_S  OC_HW  OC_SW  OV     UV     OH     STALL  FAULT_CLR */
    /* MST_INIT      */ {0xFF, 0x01,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,   0x06,  0x06,  0x06,  0x06,  0x06,  0x06,  0xFF },
    /* MST_STANDBY   */ {0xFF, 0xFF,  0x02,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,   0x06,  0x06,  0x06,  0x06,  0x06,  0x06,  0xFF },
    /* MST_IF_START  */ {0xFF, 0xFF,  0xFF,  0x01,  0x06,  0x03,  0xFF,  0xFF,  0xFF,   0x06,  0x06,  0x06,  0x06,  0x06,  0x06,  0xFF },
    /* MST_FOC_RUN   */ {0xFF, 0xFF,  0xFF,  0x01,  0xFF,  0xFF,  0x04,  0x04,  0xFF,   0x06,  0x06,  0x06,  0x06,  0x06,  0x06,  0xFF },
    /* MST_GEN_READY */ {0xFF, 0xFF,  0xFF,  0x01,  0xFF,  0xFF,  0xFF,  0xFF,  0x05,   0x06,  0x06,  0x06,  0x06,  0x06,  0x06,  0xFF },
    /* MST_GEN_RUN   */ {0xFF, 0xFF,  0xFF,  0x01,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,   0x06,  0x06,  0x06,  0x06,  0x06,  0x06,  0xFF },
    /* MST_FAULT     */ {0xFF, 0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,   0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,  0x01 }
};

/* Private function prototypes -----------------------------------------------*/
static MotorEvent_E GetNextEvent(void);
static void ClearAllEvents(void);
static void EnterState(MotorState_E newState);

/* Public functions ----------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: MotorCtrl_Init
*   功能说明: 状态机全局初始化
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void MotorCtrl_Init(void)
{
    memset(&g_MotorCtrl, 0, sizeof(g_MotorCtrl));
    g_MotorCtrl.CurState = MST_INIT;
    g_MotorCtrl.FaultCode = ERR_NONE;
    
    s_EventHead = 0;
    s_EventTail = 0;
    s_OffsetCalibCnt = 0;
    s_OffsetSumIa = 0;
    s_OffsetSumIb = 0;
    s_OffsetSumIc = 0;
    
    /* 调用INIT状态的OnEnter */
    StateOnEnterTable[MST_INIT]();
}

/*
*********************************************************************************************************
*   函 数 名: MotorCtrl_PostEvent
*   功能说明: 向状态机投递事件（可在中断或主循环中调用）
*   形    参: evt -- 事件类型
*   返 回 值: 无
*********************************************************************************************************
*/
void MotorCtrl_PostEvent(MotorEvent_E evt)
{
    uint8_t nextTail = (s_EventTail + 1) % EVENT_QUEUE_SIZE;
    if (nextTail != s_EventHead) {
        s_EventQueue[s_EventTail] = evt;
        s_EventTail = nextTail;
    }
    /* 队列满时丢弃事件（理论上不应发生） */
}

/*
*********************************************************************************************************
*   函 数 名: MotorCtrl_StateMachineRun
*   功能说明: 状态机主运行函数，建议放在1kHz慢环中调用
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void MotorCtrl_StateMachineRun(void)
{
    MotorEvent_E evt;
    MotorState_E nextState;
    uint8_t nextStateIdx;
    
    /* 处理所有待处理事件 */
    while ((evt = GetNextEvent()) != EVT_NONE) {
        nextStateIdx = StateTransitionTable[g_MotorCtrl.CurState][evt];
        if (nextStateIdx != 0xFF && nextStateIdx != g_MotorCtrl.CurState) {
            EnterState((MotorState_E)nextStateIdx);
        }
    }
    
    /* 更新状态运行时间 */
    g_MotorCtrl.StateRunTick = get_system_time_ms() - g_MotorCtrl.StateEntryTick;
    
    /* 执行当前状态的OnRun */
    if (g_MotorCtrl.CurState < MST_NUM) {
        StateOnRunTable[g_MotorCtrl.CurState]();
    }
}

/*
*********************************************************************************************************
*   函 数 名: MotorCtrl_FastLoop_ISR
*   功能说明: 30kHz快环中断入口（ADC_JEOC中断中调用）
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void MotorCtrl_FastLoop_ISR(void)
{
    /* 
     * 此处完成：
     * 1. 读取ADC三相电流 + 母线电压
     * 2. Offset校准（INIT状态前N次中断）
     * 3. 坐标变换、观测器、SVPWM
     * 4. 硬件级过流检测（<5us响应）
     */
    
    switch (g_MotorCtrl.CurState) {
        case MST_INIT:
            /* 前N次中断做电流Offset累加 */
            if (s_OffsetCalibCnt < 100) {
                s_OffsetSumIa += ADC1->JDR1;
                s_OffsetSumIb += ADC2->JDR1;
                s_OffsetSumIc += ADC1->JDR2;
                s_OffsetCalibCnt++;
                if (s_OffsetCalibCnt >= 100) {
                    /* Offset校准完成，投递事件 */
                    MotorCtrl_PostEvent(EVT_INIT_DONE);
                }
            }
            PWM_AllOff();
            break;
            
        case MST_IF_START:
            /* IF开环强拖：固定Iq，角度按斜坡递增 */
            IF_OpenLoop_Run();
            break;
            
        case MST_FOC_RUN:
            /* FOC闭环电动：SMO观测器 + 速度环 + 电流环 */
            FOC_Sensorless_Run();
            break;
            
        case MST_GEN_RUN:
            /* 发电主动整流：母线电压环 + SMO + 电流环 */
            FOC_Generating_Run();
            break;
            
        case MST_GEN_READY:
        case MST_STANDBY:
        case MST_FAULT:
        default:
            PWM_AllOff();
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: MotorCtrl_SlowLoop_ISR
*   功能说明: 1kHz慢环中断入口（速度环、电压环、状态机保护逻辑）
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void MotorCtrl_SlowLoop_ISR(void)
{
    /* 读取温度、母线电压（用于保护） */
    g_MotorCtrl.Vbus = Get_Vbus_ADC();
    g_MotorCtrl.TempMos = Get_MosTemp_ADC();
    
    /* 1. 硬件故障检测（Break标志） */
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK)) {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        g_MotorCtrl.FaultCode = ERR_OC_HW;
        MotorCtrl_PostEvent(EVT_FAULT_OC_HW);
    }
    
    /* 2. 软件过流检测（持续） */
    if (fabsf(g_MotorCtrl.Iq) > PROT_OC_CONT_A || 
        fabsf(g_MotorCtrl.Id) > PROT_OC_CONT_A) {
        static uint32_t oc_cont_timer = 0;
        oc_cont_timer++;
        if (oc_cont_timer >= PROT_OC_CONT_TIME_MS) {
            g_MotorCtrl.FaultCode = ERR_OC_SW;
            MotorCtrl_PostEvent(EVT_FAULT_OC_SW);
            oc_cont_timer = 0;
        }
    } else {
        /*  hysteresis清零  */
    }
    
    /* 3. 母线过压/欠压检测 */
    if (g_MotorCtrl.Vbus > GEN_VBUS_MAX_V) {
        g_MotorCtrl.FaultCode = ERR_OV_VBUS;
        MotorCtrl_PostEvent(EVT_FAULT_OV);
    } else if (g_MotorCtrl.Vbus < PROT_UV_VBUS_V && g_MotorCtrl.CurState >= MST_FOC_RUN) {
        g_MotorCtrl.FaultCode = ERR_UV_VBUS;
        MotorCtrl_PostEvent(EVT_FAULT_UV);
    }
    
    /* 4. 过温检测 */
    if (g_MotorCtrl.TempMos > 85.0f) {
        g_MotorCtrl.FaultCode = ERR_OH_MOS;
        MotorCtrl_PostEvent(EVT_FAULT_OH);
    }
    
    /* 5. IF启动超时检测 */
    if (g_MotorCtrl.CurState == MST_IF_START && g_MotorCtrl.StateRunTick > IF_MAX_TIME_MS) {
        g_MotorCtrl.FaultCode = ERR_IF_TIMEOUT;
        MotorCtrl_PostEvent(EVT_IF_TIMEOUT);
    }
    
    /* 6. 4s启动定时检测 */
    if (g_MotorCtrl.CurState == MST_FOC_RUN && g_MotorCtrl.StateRunTick > START_RAMP_TIME_MS) {
        MotorCtrl_PostEvent(EVT_START_TIMEOUT);
    }
    
    /* 7. SMO收敛检测（IF→FOC切换条件） */
    if (g_MotorCtrl.CurState == MST_IF_START) {
        if (g_MotorCtrl.SmoConverged && g_MotorCtrl.SpeedRpm > IF_SWITCH_SPEED_RPM) {
            MotorCtrl_PostEvent(EVT_SMO_CONVERGED);
        }
    }
    
    /* 8. 发电准备延时检测 */
    if (g_MotorCtrl.CurState == MST_GEN_READY && g_MotorCtrl.StateRunTick > GEN_READY_WAIT_MS) {
        if (g_MotorCtrl.BemfMag > 5.0f) {  /* 反电势幅值大于5V认为稳定 */
            MotorCtrl_PostEvent(EVT_BEMF_STABLE);
        }
    }
    
    /* 9. 执行状态机 */
    MotorCtrl_StateMachineRun();
}

/* Private functions ---------------------------------------------------------*/

static MotorEvent_E GetNextEvent(void)
{
    MotorEvent_E evt = EVT_NONE;
    if (s_EventHead != s_EventTail) {
        evt = s_EventQueue[s_EventHead];
        s_EventHead = (s_EventHead + 1) % EVENT_QUEUE_SIZE;
    }
    return evt;
}

static void ClearAllEvents(void)
{
    s_EventHead = 0;
    s_EventTail = 0;
}

static void EnterState(MotorState_E newState)
{
    if (newState >= MST_NUM) return;
    
    /* 执行旧状态Exit */
    StateOnExitTable[g_MotorCtrl.CurState]();
    
    /* 记录历史状态 */
    g_MotorCtrl.PreState = g_MotorCtrl.CurState;
    g_MotorCtrl.CurState = newState;
    g_MotorCtrl.StateEntryTick = get_system_time_ms();
    g_MotorCtrl.StateRunTick = 0;
    
    /* 执行新状态Enter */
    StateOnEnterTable[newState]();
}

/* ==========================================================================
 *  状态钩子函数实现
 * ========================================================================== */

/* -------------------------- MST_INIT -------------------------- */
void MST_Init_OnEnter(void)
{
    PWM_AllOff();
    s_OffsetCalibCnt = 0;
    s_OffsetSumIa = 0;
    s_OffsetSumIb = 0;
    s_OffsetSumIc = 0;
    g_MotorCtrl.FaultCode = ERR_NONE;
}

void MST_Init_OnRun(void)
{
    /* INIT状态在FastLoop_ISR中完成Offset校准，此处无需操作 */
}

void MST_Init_OnExit(void)
{
    /* 保存校准后的Offset值到全局变量 */
    /* g_IaOffset = s_OffsetSumIa / 100; 等 */
}

/* -------------------------- MST_STANDBY -------------------------- */
void MST_Standby_OnEnter(void)
{
    PWM_AllOff();
    g_MotorCtrl.IdRef = 0;
    g_MotorCtrl.IqRef = 0;
    g_MotorCtrl.AngleElec = 0;
}

void MST_Standby_OnRun(void)
{
    /* 检测启动指令（来自CAN/232解析） */
    if (g_MotorCtrl.CmdStart) {
        g_MotorCtrl.CmdStart = 0;
        MotorCtrl_PostEvent(EVT_CMD_START);
    }
}

void MST_Standby_OnExit(void)
{
    /* 清零指令，准备启动 */
    g_MotorCtrl.CmdStart = 0;
}

/* -------------------------- MST_IF_START -------------------------- */
void MST_IfStart_OnEnter(void)
{
    /* IF启动参数初始化 */
    g_MotorCtrl.IdRef = 0;
    g_MotorCtrl.IqRef = IF_START_CURRENT_A;
    g_MotorCtrl.SmoConverged = 0;
    /* 启动斜坡计时器由StateEntryTick记录 */
}

void MST_IfStart_OnRun(void)
{
    /* 检测外部停机指令 */
    if (g_MotorCtrl.CmdStop) {
        g_MotorCtrl.CmdStop = 0;
        MotorCtrl_PostEvent(EVT_CMD_STOP);
    }
}

void MST_IfStart_OnExit(void)
{
    /* 保存当前角度，供FOC无缝衔接 */
    /* g_MotorCtrl.AngleElec = SMO_GetAngle(); */
}

/* -------------------------- MST_FOC_RUN -------------------------- */
void MST_FocRun_OnEnter(void)
{
    /* FOC闭环初始化：载入速度环积分项、角度同步 */
    g_MotorCtrl.IdRef = 0;
    g_MotorCtrl.IqRef = 0;  /* 由速度环输出 */
}

void MST_FocRun_OnRun(void)
{
    /* 速度环PI计算，输出IqRef */
    float spdErr = (float)START_TARGET_SPEED_RPM - g_MotorCtrl.SpeedRpm;
    /* SpeedLoop_PI(spdErr, &g_MotorCtrl.IqRef); */
    
    /* 检测外部信号 */
    if (g_MotorCtrl.CmdStop) {
        g_MotorCtrl.CmdStop = 0;
        MotorCtrl_PostEvent(EVT_CMD_STOP);
    }
    if (g_MotorCtrl.IgnitionDone) {
        g_MotorCtrl.IgnitionDone = 0;
        MotorCtrl_PostEvent(EVT_IGNITION_OK);
    }
}

void MST_FocRun_OnExit(void)
{
    /* 退出电动模式：可记录当前转速 */
}

/* -------------------------- MST_GEN_READY -------------------------- */
void MST_GenReady_OnEnter(void)
{
    /* 全部关PWM，等待反电势稳定 */
    PWM_AllOff();
    g_MotorCtrl.IdRef = 0;
    g_MotorCtrl.IqRef = 0;
}

void MST_GenReady_OnRun(void)
{
    /* 实时检测反电势幅值和频率（由FastLoop中的ADC采样计算） */
    /* g_MotorCtrl.BemfMag = BEMF_CalculateAmplitude(); */
}

void MST_GenReady_OnExit(void)
{
    /* 进入发电前，同步观测器角度与反电势相位 */
    /* SMO_SyncAngle(g_MotorCtrl.AngleElec); */
}

/* -------------------------- MST_GEN_RUN -------------------------- */
void MST_GenRun_OnEnter(void)
{
    /* 发电模式初始化：母线电压环清零 */
    g_MotorCtrl.IdRef = 0;
    g_MotorCtrl.IqRef = 0;  /* 电压环输出负值 */
}

void MST_GenRun_OnRun(void)
{
    /* 母线电压环PI计算，输出IqRef（自动为负） */
    float vbusErr = GEN_TARGET_VBUS_V - g_MotorCtrl.Vbus;
    /* VbusLoop_PI(vbusErr, &g_MotorCtrl.IqRef); */
    /* 限制发电电流 */
    if (g_MotorCtrl.IqRef > 0) g_MotorCtrl.IqRef = 0;
    /* if (g_MotorCtrl.IqRef < -MAX_GEN_CURRENT) g_MotorCtrl.IqRef = -MAX_GEN_CURRENT; */
    
    if (g_MotorCtrl.CmdStop) {
        g_MotorCtrl.CmdStop = 0;
        MotorCtrl_PostEvent(EVT_CMD_STOP);
    }
}

void MST_GenRun_OnExit(void)
{
    /* 退出发电模式 */
}

/* -------------------------- MST_FAULT -------------------------- */
void MST_Fault_OnEnter(void)
{
    PWM_AllOff();
    /* 记录故障发生时的关键数据 */
    /* FaultLog_Save(); */
}

void MST_Fault_OnRun(void)
{
    if (g_MotorCtrl.CmdFaultClr) {
        g_MotorCtrl.CmdFaultClr = 0;
        MotorCtrl_PostEvent(EVT_FAULT_CLR);
    }
}

void MST_Fault_OnExit(void)
{
    g_MotorCtrl.FaultCode = ERR_NONE;
    ClearAllEvents();
}

/* ==========================================================================
 *  占位函数：需在其它模块实现
 * ========================================================================== */
__weak void PWM_AllOff(void)
{
    /* 关闭所有PWM输出，置低电平 */
}

__weak void IF_OpenLoop_Run(void)
{
    /* IF开环强拖：固定Iq，角度斜坡 */
}

__weak void FOC_Sensorless_Run(void)
{
    /* SMO无传感器FOC闭环 */
}

__weak void FOC_Generating_Run(void)
{
    /* 发电模式FOC主动整流 */
}

__weak float Get_Vbus_ADC(void)
{
    return 0.0f;
}

__weak float Get_MosTemp_ADC(void)
{
    return 25.0f;
}

__weak uint32_t get_system_time_ms(void)
{
    return HAL_GetTick();
}

/***************************** (END OF FILE) *********************************/
