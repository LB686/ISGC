/**
 * @brief    电机启动控制模块（无传感器I-F开环启动）
 * @file     app_motor_startup.h
 * @details  适用于启发一体控制器，三相无传感器PMSM启动控制
 *           控制策略：强拖冲击(0~5ms) → 电流环接管(5~50ms) → I-F稳态(50ms~4s) → BEMF检测/发电
 * @author   LB
 * @version  V1.0
 * @date     2026/04/23
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

#ifndef __APP_MOTOR_STARTUP_H__
#define __APP_MOTOR_STARTUP_H__

#include <stdint.h>
#include <stdbool.h>

/* =======================================================================================================
 *  第一节：电机与系统参数配置
 * ======================================================================================================= */

/* ---- 电机本体参数 ---- */
#define MOTOR_POLE_PAIRS            15          /* 电机极对数 */
#define MOTOR_KV_RPM_PER_V          60.0f       /* KV值：每伏特对应60转/分钟 */

/* ---- 启动目标参数 ---- */
#define MOTOR_TARGET_SPEED_RPM      1200.0f     /* 目标机械转速：1200 rpm */
#define MOTOR_TARGET_CURRENT_A      40.0f       /* 持续运行电流指令：40A (Q轴) */
#define MOTOR_STARTUP_TOTAL_TIME_MS 4000        /* 启动阶段总时长：4秒 */

/* ---- 阶段一：强拖冲击 (0 ~ 5ms) ----
 * 说明：启动瞬间施加最大电压矢量，让电流自然冲击到350A（说明书极限值），
 *       利用巨大转矩突破发动机静摩擦和压缩上止点阻力 */
#define MOTOR_STRONG_DRAG_TIME_MS   5           /* 强拖持续时间：5ms（严格限制，防止硬件烧毁） */
#define MOTOR_STRONG_DRAG_VQ_MOD    0.90f       /* Vq调制比 = 0.90（说明书95%占空比限制内） */
#define MOTOR_STRONG_DRAG_FREQ_HZ   200.0f      /* 强拖阶段初始电频率：200Hz（约800rpm同步速） */

/* ---- 阶段二：电流环接管 (5 ~ 50ms) ----
 * 说明：在5ms时无扰切换到电流闭环，PI积分器预置初值防止电压跳变，
 *       Iq电流指令从当前值快速收敛到40A */
#define MOTOR_TAKEOVER_END_MS       50          /* 接管阶段结束时刻：50ms */
#define MOTOR_TAKEOVER_RAMP_MS      30          /* Iq斜坡时间：30ms内从当前值降到40A */

/* ---- 阶段三：I-F稳态运行 (50ms ~ 4000ms) ----
 * 说明：电频率继续爬升到300Hz（对应1200rpm），电流维持40A，
 *       全程电流闭环，电角度开环积分 */
#define MOTOR_STEADY_IQ_A           40.0f       /* 稳态Q轴电流指令 */
#define MOTOR_STEADY_ID_A           0.0f        /* 稳态D轴电流指令：Id=0控制 */
#define MOTOR_STEADY_ELEC_FREQ_HZ   300.0f      /* 目标电频率：300Hz = 1200rpm * 15 / 60 */

/* ---- 电频率爬升加速度 ----
 * 300Hz / 0.15s = 2000 Hz/s²，从0到300Hz约需150ms
 * 加上强拖5ms和接管45ms，总启动时间约200ms，之后稳定运行3800ms */
#define MOTOR_FREQ_ACCEL_HZPS       2000.0f     /* 电频率加速度：2000 Hz/s */

/* ---- 电流环PI参数（需根据实际电机调试优化） ----
 * 说明：PI输出为无量纲调制比（-0.95~+0.95），Kp单位是 (modulation/A)
 *       大电流电机电感小（约50~200uH），初始值保守，后续在线调整 */
#define MOTOR_ID_KP                 0.15f
#define MOTOR_ID_KI                 0.008f
#define MOTOR_IQ_KP                 0.15f
#define MOTOR_IQ_KI                 0.008f

/* PI输出限幅（调制比范围，0.95对应说明书95%占空比限制） */
#define MOTOR_PI_OUTPUT_MAX         0.95f
#define MOTOR_PI_OUTPUT_MIN        -0.95f

/* ---- PWM参数 ---- */
#define MOTOR_PWM_FREQ_HZ           20000       /* PWM开关频率：20kHz */
#define MOTOR_PWM_ARR               3999        /* TIM1自动重装载值：160MHz/(2*4000)=20kHz(中心对齐) */
#define MOTOR_CTRL_PERIOD_S         (1.0f / (float)MOTOR_PWM_FREQ_HZ)   /* 控制周期：50us */

/* ---- 保护阈值 ---- */
#define MOTOR_OC_LIMIT_A            65.0f       /* 软件过流保护阈值：65A */
#define MOTOR_OC_PEAK_LIMIT_A       350.0f      /* 峰值电流阈值：350A（说明书瞬态极限，>5ms触发保护） */
#define MOTOR_OV_BUS_LIMIT_V        50.0f       /* 母线过压阈值：50V（发电阶段反电动势可能升高） */
#define MOTOR_UV_BUS_LIMIT_V        15.0f       /* 母线欠压阈值：15V */

/* =======================================================================================================
 *  第二节：数据类型定义
 * ======================================================================================================= */

/* 启动状态机枚举 */
typedef enum {
    MOTOR_STATE_IDLE = 0,           /* 待机：PWM关闭，等待启动指令 */
    MOTOR_STATE_STRONG_DRAG,        /* 阶段一：强拖冲击（0~5ms，开环最大电压） */
    MOTOR_STATE_TAKEOVER,           /* 阶段二：电流环接管（5~50ms，PI收敛到40A） */
    MOTOR_STATE_STEADY,             /* 阶段三：I-F稳态运行（50ms~4s，电流闭环+角度开环） */
    MOTOR_STATE_STOP,               /* 停机：直接关闭PWM，检测反电动势 */
    MOTOR_STATE_FAULT,              /* 故障：保护触发后的安全状态 */
    MOTOR_STATE_GENERATING          /* 发电模式：检测到BEMF后进入有源整流 */
} MOTOR_State_t;

/* 故障标志位枚举 */
typedef enum {
    MOTOR_FAULT_NONE        = 0x00,
    MOTOR_FAULT_OVER_CUR    = 0x01,     /* 软件过流（>65A持续） */
    MOTOR_FAULT_PEAK_CUR    = 0x02,     /* 峰值过流（>350A，硬件保护触发） */
    MOTOR_FAULT_OVER_VOLT   = 0x04,     /* 母线过压（>50V） */
    MOTOR_FAULT_UNDER_VOLT  = 0x08,     /* 母线欠压（<15V） */
    MOTOR_FAULT_START_FAIL  = 0x10      /* 启动失败（4s后未检测到BEMF） */
} MOTOR_Fault_t;

/* PI控制器结构体 */
typedef struct {
    float Kp;               /* 比例系数 */
    float Ki;               /* 积分系数 */
    float integral;         /* 积分累积值 */
    float outputMax;        /* 输出上限 */
    float outputMin;        /* 输出下限 */
    float output;           /* 当前输出 */
} MOTOR_PI_Controller_t;

/* =======================================================================================================
 *  第三节：API函数声明
 * ======================================================================================================= */

/*
*********************************************************************************************************
*   函 数 名: MOTOR_Init
*   功能说明: 初始化启动控制器（初始化PI参数、清零状态变量，不启动PWM）
*   形    参: 无
*   返 回 值: 无
*   说    明: 系统上电后调用一次
*********************************************************************************************************
*/
void MOTOR_Init(void);

/*
*********************************************************************************************************
*   函 数 名: MOTOR_Start
*   功能说明: 启动电机（状态机从IDLE进入STRONG_DRAG，开始计时）
*   形    参: 无
*   返 回 值: 无
*   说    明: 调用前确保INA303已完成零偏校准，母线电压正常，无故障
*********************************************************************************************************
*/
void MOTOR_Start(void);

/*
*********************************************************************************************************
*   函 数 名: MOTOR_Stop
*   功能说明: 紧急停止电机（立即关闭所有PWM输出，状态回到IDLE）
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void MOTOR_Stop(void);

/*
*********************************************************************************************************
*   函 数 名: MOTOR_FastControlTask
*   功能说明: FOC快速控制任务（20kHz电流环）
*   形    参: 无
*   返 回 值: 无
*   说    明: 
 *     【调用位置】：必须在 ADC1_DMA_IRQHandler 中断函数中调用
 *     【执行流程】：读取电流 → 电角度积分 → Clark → Park → PI → 反Park → SVPWM → 更新TIM1_CCR
*********************************************************************************************************
*/
void MOTOR_FastControlTask(void);

/*
*********************************************************************************************************
*   函 数 名: MOTOR_SlowTask_1kHz
*   功能说明: 慢速状态任务（状态机，在主循环中每1ms调用一次）
*   形    参: 无
*   返 回 值: 无
*   说    明: 
 *     【调用位置】：主循环轮询 flag_1kHz，置位时调用
 *     【执行流程】：计时器累加 → 故障检测 → 状态机切换 → 电频率/电流指令生成
*********************************************************************************************************
*/
void MOTOR_SlowTask_1kHz(void);

/*
*********************************************************************************************************
*   函 数 名: MOTOR_CheckBEMF_AndEnterGenerating
*   功能说明: BEMF反电动势检测与发电模式切入
*   形    参: 无
*   返 回 值: 
 *     true:  检测到有效BEMF，已切入发电模式
 *     false: 未检测到有效BEMF，启动失败
*   说    明: 
 *     【重要】本函数为预留框架，当前默认返回true便于联调。
 *     真实BEMF检测需额外三相电压采样电路支持，详见函数内部6步骤框架。
*********************************************************************************************************
*/
bool MOTOR_CheckBEMF_AndEnterGenerating(void);

/* ---- 状态与故障查询 ---- */
MOTOR_State_t MOTOR_GetState(void);
uint8_t MOTOR_GetFaultFlags(void);
void MOTOR_ClearFault(void);

/* ---- 实时数据查询（调试/上位机用） ---- */
float MOTOR_GetIu(void);            /* U相电流（A） */
float MOTOR_GetIv(void);            /* V相电流（A） */
float MOTOR_GetIw(void);            /* W相电流（A） */
float MOTOR_GetId(void);            /* D轴电流（A） */
float MOTOR_GetIq(void);            /* Q轴电流（A） */
float MOTOR_GetBusVoltage(void);    /* 母线电压（V） */
float MOTOR_GetElecAngle(void);     /* 电角度（rad） */
float MOTOR_GetElecFreqHz(void);    /* 电频率（Hz） */
float MOTOR_GetMechSpeedRPM(void);  /* 机械转速（rpm） */

/* ---- PI参数在线调整（调试用） ---- */
void MOTOR_SetIdPI(float Kp, float Ki);
void MOTOR_SetIqPI(float Kp, float Ki);

#endif /* __APP_MOTOR_STARTUP_H__ */
