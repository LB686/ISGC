/**
 * @brief    启发一体电机控制器状态机及核心控制框架
 * @file     app_motor_ctrl.h
 * @details  基于裸机架构的多状态电机控制状态机
 *           状态流转：INIT → STANDBY → IF_START → FOC_RUN → GEN_READY → GEN_RUN → (FAULT)
 * @author   LB
 * @version  V1.0
 * @date     2026/04/17
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

#ifndef __APP_MOTOR_CTRL_H_
#define __APP_MOTOR_CTRL_H_

#include "../../bsp/bsp.h"

/* Exported constants --------------------------------------------------------*/
#define MOTOR_POLE_PAIRS            15          /* 30P电机，极对数15 */
#define PWM_FREQUENCY_HZ            30000       /* PWM频率30kHz */
#define CURR_LOOP_FREQ_HZ           30000       /* 电流环频率30kHz */
#define SPD_LOOP_FREQ_HZ            1000        /* 速度环/电压环频率1kHz */

/* 启动过程参数 */
#define IF_START_CURRENT_A          20.0f       /* IF开环强拖电流(A) */
#define IF_SWITCH_SPEED_RPM         120         /* IF→FOC切换转速阈值(rpm) */
#define IF_SWITCH_BEMF_V            2.0f        /* IF→FOC切换反电势阈值(V) */
#define IF_MAX_TIME_MS              2000        /* IF启动最大允许时间(ms) */
#define START_TARGET_SPEED_RPM      1200        /* 启动目标转速(rpm) */
#define START_RAMP_TIME_MS          4000        /* 启动总时间(ms) */

/* 发电过程参数 */
#define GEN_TARGET_VBUS_V           28.0f       /* 发电目标母线电压(V) */
#define GEN_VBUS_MAX_V              32.0f       /* 母线过压保护阈值(V) */
#define GEN_READY_WAIT_MS           100         /* 关PWM后等待反电势稳定时间(ms) */
#define GEN_MAX_SPEED_RPM           6000        /* 发电最高转速(rpm) */

/* 保护阈值 */
#define PROT_OC_PEAK_A              350.0f      /* 瞬态峰值过流阈值(A) */
#define PROT_OC_CONT_A              45.0f       /* 持续过流阈值(A)，40A额定留余量 */
#define PROT_OC_CONT_TIME_MS        5000        /* 持续过流允许时间(ms) */
#define PROT_UV_VBUS_V              18.0f       /* 母线欠压阈值(V) */

/* Exported types ------------------------------------------------------------*/

/* 电机主状态枚举 */
typedef enum
{
    MST_INIT = 0,           /* 上电初始化：电流Offset校准、硬件自检 */
    MST_STANDBY,            /* 待机：等待启动指令 */
    MST_IF_START,           /* IF开环强拖：固定Iq，频率斜坡升速 */
    MST_FOC_RUN,            /* FOC闭环电动：无传感器FOC拖动发动机 */
    MST_GEN_READY,          /* 发电准备：关PWM，等待反电势稳定 */
    MST_GEN_RUN,            /* 发电运行：主动整流稳压28V输出 */
    MST_FAULT,              /* 故障：关闭PWM，记录故障码 */
    MST_NUM                 /* 状态总数 */
} MotorState_E;

/* 电机事件枚举（驱动状态转移） */
typedef enum
{
    EVT_NONE = 0,
    EVT_INIT_DONE,          /* 初始化完成（Offset校准结束） */
    EVT_CMD_START,          /* 收到CAN/232启动指令 */
    EVT_CMD_STOP,           /* 收到停机指令 */
    EVT_IF_TIMEOUT,         /* IF启动超时保护 */
    EVT_SMO_CONVERGED,      /* SMO观测器收敛 + 转速达标 */
    EVT_START_TIMEOUT,      /* 4s启动定时到达 */
    EVT_IGNITION_OK,        /* 外部发动机点火完成信号 */
    EVT_BEMF_STABLE,        /* 反电势稳定（延时+幅值检测） */
    EVT_FAULT_OC_HW,        /* 硬件过流（TIM1_Break触发） */
    EVT_FAULT_OC_SW,        /* 软件过流（ADC检测） */
    EVT_FAULT_OV,           /* 母线过压 */
    EVT_FAULT_UV,           /* 母线欠压 */
    EVT_FAULT_OH,           /* 功率级过温 */
    EVT_FAULT_STALL,        /* 堵转检测 */
    EVT_FAULT_CLR,          /* 故障清除指令 */
    EVT_NUM
} MotorEvent_E;

/* 故障码定义 */
typedef enum
{
    ERR_NONE = 0,
    ERR_OC_HW = 0x01,       /* 硬件过流 */
    ERR_OC_SW = 0x02,       /* 软件过流 */
    ERR_OV_VBUS = 0x04,     /* 母线过压 */
    ERR_UV_VBUS = 0x08,     /* 母线欠压 */
    ERR_OH_MOS = 0x10,      /* MOS过温 */
    ERR_STALL = 0x20,       /* 堵转 */
    ERR_IF_TIMEOUT = 0x40,  /* IF启动超时 */
    ERR_SMO_FAIL = 0x80,    /* 观测器发散 */
} FaultCode_E;

/* 状态机控制结构体 */
typedef struct
{
    MotorState_E    CurState;           /* 当前状态 */
    MotorState_E    PreState;           /* 前一状态（用于故障恢复） */
    FaultCode_E     FaultCode;          /* 当前故障码 */
    uint32_t        StateEntryTick;     /* 进入当前状态的时刻(ms) */
    uint32_t        StateRunTick;       /* 当前状态已运行时间(ms) */
    
    /* 运行指令 */
    uint8_t         CmdStart;           /* 启动指令 */
    uint8_t         CmdStop;            /* 停机指令 */
    uint8_t         CmdFaultClr;        /* 故障清除指令 */
    
    /* 外部信号 */
    uint8_t         IgnitionDone;       /* 发动机点火完成 */
    
    /* 实时反馈 */
    float           SpeedRpm;           /* 估算转速(rpm) */
    float           SpeedElecHz;        /* 电频率(Hz) */
    float           Vbus;               /* 母线电压(V) */
    float           Ia, Ib, Ic;         /* 三相电流(A) */
    float           Iq, Id;             /* 旋转坐标系电流(A) */
    float           BemfMag;            /* 反电势幅值(V) */
    uint8_t         SmoConverged;       /* SMO收敛标志 */
    float           TempMos;            /* MOS温度(°C) */
    
    /* 控制输出 */
    float           IdRef;              /* d轴电流指令 */
    float           IqRef;              /* q轴电流指令 */
    float           VdOut;              /* d轴电压输出 */
    float           VqOut;              /* q轴电压输出 */
    float           AngleElec;          /* 电角度(rad) */
    uint16_t        PwmDuty[3];         /* 三相PWM占空比 */
} MotorCtrl_T;

/* 状态机节点函数表原型 */
typedef void (*StateFuncPtr)(void);

/* Exported variables --------------------------------------------------------*/
extern MotorCtrl_T g_MotorCtrl;

/* Exported functions --------------------------------------------------------*/
void MotorCtrl_Init(void);
void MotorCtrl_StateMachineInit(void);
void MotorCtrl_StateMachineRun(void);
void MotorCtrl_PostEvent(MotorEvent_E evt);
void MotorCtrl_FastLoop_ISR(void);      /* 30kHz ADC中断调用 */
void MotorCtrl_SlowLoop_ISR(void);      /* 1kHz 定时器中断调用 */

/* 状态OnEnter接口 */
void MST_Init_OnEnter(void);
void MST_Standby_OnEnter(void);
void MST_IfStart_OnEnter(void);
void MST_FocRun_OnEnter(void);
void MST_GenReady_OnEnter(void);
void MST_GenRun_OnEnter(void);
void MST_Fault_OnEnter(void);

/* 状态OnRun接口 */
void MST_Init_OnRun(void);
void MST_Standby_OnRun(void);
void MST_IfStart_OnRun(void);
void MST_FocRun_OnRun(void);
void MST_GenReady_OnRun(void);
void MST_GenRun_OnRun(void);
void MST_Fault_OnRun(void);

/* 状态OnExit接口 */
void MST_Init_OnExit(void);
void MST_Standby_OnExit(void);
void MST_IfStart_OnExit(void);
void MST_FocRun_OnExit(void);
void MST_GenReady_OnExit(void);
void MST_GenRun_OnExit(void);
void MST_Fault_OnExit(void);

#endif /* __APP_MOTOR_CTRL_H_ */

/***************************** (END OF FILE) *********************************/
