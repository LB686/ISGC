/**
 * @brief    启发一体控制器状态机与FOC控制
 * @file     app_motor_startup.h
 * @details  六状态主状态机 + 四阶段拖动子状态机
 *           控制策略：上电自检 → 待命 → VF强拖(0~5ms) → 电流环接管(5~50ms) → I-F稳态(50ms~4s) → BEMF检测 → 发电
 * @author   LB
 * @version  V2.0
 * @date     2026/04/27
 */

#ifndef __APP_MOTOR_STARTUP_H__
#define __APP_MOTOR_STARTUP_H__

#include <stdint.h>
#include <stdbool.h>

/* =======================================================================================================
 *  第一节：电机与系统参数
 * ======================================================================================================= */

#define MOTOR_POLE_PAIRS            15
#define MOTOR_KV_RPM_PER_V          60.0f
#define MOTOR_TARGET_SPEED_RPM      1200.0f
#define MOTOR_TARGET_CURRENT_A      40.0f
#define MOTOR_STARTUP_TOTAL_TIME_MS 4000

/* ---- PWM参数（168MHz中心对齐，ARR=4199 → 20kHz） ---- */
#define MOTOR_PWM_FREQ_HZ           20000
#define MOTOR_PWM_ARR               4199
#define MOTOR_CTRL_PERIOD_S         (1.0f / (float)MOTOR_PWM_FREQ_HZ)

/* ---- 阶段一：VF强拖 (0~5ms) ---- */
#define MOTOR_DRAG_VF_TIME_MS       5
#define MOTOR_DRAG_VQ_MOD           0.90f       /* 调制比直接给0.9 */
#define MOTOR_DRAG_FREQ_HZ          200.0f

/* ---- 阶段二：电流环接管 (5~50ms) ---- */
#define MOTOR_DRAG_TAKEOVER_END_MS  50
#define MOTOR_DRAG_TAKEOVER_RAMP_MS 30

/* ---- 阶段三：I-F稳态 (50ms~4s) ---- */
#define MOTOR_DRAG_STEADY_IQ_A      40.0f
#define MOTOR_DRAG_STEADY_ID_A      0.0f
#define MOTOR_DRAG_STEADY_FREQ_HZ   300.0f
#define MOTOR_FREQ_ACCEL_HZPS       2000.0f

/* ---- 电流环PI参数（输出为调制比，单位 mod/A） ---- */
#define MOTOR_PI_ID_KP              0.15f
#define MOTOR_PI_ID_KI              0.008f
#define MOTOR_PI_IQ_KP              0.15f
#define MOTOR_PI_IQ_KI              0.008f
#define MOTOR_PI_OUTPUT_MAX         0.95f
#define MOTOR_PI_OUTPUT_MIN        -0.95f

/* ---- 母线电压保护 ---- */
#define MOTOR_VBUS_NOMINAL_V        28.5f
#define MOTOR_VBUS_START_MIN_V      22.0f       /* 启动前自检下限 */
#define MOTOR_VBUS_OV_V             50.0f
#define MOTOR_VBUS_UV_V             15.0f

/* ---- 温度保护 ---- */
#define MOTOR_TEMP_START_MAX_C      80.0f       /* 启动前自检上限 */
#define MOTOR_TEMP_OT_WARN_C        100.0f      /* MOS降功率 */
#define MOTOR_TEMP_OT_STOP_C        120.0f      /* 封波 */

/* ---- 软件过流阈值（分阶段动态调整） ---- */
#define MOTOR_OC_DRAG_VF_A          400.0f      /* 阶段一：跟随硬件，不限制 */
#define MOTOR_OC_DRAG_TAKEOVER_A    150.0f      /* 阶段二 */
#define MOTOR_OC_DRAG_STEADY_A      65.0f       /* 阶段三/发电 */

/* =======================================================================================================
 *  第二节：状态机定义
 * ======================================================================================================= */

/* 主状态 */
typedef enum {
    MST_STATE_POWER_ON = 0,     /* 上电初始化 */
    MST_STATE_HW_CHECK,         /* 硬件自检 */
    MST_STATE_STANDBY,          /* 待命等待 */
    MST_STATE_STARTUP_DRAG,     /* 启动强拖 */
    MST_STATE_GENERATING,       /* 发电运行（预留框架） */
    MST_STATE_SAFE_STOP,        /* 安全停机 */
    MST_STATE_FAULT             /* 故障锁定 */
} MST_State_t;

/* 拖动子状态（仅在STARTUP_DRAG内使用） */
typedef enum {
    MST_DRAG_VF_OPEN = 0,       /* 0~5ms: VF开环强拖 */
    MST_DRAG_CURR_TAKEOVER,     /* 5~50ms: 电流闭环切入 */
    MST_DRAG_IF_STEADY,         /* 50ms~4s: I-F稳态运行 */
    MST_DRAG_BEMF_CHECK         /* 4s后: 停PWM，BEMF检测 */
} MST_DragSubState_t;

/* 故障码 */
typedef enum {
    MST_FAULT_NONE = 0,
    MST_FAULT_HW_OC     = 0x01, /* 硬件过流(INA303 ALERT1) */
    MST_FAULT_SW_OC     = 0x02, /* 软件过流(ADC中断检测) */
    MST_FAULT_OV        = 0x04, /* 母线过压 */
    MST_FAULT_UV        = 0x08, /* 母线欠压 */
    MST_FAULT_OT        = 0x10, /* 过温 */
    MST_FAULT_BEMF_FAIL = 0x20, /* BEMF检测失败 */
    MST_FAULT_TIMEOUT   = 0x40  /* 启动超时(>5s) */
} MST_FaultCode_t;

/* PI控制器 */
typedef struct {
    float Kp;
    float Ki;
    float integral;
    float outputMax;
    float outputMin;
} MOTOR_PI_Controller_t;

/* =======================================================================================================
 *  第三节：API声明
 * ======================================================================================================= */

void MOTOR_Init(void);
void MOTOR_Start(void);
void MOTOR_Stop(void);
void MOTOR_EmergencyStop(void);     /* 紧急封波，可在中断中调用 */

/* 20kHz FOC电流环，在ADC1 JEOC中断中调用 */
void MOTOR_FastControlTask(void);

/* 1kHz状态机，在TIM5中断中调用 */
void MOTOR_SlowTask_1kHz(void);

/* 后台任务，在主循环中调用（通信、状态上报、喂狗） */
void MOTOR_BackgroundTask(void);

/* 状态与故障查询 */
MST_State_t MOTOR_GetState(void);
MST_DragSubState_t MOTOR_GetDragSubState(void);
uint8_t MOTOR_GetFaultCode(void);
void MOTOR_ClearFault(void);

/* 实时数据查询 */
float MOTOR_GetIu(void);
float MOTOR_GetIv(void);
float MOTOR_GetIw(void);
float MOTOR_GetId(void);
float MOTOR_GetIq(void);
float MOTOR_GetBusVoltage(void);
float MOTOR_GetElecAngle(void);
float MOTOR_GetElecFreqHz(void);
float MOTOR_GetMechSpeedRPM(void);

/* PI参数在线调整 */
void MOTOR_SetIdPI(float Kp, float Ki);
void MOTOR_SetIqPI(float Kp, float Ki);

#endif /* __APP_MOTOR_STARTUP_H__ */
