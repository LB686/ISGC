/**
 * @brief    启发一体控制器状态机与FOC控制实现
 * @file     app_motor_startup.c
 * @version  V2.0
 * @date     2026/04/27
 */

#include "app_motor_startup.h"
#include "app_motor_math.h"
#include "bsp_ina303.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* =======================================================================================================
 *  第一节：静态变量
 * ======================================================================================================= */

/* PI控制器（输出为调制比） */
static MOTOR_PI_Controller_t s_piId = {
    .Kp = MOTOR_PI_ID_KP, .Ki = MOTOR_PI_ID_KI, .integral = 0.0f,
    .outputMax = MOTOR_PI_OUTPUT_MAX, .outputMin = MOTOR_PI_OUTPUT_MIN
};
static MOTOR_PI_Controller_t s_piIq = {
    .Kp = MOTOR_PI_IQ_KP, .Ki = MOTOR_PI_IQ_KI, .integral = 0.0f,
    .outputMax = MOTOR_PI_OUTPUT_MAX, .outputMin = MOTOR_PI_OUTPUT_MIN
};

/* 主状态机 */
static volatile MST_State_t s_state = MST_STATE_POWER_ON;
static volatile MST_DragSubState_t s_dragSubState = MST_DRAG_VF_OPEN;
static volatile uint8_t s_faultCode = MST_FAULT_NONE;

/* 时间变量 */
static volatile uint32_t s_runTime_ms = 0;      /* 从Start()开始累计 */
static volatile uint32_t s_dragTime_ms = 0;     /* 拖动阶段独立计时 */
static volatile uint8_t  s_bemfDelayMs = 0;     /* BEMF消磁延时 */

/* 电角度与频率 */
static float s_elecAngle_rad = 0.0f;
static float s_elecFreq_hz = 0.0f;
static float s_elecFreqTarget_hz = 0.0f;

/* 电流指令 */
static float s_idCmd = 0.0f;
static float s_iqCmd = 0.0f;

/* 电流反馈 */
static float s_iu = 0.0f, s_iv = 0.0f, s_iw = 0.0f;
static float s_iAlpha = 0.0f, s_iBeta = 0.0f;
static float s_id = 0.0f, s_iq = 0.0f;

/* 电压输出（调制比，无量纲） */
static float s_vd = 0.0f, s_vq = 0.0f;
static float s_vAlpha = 0.0f, s_vBeta = 0.0f;

/* 母线电压 */
static float s_vbus = 28.5f;

/* 控制标志 */
static uint8_t s_initDone = 0;
static uint8_t s_pwmRunning = 0;
static uint8_t s_currentLoopEnabled = 0;

/* 软件过流阈值（20kHz中断中使用） */
static volatile float s_swOCLimitA = MOTOR_OC_DRAG_STEADY_A;

/* SVPWM扇区（调试用） */
static uint32_t s_svmSector = 0;

/* 强拖阶段最后输出（用于PI积分预置） */
static float s_dragVqLast = 0.0f;



/* =======================================================================================================
 *  第二节：内部函数声明
 * ======================================================================================================= */
static float MOTOR_PI_Update(MOTOR_PI_Controller_t *pi, float error);
static void MOTOR_UpdatePWM(float alpha, float beta);
static void MOTOR_StateMachine_1kHz(void);
static void MOTOR_DragSubStateMachine_1kHz(void);
static void MOTOR_DetectFault_1kHz(void);
static void MOTOR_EnterFault(uint8_t faultCode);
static void MOTOR_StopPWM(void);
static void MOTOR_StartPWM(void);
static bool MOTOR_CheckBEMF(void);

/* =======================================================================================================
 *  第三节：初始化与启停
 * ======================================================================================================= */

void MOTOR_Init(void)
{
    /* 清零PI */
    s_piId.integral = 0.0f;
    s_piIq.integral = 0.0f;

    /* 清零状态 */
    s_state = MST_STATE_POWER_ON;
    s_dragSubState = MST_DRAG_VF_OPEN;
    s_faultCode = MST_FAULT_NONE;
    s_runTime_ms = 0;
    s_dragTime_ms = 0;
    s_bemfDelayMs = 0;

    s_elecAngle_rad = 0.0f;
    s_elecFreq_hz = 0.0f;
    s_elecFreqTarget_hz = 0.0f;
    s_idCmd = 0.0f;
    s_iqCmd = 0.0f;

    s_currentLoopEnabled = 0;
    s_pwmRunning = 0;
    s_swOCLimitA = MOTOR_OC_DRAG_VF_A;

    /* 读取母线电压 */
    s_vbus = BSP_INA303_GetBusVoltage();
    if (s_vbus < 5.0f) {
        s_vbus = MOTOR_VBUS_NOMINAL_V;
    }

    s_initDone = 1;
}

void MOTOR_Start(void)
{
    if (!s_initDone) {
        MOTOR_Init();
    }
    if (s_faultCode != MST_FAULT_NONE) {
        return;
    }
    if (s_state != MST_STATE_STANDBY) {
        return;
    }

    /* 准备拖动 */
    s_elecAngle_rad = 0.0f;
    s_elecFreq_hz = MOTOR_DRAG_FREQ_HZ;
    s_elecFreqTarget_hz = MOTOR_DRAG_FREQ_HZ;
    s_dragTime_ms = 0;
    s_bemfDelayMs = 0;
    s_dragSubState = MST_DRAG_VF_OPEN;
    s_swOCLimitA = MOTOR_OC_DRAG_VF_A;

    s_piId.integral = 0.0f;
    s_piIq.integral = 0.0f;
    s_currentLoopEnabled = 0;

    /* 启动PWM */
    MOTOR_StartPWM();

    s_state = MST_STATE_STARTUP_DRAG;
}

void MOTOR_Stop(void)
{
    MOTOR_StopPWM();
    s_state = MST_STATE_SAFE_STOP;
    s_dragTime_ms = 0;
}

void MOTOR_EmergencyStop(void)
{
    /* 可在中断中调用的紧急封波 */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = INA303_TIM1_CCR4_VALUE;  /* 保持ADC触发 */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    s_pwmRunning = 0;
    s_currentLoopEnabled = 0;
}

static void MOTOR_StartPWM(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    s_pwmRunning = 1;
}

static void MOTOR_StopPWM(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    s_pwmRunning = 0;
    s_currentLoopEnabled = 0;
}

static void MOTOR_EnterFault(uint8_t faultCode)
{
    MOTOR_StopPWM();
    s_faultCode |= faultCode;
    s_state = MST_STATE_FAULT;
}

/* =======================================================================================================
 *  第四节：PI控制器
 * ======================================================================================================= */

static float MOTOR_PI_Update(MOTOR_PI_Controller_t *pi, float error)
{
    float p_term = error * pi->Kp;
    pi->integral += error * pi->Ki;

    /* 积分限幅 */
    if (pi->integral > pi->outputMax) {
        pi->integral = pi->outputMax;
    } else if (pi->integral < pi->outputMin) {
        pi->integral = pi->outputMin;
    }

    float output = p_term + pi->integral;

    /* 输出限幅 */
    if (output > pi->outputMax) {
        output = pi->outputMax;
    } else if (output < pi->outputMin) {
        output = pi->outputMin;
    }

    return output;
}

/* =======================================================================================================
 *  第五节：PWM更新（SVPWM）
 * ======================================================================================================= */

static void MOTOR_UpdatePWM(float alpha, float beta)
{
    uint32_t duty1, duty2, duty3;

    MOTOR_SVM(alpha, beta, 0.95f, (uint32_t)MOTOR_PWM_ARR,
              &duty1, &duty2, &duty3, &s_svmSector);

    TIM1->CCR1 = duty1;
    TIM1->CCR2 = duty2;
    TIM1->CCR3 = duty3;
}

/* =======================================================================================================
 *  第六节：20kHz快速控制任务（FOC电流环）
 * ======================================================================================================= */

void MOTOR_FastControlTask(void)
{
    float sinTheta, cosTheta;

    if (!s_initDone || !s_pwmRunning) {
        return;
    }

    /* ---- Step 1: 读取三相电流 ---- */
    s_iu = BSP_INA303_GetCurrentU();
    s_iv = BSP_INA303_GetCurrentV();
    s_iw = BSP_INA303_GetCurrentW();

    /* ---- Step 2: 20kHz软件过流检查（最快响应） ---- */
    float ocLimit = s_swOCLimitA;
    if (fabsf(s_iu) > ocLimit || fabsf(s_iv) > ocLimit || fabsf(s_iw) > ocLimit) {
        MOTOR_EmergencyStop();
        s_faultCode |= MST_FAULT_SW_OC;
        return;     /* 直接返回，不再执行后续控制 */
    }

    /* ---- Step 3: 电角度积分 ---- */
    s_elecAngle_rad += 2.0f * M_PI * s_elecFreq_hz * MOTOR_CTRL_PERIOD_S;
    while (s_elecAngle_rad >= 2.0f * M_PI) {
        s_elecAngle_rad -= 2.0f * M_PI;
    }
    while (s_elecAngle_rad < 0.0f) {
        s_elecAngle_rad += 2.0f * M_PI;
    }

    /* ---- Step 4: Clark变换 ---- */
    MOTOR_Clarke(s_iu, s_iv, &s_iAlpha, &s_iBeta);

    /* ---- Step 5: Park变换 ---- */
    MOTOR_Park(s_iAlpha, s_iBeta, s_elecAngle_rad, &s_id, &s_iq);

    /* ---- Step 6: 根据状态生成Vd/Vq（调制比） ---- */
    switch (s_state) {
        case MST_STATE_STARTUP_DRAG: {
            if (s_dragSubState == MST_DRAG_VF_OPEN) {
                /* VF开环：直接给调制比 */
                s_vd = 0.0f;
                s_vq = MOTOR_DRAG_VQ_MOD;
                s_dragVqLast = s_vq;    /* 记录用于PI预置 */
            } else if (s_currentLoopEnabled) {
                /* 电流闭环：PI输出调制比 */
                s_vd = MOTOR_PI_Update(&s_piId, s_idCmd - s_id);
                s_vq = MOTOR_PI_Update(&s_piIq, s_iqCmd - s_iq);
                /* 圆限幅 */
                float mag = sqrtf(s_vd * s_vd + s_vq * s_vq);
                if (mag > MOTOR_PI_OUTPUT_MAX && mag > 0.0f) {
                    float scale = MOTOR_PI_OUTPUT_MAX / mag;
                    s_vd *= scale;
                    s_vq *= scale;
                }
            } else {
                s_vd = 0.0f;
                s_vq = 0.0f;
            }
            break;
        }

        case MST_STATE_GENERATING: {
            /* 发电模式预留：Id<0, Iq=0, 电压外环 */
            /* 当前简化为关闭输出 */
            s_vd = 0.0f;
            s_vq = 0.0f;
            break;
        }

        default: {
            /* 其他状态输出0 */
            s_vd = 0.0f;
            s_vq = 0.0f;
            break;
        }
    }

    /* ---- Step 7: InvPark → SVPWM → 更新CCR ---- */
    MOTOR_InvPark(s_vd, s_vq, s_elecAngle_rad, &s_vAlpha, &s_vBeta);
    MOTOR_UpdatePWM(s_vAlpha, s_vBeta);
}

/* =======================================================================================================
 *  第七节：1kHz慢速状态任务
 * ======================================================================================================= */

void MOTOR_SlowTask_1kHz(void)
{
    if (!s_initDone) {
        return;
    }

    /* 更新母线电压 */
    s_vbus = BSP_INA303_GetBusVoltage();

    /* 主状态机 */
    MOTOR_StateMachine_1kHz();

    /* 故障检测（1kHz消抖） */
    MOTOR_DetectFault_1kHz();
}

static void MOTOR_DetectFault_1kHz(void)
{
    /* 母线过压 */
    if (s_vbus > MOTOR_VBUS_OV_V) {
        MOTOR_EnterFault(MST_FAULT_OV);
        return;
    }

    /* 母线欠压（运行时检测） */
    if (s_vbus < MOTOR_VBUS_UV_V && s_vbus > 5.0f) {
        if (s_state == MST_STATE_STARTUP_DRAG || s_state == MST_STATE_GENERATING) {
            MOTOR_EnterFault(MST_FAULT_UV);
            return;
        }
    }

    /* 启动超时检测 */
    if (s_state == MST_STATE_STARTUP_DRAG && s_dragTime_ms > 5000) {
        MOTOR_EnterFault(MST_FAULT_TIMEOUT);
        return;
    }

    /* 温度保护（从INA303驱动获取温度） */
    float temp = BSP_INA303_GetTemperature();
    if (temp > MOTOR_TEMP_OT_STOP_C) {
        MOTOR_EnterFault(MST_FAULT_OT);
        return;
    }
}

static void MOTOR_StateMachine_1kHz(void)
{
    switch (s_state) {

        /* ═══════════════════════════════════════════════════════
         * POWER_ON：上电初始化
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_POWER_ON: {
            /* 延时100ms等待电源稳定 */
            static uint16_t powerOnDelay = 0;
            if (++powerOnDelay >= 100) {
                powerOnDelay = 0;
                s_state = MST_STATE_HW_CHECK;
            }
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * HW_CHECK：硬件自检
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_HW_CHECK: {
            float vbus = BSP_INA303_GetBusVoltage();
            float temp = BSP_INA303_GetTemperature();

            bool vbus_ok = (vbus >= MOTOR_VBUS_START_MIN_V) && (vbus <= MOTOR_VBUS_OV_V);
            bool temp_ok = (temp < MOTOR_TEMP_START_MAX_C);
            bool no_hw_fault = (BSP_INA303_GetFaultFlags() == 0);

            if (vbus_ok && temp_ok && no_hw_fault) {
                s_state = MST_STATE_STANDBY;
            } else {
                /* 自检失败，记录具体故障 */
                if (!vbus_ok) {
                    s_faultCode |= (vbus < MOTOR_VBUS_START_MIN_V) ? MST_FAULT_UV : MST_FAULT_OV;
                }
                if (!temp_ok) {
                    s_faultCode |= MST_FAULT_OT;
                }
                s_state = MST_STATE_FAULT;
            }
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * STANDBY：待命等待
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_STANDBY: {
            /* 等待MOTOR_Start()启动 */
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * STARTUP_DRAG：启动强拖（内含四段式子状态机）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_STARTUP_DRAG: {
            s_dragTime_ms++;
            MOTOR_DragSubStateMachine_1kHz();
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * GENERATING：发电运行（预留框架）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_GENERATING: {
            /* 预留：有源整流控制 */
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * SAFE_STOP：安全停机
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_SAFE_STOP: {
            s_dragTime_ms++;
            if (s_dragTime_ms >= 100) {   /* 100ms消磁+冷却 */
                if (s_faultCode == MST_FAULT_NONE) {
                    s_state = MST_STATE_STANDBY;
                } else {
                    s_state = MST_STATE_FAULT;
                }
            }
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * FAULT：故障锁定
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_FAULT: {
            /* PWM已关闭，等待外部清除 */
            break;
        }

        default: {
            break;
        }
    }
}

/* =======================================================================================================
 *  第八节：拖动子状态机（在STARTUP_DRAG内运行）
 * ======================================================================================================= */

static void MOTOR_DragSubStateMachine_1kHz(void)
{
    switch (s_dragSubState) {

        /* ─── 阶段一：VF开环强拖 (0~5ms) ─── */
        case MST_DRAG_VF_OPEN: {
            s_idCmd = 0.0f;
            s_iqCmd = 0.0f;

            if (s_dragTime_ms >= MOTOR_DRAG_VF_TIME_MS) {
                /* 切换到电流闭环，预置PI积分器 */
                s_piId.integral = 0.0f;
                s_piIq.integral = s_dragVqLast;   /* Vq预置 */
                s_currentLoopEnabled = 1;
                s_swOCLimitA = MOTOR_OC_DRAG_TAKEOVER_A;
                s_dragSubState = MST_DRAG_CURR_TAKEOVER;
            }
            break;
        }

        /* ─── 阶段二：电流环接管 (5~50ms) ─── */
        case MST_DRAG_CURR_TAKEOVER: {
            /* Iq斜坡：从50A降到40A */
            uint32_t ramp_time = s_dragTime_ms - MOTOR_DRAG_VF_TIME_MS;
            if (ramp_time < MOTOR_DRAG_TAKEOVER_RAMP_MS) {
                float step = (50.0f - MOTOR_DRAG_STEADY_IQ_A) / (float)MOTOR_DRAG_TAKEOVER_RAMP_MS;
                s_iqCmd = 50.0f - step * (float)ramp_time;
            } else {
                s_iqCmd = MOTOR_DRAG_STEADY_IQ_A;
            }
            s_idCmd = MOTOR_DRAG_STEADY_ID_A;

            /* 频率爬升 */
            if (s_elecFreq_hz < s_elecFreqTarget_hz) {
                s_elecFreq_hz += MOTOR_FREQ_ACCEL_HZPS * 0.001f;
            }

            if (s_dragTime_ms >= MOTOR_DRAG_TAKEOVER_END_MS) {
                s_dragSubState = MST_DRAG_IF_STEADY;
                s_elecFreqTarget_hz = MOTOR_DRAG_STEADY_FREQ_HZ;
                s_swOCLimitA = MOTOR_OC_DRAG_STEADY_A;
            }
            break;
        }

        /* ─── 阶段三：I-F稳态 (50ms~4s) ─── */
        case MST_DRAG_IF_STEADY: {
            s_idCmd = MOTOR_DRAG_STEADY_ID_A;
            s_iqCmd = MOTOR_DRAG_STEADY_IQ_A;

            /* 频率斜坡至300Hz */
            if (s_elecFreq_hz < s_elecFreqTarget_hz) {
                s_elecFreq_hz += MOTOR_FREQ_ACCEL_HZPS * 0.001f;
                if (s_elecFreq_hz > s_elecFreqTarget_hz) {
                    s_elecFreq_hz = s_elecFreqTarget_hz;
                }
            }

            if (s_dragTime_ms >= MOTOR_STARTUP_TOTAL_TIME_MS) {
                /* 4s到，停PWM，准备BEMF检测 */
                MOTOR_StopPWM();
                s_bemfDelayMs = 0;
                s_dragSubState = MST_DRAG_BEMF_CHECK;
            }
            break;
        }

        /* ─── 阶段四：BEMF检测 ─── */
        case MST_DRAG_BEMF_CHECK: {
            s_bemfDelayMs++;
            if (s_bemfDelayMs >= 2) {   /* 2ms消磁 */
                if (MOTOR_CheckBEMF()) {
                    /* BEMF检测成功，进入发电模式 */
                    s_state = MST_STATE_GENERATING;
                    /* 如需启用ADC2 BEMF持续采集： */
                    /* BSP_INA303_ADC2_Start(); */
                } else {
                    s_faultCode |= MST_FAULT_BEMF_FAIL;
                    s_state = MST_STATE_SAFE_STOP;
                }
            }
            break;
        }

        default: {
            break;
        }
    }
}

/* =======================================================================================================
 *  第九节：BEMF检测（预留框架）
 * ======================================================================================================= */

static bool MOTOR_CheckBEMF(void)
{
    /* 预留：BEMF检测逻辑
     * 当前返回true便于联调，后续实现：
     * 1. 读取ADC2注入组三相端电压（JDR1/2/3）
     * 2. 计算线电压幅值或频率
     * 3. 判断幅值>阈值且频率>250Hz */
    return true;
}

/* =======================================================================================================
 *  第十节：后台任务（主循环）
 * ======================================================================================================= */

void MOTOR_BackgroundTask(void)
{
    if (!s_initDone) {
        return;
    }

    /* CAN/RS232状态上报、通信解析等后台非实时任务 */
    /* 注意：不要在这里做实时控制，实时任务已在对应ISR中执行 */
}

/* =======================================================================================================
 *  第十一节：查询接口
 * ======================================================================================================= */

MST_State_t MOTOR_GetState(void)            { return s_state; }
MST_DragSubState_t MOTOR_GetDragSubState(void) { return s_dragSubState; }
uint8_t MOTOR_GetFaultCode(void)            { return s_faultCode; }

void MOTOR_ClearFault(void)
{
    s_faultCode = MST_FAULT_NONE;
    if (s_state == MST_STATE_FAULT) {
        s_state = MST_STATE_HW_CHECK;
    }
}

float MOTOR_GetIu(void)                     { return s_iu; }
float MOTOR_GetIv(void)                     { return s_iv; }
float MOTOR_GetIw(void)                     { return s_iw; }
float MOTOR_GetId(void)                     { return s_id; }
float MOTOR_GetIq(void)                     { return s_iq; }
float MOTOR_GetBusVoltage(void)             { return s_vbus; }
float MOTOR_GetElecAngle(void)              { return s_elecAngle_rad; }
float MOTOR_GetElecFreqHz(void)             { return s_elecFreq_hz; }

float MOTOR_GetMechSpeedRPM(void)
{
    return (s_elecFreq_hz * 60.0f) / (float)MOTOR_POLE_PAIRS;
}

void MOTOR_SetIdPI(float Kp, float Ki)      { s_piId.Kp = Kp; s_piId.Ki = Ki; }
void MOTOR_SetIqPI(float Kp, float Ki)      { s_piIq.Kp = Kp; s_piIq.Ki = Ki; }

/* ============================= (END OF FILE) ============================= */
