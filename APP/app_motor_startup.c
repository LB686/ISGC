/*
 *********************************************************************************************************
 *   模块名称 : 电机启动控制模块实现
 *   文件名称 : app_motor_startup.c
 *   版    本 : V1.0
 *   说    明 : 三阶段启动控制：强拖冲击 → 电流环接管 → I-F稳态 → BEMF检测/发电
 *              20kHz FOC电流环 + 1kHz状态机
 *   硬件依赖 : TIM1(20kHz PWM) + TIM2(ADC触发) + TIM5(1MHz计时) + ADC1(三相电流) + ADC2(母线电压)
 *   重要提示 :
 *     1. 本代码假设ADC已配置为TIM2触发+DMA循环模式（详见CUBEMX配置表）
 *     2. 需在 ADC1_DMA_IRQHandler 中调用 MST_FastControlTask()
 *     3. 需在主循环中每1ms调用 MST_SlowTask_1kHz()
 *     4. 需在 HAL_ADC_ConvHalfCpltCallback / HAL_ADC_ConvCpltCallback 中调用INA303 DMA回调
 *********************************************************************************************************
 */

#include "../../app/app_motor_startup.h"

#include "../../app/app_motor_math.h"
#include "../../bsp/inc/bsp_ina303.h"

#include "stm32f4xx_hal.h"

/* =======================================================================================================
 *  第一节：静态变量定义
 * ======================================================================================================= */

/* ---- PI控制器实例（Id=0控制，Iq=转矩控制） ---- */
static MST_PI_Controller_t s_piId = {
    .Kp = MST_ID_KP, .Ki = MST_ID_KI, .integral = 0.0f,
    .outputMax = MST_PI_OUTPUT_MAX, .outputMin = MST_PI_OUTPUT_MIN, .output = 0.0f
};
static MST_PI_Controller_t s_piIq = {
    .Kp = MST_IQ_KP, .Ki = MST_IQ_KI, .integral = 0.0f,
    .outputMax = MST_PI_OUTPUT_MAX, .outputMin = MST_PI_OUTPUT_MIN, .output = 0.0f
};

/* ---- 状态机变量 ---- */
static volatile MST_State_t s_state = MST_STATE_IDLE;       /* 当前状态 */
static volatile uint8_t s_faultFlags = 0;                   /* 故障标志位 */
static volatile uint32_t s_runTime_ms = 0;                  /* 累计运行时间(ms)，从MST_Start()开始计时 */
static volatile uint8_t s_flag_1kHz = 0;                    /* 1kHz慢速任务标志 */

/* ---- 电角度与频率 ---- */
static float s_elecAngle_rad = 0.0f;                        /* 电角度，单位弧度，全程虚拟积分 */
static float s_elecFreq_hz = 0.0f;                          /* 电频率，单位Hz */
static float s_elecFreqTarget_hz = 0.0f;                    /* 电频率指令值 */

/* ---- 电流指令 ---- */
static float s_idCmd = 0.0f;                                /* D轴电流指令 */
static float s_iqCmd = 0.0f;                                /* Q轴电流指令 */

/* ---- 电流反馈（物理量，单位A） ---- */
static float s_iu = 0.0f, s_iv = 0.0f, s_iw = 0.0f;        /* 三相电流 */
static float s_ialpha = 0.0f, s_ibeta = 0.0f;              /* Clark变换后 */
static float s_id = 0.0f, s_iq = 0.0f;                     /* Park变换后 */

/* ---- 电压输出（归一化前的物理电压，单位V） ---- */
static float s_vd = 0.0f, s_vq = 0.0f;                     /* 旋转坐标系电压 */
static float s_valpha = 0.0f, s_vbeta = 0.0f;              /* 静止坐标系电压 */

/* ---- 母线电压 ---- */
static float s_vbus = 28.5f;                               /* 母线电压，由ADC2 DMA中断更新 */

/* ---- 控制标志 ---- */
static uint8_t s_initDone = 0;                              /* 初始化完成标志 */
static uint8_t s_currentLoopEnabled = 0;                    /* 电流环使能标志（阶段二/三使能） */
static uint32_t s_1kHzDivider = 0;                          /* 20kHz→1kHz分频计数器 */

/* ---- 强拖阶段记录（用于PI预置） ---- */
static float s_dragVdLast = 0.0f;                           /* 强拖阶段最后输出的Vd */
static float s_dragVqLast = 0.0f;                           /* 强拖阶段最后输出的Vq */

/* ---- SVPWM扇区（调试用） ---- */
static uint32_t s_svmSector = 0;

/* ---- BEMF检测延时计数器 ---- */
static uint8_t s_bemfDelayCnt = 0;

/* =======================================================================================================
 *  第二节：内部函数声明
 * ======================================================================================================= */
static float MST_PI_Update(MST_PI_Controller_t *pi, float error);
static void MST_UpdatePWM(float valpha, float vbeta);
static void MST_StateMachine_1kHz(void);
static void MST_DetectFault(void);

/* =======================================================================================================
 *  第三节：初始化与启停控制
 * ======================================================================================================= */

/**
 * @brief  初始化启动控制器
 * @note   系统上电后调用一次，仅初始化参数，不启动PWM
 */
void MST_Init(void) {
    /* 清零PI控制器 */
    s_piId.integral = 0.0f;  s_piId.output = 0.0f;
    s_piIq.integral = 0.0f;  s_piIq.output = 0.0f;

    /* 清零状态变量 */
    s_state = MST_STATE_IDLE;
    s_faultFlags = 0;
    s_runTime_ms = 0;
    s_elecAngle_rad = 0.0f;
    s_elecFreq_hz = 0.0f;
    s_elecFreqTarget_hz = 0.0f;
    s_idCmd = 0.0f;
    s_iqCmd = 0.0f;
    s_currentLoopEnabled = 0;

    /* 读取初始母线电压 */
    s_vbus = BSP_INA303_GetBusVoltage();
    if (s_vbus < 5.0f) {
        s_vbus = 28.5f;   /* 如果ADC还未就绪，使用默认值 */
    }

    s_initDone = 1;
}

/**
 * @brief  启动电机
 * @note   调用后状态机进入STRONG_DRAG，电角度清零，计时开始
 *         调用前务必确认：INA303零偏校准已完成，母线电压正常，无故障
 */
void MST_Start(void) {
    if (!s_initDone) {
        MST_Init();
    }
    if (s_faultFlags != 0) {
        return;     /* 有故障未清除，禁止启动 */
    }

    /* 清零电角度和频率，准备从0开始积分 */
    s_elecAngle_rad = 0.0f;
    s_elecFreq_hz = MST_STRONG_DRAG_FREQ_HZ;   /* 强拖阶段以较高频率起步 */
    s_elecFreqTarget_hz = MST_STRONG_DRAG_FREQ_HZ;
    s_bemfDelayCnt = 0;                        /* 清零BEMF检测计数器 */

    /* 清零PI积分器（阶段一不使用，但提前清零避免遗留） */
    s_piId.integral = 0.0f;  s_piId.output = 0.0f;
    s_piIq.integral = 0.0f;  s_piIq.output = 0.0f;

    /* 电流环关闭（阶段一开环） */
    s_currentLoopEnabled = 0;

    /* 清零运行计时 */
    s_runTime_ms = 0;

    /* 使能TIM1 PWM输出（需确保TIM1已在CUBEMX中配置并使能） */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 进入强拖阶段 */
    s_state = MST_STATE_STRONG_DRAG;
}

/**
 * @brief  紧急停止电机
 * @note   立即关闭所有PWM输出，状态回到IDLE
 */
void MST_Stop(void) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    /* 三相下桥臂短接制动（可选：如需高阻态，改为全部关闭） */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    s_state = MST_STATE_IDLE;
    s_currentLoopEnabled = 0;
    s_elecFreq_hz = 0.0f;
}

/* =======================================================================================================
 *  第四节：PI控制器实现
 * ======================================================================================================= */

/**
 * @brief  PI控制器更新
 * @param  pi: PI结构体指针
 * @param  error: 当前误差（指令值 - 反馈值）
 * @retval PI输出值
 * @note   采用增量式抗饱和PI：积分项单独限幅，输出再限幅
 */
static float MST_PI_Update(MST_PI_Controller_t *pi, float error) {
    /* 比例项 */
    float p_term = error * pi->Kp;

    /* 积分项（累加） */
    pi->integral += error * pi->Ki;

    /* 积分限幅（防止积分饱和 wind-up） */
    if (pi->integral > pi->outputMax) {
        pi->integral = pi->outputMax;
    } else if (pi->integral < pi->outputMin) {
        pi->integral = pi->outputMin;
    }

    /* 总输出 = 比例 + 积分 */
    float output = p_term + pi->integral;

    /* 输出限幅 */
    if (output > pi->outputMax) {
        output = pi->outputMax;
    } else if (output < pi->outputMin) {
        output = pi->outputMin;
    }

    pi->output = output;
    return output;
}

/* =======================================================================================================
 *  第五节：PWM更新（SVPWM调制 + 写TIM1_CCR）
 * ======================================================================================================= */

/**
 * @brief  更新三相PWM占空比
 * @param  valpha: 静止坐标系Vα电压（单位V）
 * @param  vbeta:  静止坐标系Vβ电压（单位V）
 * @note   将物理电压归一化后调用VESC的SVPWM算法，输出直接写入TIM1_CCR1/2/3
 */
static void MST_UpdatePWM(float valpha, float vbeta) {
    uint32_t duty1, duty2, duty3;

    /* 归一化：将物理电压转换为调制比（modulation index）
     * 最大线性调制电压幅值 = Vbus * (2/√3) * max_mod
     * 因此 mod = V / (Vbus * 2/√3)
     */
    float v_max = s_vbus * TWO_BY_SQRT3;
    if (v_max < 1.0f) {
        v_max = 1.0f;   /* 防止除零 */
    }

    float mod_alpha = valpha / v_max;
    float mod_beta  = vbeta  / v_max;

    /* 圆限幅：防止过调制 */
    utils_saturate_vector_2d(&mod_alpha, &mod_beta, 0.95f);

    /* 调用VESC SVPWM算法，输出0~ARR的计数值 */
    foc_svm(mod_alpha, mod_beta, 0.95f, (uint32_t)MST_PWM_ARR,
            &duty1, &duty2, &duty3, &s_svmSector);

    /* 写入TIM1捕获比较寄存器（CCR有预装载，下个PWM周期生效） */
    TIM1->CCR1 = duty1;
    TIM1->CCR2 = duty2;
    TIM1->CCR3 = duty3;
}

/* =======================================================================================================
 *  第六节：20kHz快速控制任务（FOC电流环，在ADC1_DMA中断中调用）
 * ======================================================================================================= */

/**
 * @brief  FOC快速控制任务（20kHz）
 * @note   
 *   【调用位置】：必须在 ADC1_DMA_IRQHandler 中断函数中调用
 *   【时序要求】：ADC1_DMA中断由TIM2触发，确保采样时刻在下桥臂导通中点
 *   【执行流程】：
 *     1. 读取三相电流（INA303驱动API）
 *     2. 电角度积分（每50us累加一次）
 *     3. Clark变换 → Park变换
 *     4. 根据状态机选择控制模式（开环/闭环）
 *     5. 反Park变换 → SVPWM → 更新TIM1_CCR
 *     6. 20分频置位1kHz标志
 */
void MST_FastControlTask(void) {
    float sin_t, cos_t;

    if (!s_initDone) {
        return;
    }

    /* ========== Step 1: 读取三相电流（单位：A）==========
     * 从INA303驱动获取最新的三相电流值
     * 注意：INA303驱动内部已完成DMA双缓冲处理和滤波
     */
    s_iu = BSP_INA303_GetCurrentU();
    s_iv = BSP_INA303_GetCurrentV();
    s_iw = BSP_INA303_GetCurrentW();

    /* 三相和校验（可选调试）：|Iu+Iv+Iw| 应接近0，若偏差大说明采样异常 */
    /* float i_sum = s_iu + s_iv + s_iw; */

    /* ========== Step 2: 电角度积分 ==========
     * 电角度增量 = 2π * 频率 * 控制周期(50us)
     * 频率由1kHz慢任务根据状态机更新
     */
    s_elecAngle_rad += 2.0f * M_PI * s_elecFreq_hz * MST_CTRL_PERIOD_S;
    utils_norm_angle_rad(&s_elecAngle_rad);

    /* ========== Step 3: Clark变换（三相 → 两相静止坐标系） ========== */
    MST_Clarke(s_iu, s_iv, &s_ialpha, &s_ibeta);

    /* ========== Step 4: Park变换（静止 → 旋转坐标系） ========== */
    utils_fast_sincos_better(s_elecAngle_rad, &sin_t, &cos_t);
    s_id =  s_ialpha * cos_t + s_ibeta * sin_t;
    s_iq = -s_ialpha * sin_t + s_ibeta * cos_t;

    /* ========== Step 5: 根据当前状态执行控制 ========== */
    switch (s_state) {

        /* ───────────────────────────────────────────
         * 阶段一：强拖冲击（开环，不控电流，直接输出电压）
         * ─────────────────────────────────────────── */
        case MST_STATE_STRONG_DRAG: {
            /* Vd = 0，Vq = 0.9 * Vbus（说明书95%占空比限制内） */
            s_vd = 0.0f;
            s_vq = MST_STRONG_DRAG_VQ_RATIO * s_vbus;

            /* 记录强拖最后输出，用于阶段二PI积分器预置 */
            s_dragVdLast = s_vd;
            s_dragVqLast = s_vq;

            /* 反Park变换 */
            MST_InvPark(s_vd, s_vq, s_elecAngle_rad, &s_valpha, &s_vbeta);
            break;
        }

        /* ───────────────────────────────────────────
         * 阶段二：电流环接管（闭环，PI控制电流）
         * 阶段三：I-F稳态运行（闭环，电流+频率双控制）
         * ─────────────────────────────────────────── */
        case MST_STATE_TAKEOVER:
        case MST_STATE_STEADY: {
            if (s_currentLoopEnabled) {
                /* Id=0控制：只保留Iq用于产生转矩 */
                float id_error = s_idCmd - s_id;
                float iq_error = s_iqCmd - s_iq;

                /* 电流PI控制，输出为归一化电压指令（-0.95~+0.95） */
                s_vd = MST_PI_Update(&s_piId, id_error);
                s_vq = MST_PI_Update(&s_piIq, iq_error);

                /* 圆限幅：Vd/Vq合成矢量不超过最大调制比 */
                utils_saturate_vector_2d(&s_vd, &s_vq, MST_PI_OUTPUT_MAX);

                /* 反Park变换，得到静止坐标系电压 */
                MST_InvPark(s_vd, s_vq, s_elecAngle_rad, &s_valpha, &s_vbeta);
            } else {
                /* 电流环未使能时的安全输出：0电压 */
                s_valpha = 0.0f;
                s_vbeta  = 0.0f;
            }
            break;
        }

        /* ───────────────────────────────────────────
         * 其他状态：输出0电压（停机/故障/发电判断中）
         * ─────────────────────────────────────────── */
        default: {
            s_valpha = 0.0f;
            s_vbeta  = 0.0f;
            break;
        }
    }

    /* ========== Step 6: SVPWM调制并更新PWM ========== */
    if (s_state != MST_STATE_IDLE && s_state != MST_STATE_FAULT) {
        MST_UpdatePWM(s_valpha, s_vbeta);
    }

    /* ========== Step 7: 20kHz → 1kHz分频，置位慢任务标志 ========== */
    s_1kHzDivider++;
    if (s_1kHzDivider >= 20) {
        s_1kHzDivider = 0;
        s_flag_1kHz = 1;
    }
}

/* =======================================================================================================
 *  第七节：1kHz慢速状态任务（状态机，在主循环中调用）
 * ======================================================================================================= */

/**
 * @brief  慢速状态任务（1kHz）
 * @note   
 *   【调用位置】：主循环轮询 s_flag_1kHz，置位时调用
 *   【执行流程】：
 *     1. 计时器累加
 *     2. 故障检测
 *     3. 状态机切换与指令生成
 *     4. 电频率/电流指令斜坡处理
 */
void MST_SlowTask_1kHz(void) {
    if (!s_flag_1kHz) {
        return;
    }
    s_flag_1kHz = 0;

    /* 运行计时（从MST_Start()开始累计） */
    s_runTime_ms++;

    /* 更新母线电压（INA303驱动内部已滤波） */
    s_vbus = BSP_INA303_GetBusVoltage();

    /* 故障检测（过流/过压/欠压） */
    MST_DetectFault();
    if (s_faultFlags != 0 && s_state != MST_STATE_FAULT) {
        s_state = MST_STATE_FAULT;
        MST_Stop();
        return;
    }

    /* 执行状态机 */
    MST_StateMachine_1kHz();
}

/**
 * @brief  故障检测
 * @note   检测持续过流、母线过压/欠压。峰值过流由硬件Alert1处理（EXTI中断）
 */
static void MST_DetectFault(void) {
    /* 软件过流检测：任一相电流超过65A */
    if (fabsf(s_iu) > MST_OC_LIMIT_A ||
        fabsf(s_iv) > MST_OC_LIMIT_A ||
        fabsf(s_iw) > MST_OC_LIMIT_A) {
        s_faultFlags |= MST_FAULT_OVER_CURRENT;
    }

    /* 母线过压检测 */
    if (s_vbus > MST_OV_BUS_LIMIT_V) {
        s_faultFlags |= MST_FAULT_OVER_VOLTAGE;
    }

    /* 母线欠压检测 */
    if (s_vbus < MST_UV_BUS_LIMIT_V && s_vbus > 5.0f) {   /* >5V避免ADC未就绪时误触发 */
        s_faultFlags |= MST_FAULT_UNDER_VOLTAGE;
    }
}

/**
 * @brief  状态机核心逻辑（1kHz）
 * @note   三阶段切换 + 4s BEMF判断
 */
static void MST_StateMachine_1kHz(void) {
    switch (s_state) {

        /* ═══════════════════════════════════════════════════════
         * 状态：IDLE（待机）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_IDLE: {
            /* 等待 MST_Start() 启动 */
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * 状态：STRONG_DRAG（阶段一：强拖冲击，0~5ms）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_STRONG_DRAG: {
            /* 强拖阶段电流指令固定（开环不控电流，仅记录） */
            s_idCmd = 0.0f;
            s_iqCmd = 0.0f;

            /* 5ms时间到，切换到接管阶段 */
            if (s_runTime_ms >= MST_STRONG_DRAG_TIME_MS) {
                /* PI积分器预置：将强拖阶段的电压输出作为PI初值
                 * 实现无扰切换，避免5ms时刻电压跳变导致电流震荡 */
                s_piId.integral = s_dragVdLast;
                s_piIq.integral = s_dragVqLast;

                /* 使能电流环 */
                s_currentLoopEnabled = 1;

                /* 电频率目标设为强拖频率，后续逐渐加速 */
                s_elecFreqTarget_hz = MST_STRONG_DRAG_FREQ_HZ;

                s_state = MST_STATE_TAKEOVER;
            }
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * 状态：TAKEOVER（阶段二：电流环接管，5~50ms）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_TAKEOVER: {
            /* Iq指令斜坡：从当前实际电流值快速降到40A
             * 初始值设为50A（略高于40A，确保快速收敛），30ms内降到40A */
            float ramp_step = (50.0f - MST_TARGET_CURRENT_A) / (float)MST_TAKEOVER_RAMP_MS;
            uint32_t ramp_time = s_runTime_ms - MST_STRONG_DRAG_TIME_MS;
            if (ramp_time < MST_TAKEOVER_RAMP_MS) {
                s_iqCmd = 50.0f - ramp_step * (float)ramp_time;
            } else {
                s_iqCmd = MST_TARGET_CURRENT_A;
            }
            s_idCmd = MST_STEADY_ID_A;

            /* 电频率继续以加速度爬升 */
            if (s_elecFreq_hz < s_elecFreqTarget_hz) {
                s_elecFreq_hz += MST_FREQ_ACCEL_HZPS * 0.001f;
            }

            /* 50ms时间到，进入稳态 */
            if (s_runTime_ms >= MST_TAKEOVER_END_MS) {
                s_state = MST_STATE_STEADY;
                s_elecFreqTarget_hz = MST_STEADY_ELEC_FREQ_HZ;   /* 目标改为300Hz */
            }
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * 状态：STEADY（阶段三：I-F稳态，50ms ~ 4000ms）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_STEADY: {
            /* 电流指令固定 */
            s_idCmd = MST_STEADY_ID_A;
            s_iqCmd = MST_STEADY_IQ_A;

            /* 电频率以固定加速度向目标300Hz爬升 */
            if (s_elecFreq_hz < s_elecFreqTarget_hz) {
                s_elecFreq_hz += MST_FREQ_ACCEL_HZPS * 0.001f;
                if (s_elecFreq_hz > s_elecFreqTarget_hz) {
                    s_elecFreq_hz = s_elecFreqTarget_hz;
                }
            }

            /* 4s时间到，进入停机并准备BEMF检测 */
            if (s_runTime_ms >= MST_STARTUP_TOTAL_TIME_MS) {
                s_state = MST_STATE_STOP;
                /* 立即关闭PWM，准备检测反电动势 */
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
                TIM1->CCR1 = 0;
                TIM1->CCR2 = 0;
                TIM1->CCR3 = 0;
            }
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * 状态：STOP（停机，检测BEMF判断是否启动成功）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_STOP: {
            /* 延时2ms确保MOS完全关断，避免导通压降影响BEMF检测 */
            s_bemfDelayCnt++;
            if (s_bemfDelayCnt >= 2) {   /* 2ms延时（调用2次1kHz任务） */
                s_bemfDelayCnt = 0;
                /* 调用BEMF检测，判断是否进入发电模式 */
                bool bemf_ok = MST_CheckBEMF_AndEnterGenerating();
                if (bemf_ok) {
                    /* 启动成功，进入发电模式 */
                    s_state = MST_STATE_GENERATING;
                } else {
                    /* 启动失败，标记故障 */
                    s_faultFlags |= MST_FAULT_STARTUP_FAIL;
                    s_state = MST_STATE_FAULT;
                }
            }
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * 状态：GENERATING（发电模式，预留框架）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_GENERATING: {
            /* 发电模式控制逻辑需后续扩展，当前仅维持状态 */
            /* 预期实现：有源整流FOC，Id<0，母线电压闭环28.5V */
            break;
        }

        /* ═══════════════════════════════════════════════════════
         * 状态：FAULT（故障保护状态）
         * ═══════════════════════════════════════════════════════ */
        case MST_STATE_FAULT: {
            /* PWM已关闭，等待外部复位指令 */
            break;
        }

        default: {
            break;
        }
    }
}

/* =======================================================================================================
 *  第八节：BEMF检测与发电模式切入（预留框架）
 * ======================================================================================================= */

/**
 * @brief  BEMF反电动势检测与发电模式切入
 * @retval true:  检测到有效BEMF，已切入发电模式
 * @retval false: 未检测到有效BEMF，启动失败
 * @warning 
 *   【重要】本函数为预留框架，BEMF检测需要三相电压采样通道支持！
 *   当前硬件仅有：ADC1(3路INA303电流) + ADC2(1路母线电压)
 *   如需实现BEMF检测，需额外增加三相相电压分压采样电路，或利用PWM关断期的ADC采样。
 */
bool MST_CheckBEMF_AndEnterGenerating(void) {
    /* =========================================================================
     *  BEMF检测框架 - 步骤说明（后续按此框架填充）
     * =========================================================================
     *
     * 【前置条件】
     *   调用本函数前，PWM已关闭至少2ms（s_bemfDelayCnt确保），MOS完全关断。
     *
     * 【步骤1】配置ADC为软件触发单次模式（或保持现有DMA循环模式）
     *   - 如新增三相电压采样通道：配置3路ADC为单次扫描，采样10ms
     *   - 如复用现有通道：需确保INA303偏置不影响BEMF判断
     *
     * 【步骤2】采集三相端电压（Va, Vb, Vc）
     *   - 采样频率：10kHz（每相100个点，共10ms窗口）
     *   - 采样数组：float bemf_u[100], bemf_v[100], bemf_w[100];
     *
     * 【步骤3】数字滤波与过零点检测
     *   - 对每相电压做低通滤波（去除开关噪声）
     *   - 检测U-V、V-W、W-U线电压的过零点
     *   - 记录过零点时间戳 t[0], t[1], t[2]...
     *
     * 【步骤4】计算BEMF频率
     *   - 电周期 T = 相邻两个同方向过零点时间差 * 2
     *   - 电频率 f = 1/T
     *   - 取多周期平均，提高精度
     *
     * 【步骤5】判定启动是否成功
     *   - 条件1: f > 250Hz（对应机械转速 > 1000rpm）
     *   - 条件2: 频率稳定（连续3个周期偏差 < 10%）
     *   - 条件3: 三相电压对称（幅值差异 < 20%）
     *   - 全部满足 → 启动成功，进入发电模式
     *
     * 【步骤6】发电模式初始化（启动成功时执行）
     *   - s_elecFreq_hz = f;                    // 跟踪实际转速
     *   - s_elecAngle_rad = MST_EstimateAngleFromBEMF(); // 估计初始电角度
     *   - s_idCmd = -30.0f;                     // Id<0，发电（电流流向母线）
     *   - s_iqCmd = 0.0f;                       // Iq=0，单位功率因数
     *   - 启动电压外环PI：目标母线28.5V，输出作为Id指令
     *   - 重新使能PWM，切入有源整流FOC
     *
     * 【硬件改造提示】
     *   当前板子无三相电压采样，推荐方案：
     *   在电机三相端子与GND之间增加电阻分压（如100k:10k），
     *   分压后接入MCU的3个ADC通道（需确认ADC通道是否够用）。
     * ========================================================================= */

    (void)0;  /* 占位，防止空函数警告 */

    /* 当前阶段：默认返回true，便于系统联调。后续实现步骤2~6后删除此行。 */
    return true;
}

/* =======================================================================================================
 *  第九节：查询接口实现
 * ======================================================================================================= */

MST_State_t MST_GetState(void)          { return s_state; }
uint8_t MST_GetFaultFlags(void)         { return s_faultFlags; }
void MST_ClearFault(void)               { s_faultFlags = 0; s_state = MST_STATE_IDLE; }

float MST_GetIu(void)                   { return s_iu; }
float MST_GetIv(void)                   { return s_iv; }
float MST_GetIw(void)                   { return s_iw; }
float MST_GetId(void)                   { return s_id; }
float MST_GetIq(void)                   { return s_iq; }
float MST_GetBusVoltage(void)           { return s_vbus; }
float MST_GetElecAngle(void)            { return s_elecAngle_rad; }
float MST_GetElecFreqHz(void)           { return s_elecFreq_hz; }
float MST_GetMechSpeedRPM(void)         { return (s_elecFreq_hz * 60.0f) / (float)MST_POLE_PAIRS; }

void MST_SetIdPI(float Kp, float Ki)    { s_piId.Kp = Kp; s_piId.Ki = Ki; }
void MST_SetIqPI(float Kp, float Ki)    { s_piIq.Kp = Kp; s_piIq.Ki = Ki; }
