/**
 * @brief    电机控制数学工具库实现
 * @file     app_motor_math.c
 * @details  FOC快速数学函数实现，包含sin/cos/atan2/SVPWM/Clark/Park变换
 * @author   LB
 * @version  V1.0
 * @date     2026/04/23
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

#include "../../app/app_motor_math.h"

/* =======================================================================================================
 *  第一节：快速 sin/cos 实现（抛物线逼近算法）
 * ======================================================================================================= */

/*
*********************************************************************************************************
*   函 数 名: MOTOR_SincosBetter
*   功能说明: 快速正弦/余弦计算
*   形    参:
*       angle: 输入角度，单位弧度
*       sin:   输出正弦值指针
*       cos:   输出余弦值指针
*   返 回 值: 无
*********************************************************************************************************
*/
void MOTOR_SincosBetter(float angle, float *sin, float *cos) {
    /* 将输入角度归一化到 [-π, +π] 区间 */
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    while (angle >  M_PI) {
        angle -= 2.0f * M_PI;
    }

    /* 计算 sin(angle)
     * 先用抛物线逼近：sin(x) ≈ 1.273x - 0.405x² (x>0) 或 +0.405x² (x<0)
     * 再用二次校正项提高精度：0.225*(sin² - sin) + sin
     */
    if (angle < 0.0f) {
        *sin = 1.27323954f * angle + 0.405284735f * angle * angle;
        if (*sin < 0.0f) {
            *sin = 0.225f * (*sin * -*sin - *sin) + *sin;
        } else {
            *sin = 0.225f * (*sin * *sin - *sin) + *sin;
        }
    } else {
        *sin = 1.27323954f * angle - 0.405284735f * angle * angle;
        if (*sin < 0.0f) {
            *sin = 0.225f * (*sin * -*sin - *sin) + *sin;
        } else {
            *sin = 0.225f * (*sin * *sin - *sin) + *sin;
        }
    }

    /* 计算 cos(angle) = sin(angle + π/2)，复用sin计算逻辑 */
    angle += 0.5f * M_PI;
    if (angle >  M_PI) {
        angle -= 2.0f * M_PI;
    }

    if (angle < 0.0f) {
        *cos = 1.27323954f * angle + 0.405284735f * angle * angle;
        if (*cos < 0.0f) {
            *cos = 0.225f * (*cos * -*cos - *cos) + *cos;
        } else {
            *cos = 0.225f * (*cos * *cos - *cos) + *cos;
        }
    } else {
        *cos = 1.27323954f * angle - 0.405284735f * angle * angle;
        if (*cos < 0.0f) {
            *cos = 0.225f * (*cos * -*cos - *cos) + *cos;
        } else {
            *cos = 0.225f * (*cos * *cos - *cos) + *cos;
        }
    }
}

/* =======================================================================================================
 *  第二节：快速 atan2 实现
 * ======================================================================================================= */

/*
*********************************************************************************************************
*   函 数 名: MOTOR_Atan2
*   功能说明: 快速 atan2 计算
*   形    参:
*       y: 对边
*       x: 邻边
*   返 回 值: 角度，单位弧度，范围 -π~+π
*********************************************************************************************************
*/
float MOTOR_Atan2(float y, float x) {
    /* 加极小值防止 0/0 除零异常 */
    float abs_y = fabsf(y) + 1e-20f;
    float angle;

    if (x >= 0.0f) {
        float r = (x - abs_y) / (x + abs_y);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + (M_PI / 4.0f);
    } else {
        float r = (x + abs_y) / (abs_y - x);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * M_PI / 4.0f);
    }

    MOTOR_NAN_ZERO(angle);
    return (y < 0.0f) ? (-angle) : angle;
}

/* =======================================================================================================
 *  第三节：SVPWM 空间矢量调制
 * ======================================================================================================= */

/*
*********************************************************************************************************
*   函 数 名: MOTOR_SVM
*   功能说明: 7段式对称SVPWM调制
*   形    参:
*       alpha:  归一化电压 Vα（调制比，范围 -1.0 ~ +1.0）
*       beta:   归一化电压 Vβ（调制比，范围 -1.0 ~ +1.0）
*       maxMod: 最大调制比限制（如0.95）
*       pwmArr: PWM计数器峰值（ARR值，如3999）
*       tAout:  输出A相PWM计数值指针
*       tBout:  输出B相PWM计数值指针
*       tCout:  输出C相PWM计数值指针
*       sector: 输出扇区号指针（1~6，调试用）
*   返 回 值: 无
*********************************************************************************************************
*/
void MOTOR_SVM(float alpha, float beta, float maxMod, uint32_t pwmArr,
               uint32_t *tAout, uint32_t *tBout, uint32_t *tCout, uint32_t *sector) {
    uint32_t sec;

    /* ========== 扇区判断（基于 Vα/Vβ 所在象限）==========
     *              β↑
     *           Ⅱ  │  Ⅰ
     *         3 │ 2│ 1
     *       ────┼──┼────→ α
     *         4 │ 5│ 6
     *           Ⅲ  │  Ⅳ
     */
    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            sec = (MOTOR_ONE_BY_SQRT3 * beta > alpha) ? 2 : 1;
        } else {
            sec = (-MOTOR_ONE_BY_SQRT3 * beta > alpha) ? 3 : 2;
        }
    } else {
        if (alpha >= 0.0f) {
            sec = (-MOTOR_ONE_BY_SQRT3 * beta > alpha) ? 5 : 6;
        } else {
            sec = (MOTOR_ONE_BY_SQRT3 * beta > alpha) ? 4 : 5;
        }
    }

    /* ========== 根据扇区计算各相PWM占空比 ========== */
    int tA, tB, tC;
    switch (sec) {
        case 1: {
            int t1 = (int)((alpha - MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            int t2 = (int)((MOTOR_TWO_BY_SQRT3 * beta) * (float)pwmArr);
            tA = ((int)pwmArr + t1 + t2) / 2;
            tB = tA - t1;
            tC = tB - t2;
            break;
        }
        case 2: {
            int t2 = (int)((alpha + MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            int t3 = (int)((-alpha + MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            tB = ((int)pwmArr + t2 + t3) / 2;
            tA = tB - t3;
            tC = tA - t2;
            break;
        }
        case 3: {
            int t3 = (int)((MOTOR_TWO_BY_SQRT3 * beta) * (float)pwmArr);
            int t4 = (int)((-alpha - MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            tB = ((int)pwmArr + t3 + t4) / 2;
            tC = tB - t3;
            tA = tC - t4;
            break;
        }
        case 4: {
            int t4 = (int)((-alpha + MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            int t5 = (int)((-MOTOR_TWO_BY_SQRT3 * beta) * (float)pwmArr);
            tC = ((int)pwmArr + t4 + t5) / 2;
            tB = tC - t5;
            tA = tB - t4;
            break;
        }
        case 5: {
            int t5 = (int)((-alpha - MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            int t6 = (int)((alpha - MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            tC = ((int)pwmArr + t5 + t6) / 2;
            tA = tC - t5;
            tB = tA - t6;
            break;
        }
        case 6: {
            int t6 = (int)((-MOTOR_TWO_BY_SQRT3 * beta) * (float)pwmArr);
            int t1 = (int)((alpha + MOTOR_ONE_BY_SQRT3 * beta) * (float)pwmArr);
            tA = ((int)pwmArr + t6 + t1) / 2;
            tC = tA - t1;
            tB = tC - t6;
            break;
        }
        default: {
            tA = tB = tC = (int)pwmArr / 2;
            break;
        }
    }

    /* ========== 输出限幅（防止超过最大调制比）========== */
    int t_max = (int)((float)pwmArr * (1.0f - (1.0f - maxMod) * 0.5f));
    if (tA < 0) tA = 0; else if (tA > t_max) tA = t_max;
    if (tB < 0) tB = 0; else if (tB > t_max) tB = t_max;
    if (tC < 0) tC = 0; else if (tC > t_max) tC = t_max;

    *tAout = (uint32_t)tA;
    *tBout = (uint32_t)tB;
    *tCout = (uint32_t)tC;
    *sector = sec;
}

/* =======================================================================================================
 *  第四节：FOC 坐标变换实现
 * ======================================================================================================= */

/*
*********************************************************************************************************
*   函 数 名: MOTOR_Clarke
*   功能说明: Clark变换（三相 → 两相静止坐标系）
*   形    参:
*       ia:     A相电流
*       ib:     B相电流
*       ialpha: 输出 Iα 指针
*       ibeta:  输出 Iβ 指针
*   返 回 值: 无
*********************************************************************************************************
*/
void MOTOR_Clarke(float ia, float ib, float *ialpha, float *ibeta) {
    *ialpha = ia;
    *ibeta  = MOTOR_ONE_BY_SQRT3 * ia + MOTOR_TWO_BY_SQRT3 * ib;
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_Park
*   功能说明: Park变换（静止 → 旋转坐标系）
*   形    参:
*       ialpha: 输入 Iα
*       ibeta:  输入 Iβ
*       theta:  电角度，单位弧度
*       id:     输出 Id 指针
*       iq:     输出 Iq 指针
*   返 回 值: 无
*********************************************************************************************************
*/
void MOTOR_Park(float ialpha, float ibeta, float theta, float *id, float *iq) {
    float sin_t, cos_t;
    MOTOR_SincosBetter(theta, &sin_t, &cos_t);
    *id =  ialpha * cos_t + ibeta * sin_t;
    *iq = -ialpha * sin_t + ibeta * cos_t;
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_InvPark
*   功能说明: 反Park变换（旋转 → 静止坐标系）
*   形    参:
*       vd:     输入 Vd
*       vq:     输入 Vq
*       theta:  电角度，单位弧度
*       valpha: 输出 Vα 指针
*       vbeta:  输出 Vβ 指针
*   返 回 值: 无
*********************************************************************************************************
*/
void MOTOR_InvPark(float vd, float vq, float theta, float *valpha, float *vbeta) {
    float sin_t, cos_t;
    MOTOR_SincosBetter(theta, &sin_t, &cos_t);
    *valpha = vd * cos_t - vq * sin_t;
    *vbeta  = vd * sin_t + vq * cos_t;
}
