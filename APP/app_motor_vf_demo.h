/**
 * @brief    电机开环VF强拖启动测试Demo
 * @file     app_motor_vf_demo.h
 * @details  
 * @author   LB
 * @version  V1.0
 * @date     2026/04/14
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

#ifndef __APP_MOTOR_VF_DEMO_H_
#define __APP_MOTOR_VF_DEMO_H_

/* Includes ------------------------------------------------------------------*/
#include "../../bsp/bsp.h"
#include "../../matlab/VF.h"

/* Exported constants --------------------------------------------------------*/
#define UART_BUFFER_SIZE            256

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    STOP = 0x00U,
    START = 0x01U,
    FAULT = 0x03U,
}tagMotorState_T;

typedef struct
{
    tagMotorState_T State;
    int16_t SpdRef;
    float PosRef;
    float SpdKp;
    float SpdKi;
    float CurrKp;
    float CurrKi;
    uint8_t CtlMode;
}tagMotorUartCmd_T;

typedef struct
{
    float Ia;
    float Ib;
    float Ic;
    float Iq_ref;
    float Iq;
    float Id;
    float VBus;
    float ElAngle;
    float Position;
    int16_t Speed;
    uint16_t IaOffset;
    uint16_t IbOffset;
    uint16_t IcOffset;
}tagMotorFocVars_T;

typedef struct
{
    uint8_t UartRxBuffer[UART_BUFFER_SIZE];
    uint8_t UartRxComplete;
    uint8_t DataDecodingState;
}tagMotorUartRxBuf_T;

typedef struct
{
    tagMotorUartCmd_T       UART_Cmd;
    tagMotorFocVars_T       FOCVars;
    tagMotorUartRxBuf_T     UART_RxBuff;
}tagMotorApp_T;

/* Exported variables --------------------------------------------------------*/
extern tagMotorApp_T t_MotorApp;

/* Exported functions --------------------------------------------------------*/
void motor_Init(void);
void motor_FocTaskRun(void);
void motor_FocVarsUpdate(void);

#endif /* __APP_MOTOR_VF_DEMO_H_ */

/***************************** (END OF FILE) *********************************/

