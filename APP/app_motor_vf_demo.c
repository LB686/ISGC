/**
 * @brief    电机开环VF强拖启动测试Demo
 * @file     app_motor_vf_demo.c
 * @details  
 * @mainpage
 * @author   LB
 * @version  V1.0
 * @date     2026/04/14
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "../../app/app_motor_vf_demo.h"

#include "dac.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "opamp.h"
#include "comp.h"

#include "../../bsp/bsp.h"
#include "../../app/app_motor_uart_parse.h"


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static void motor_InitVar(void);
static void Motor_StopTSK(void);
static void Motor_StartTSK(void);
static float Get_Vbus(void);
static void motor_StateKeyUpdata(void);

/* Public variables ----------------------------------------------------------*/
/* 对外提供的统一操作接口 */
tagMotorApp_T t_MotorApp;

/* Public functions ---------------------------------------------------------*/

/*
*********************************************************************************************************
*	函 数 名: motor_InitVar
*	功能说明: 初始化电机相关变量. 该函数被 motor_Init() 调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void motor_Init(void)
{
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

	HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start( &hadc2, ADC_SINGLE_ENDED);

	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);
	__HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_JEOC);

	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);

	HAL_TIM_Base_Start( &htim1);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_4);

	__HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_IDLEF);

	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2958);
	HAL_DAC_Start(&hdac3,DAC_CHANNEL_1);

	HAL_COMP_Start(&hcomp1);

	rtU.Freq = 4;
	rtU.ud = 0;
	rtU.uq = 2;

	motor_InitVar();	
}


/*
*********************************************************************************************************
*	函 数 名: motor_InitVar
*	功能说明: 初始化电机相关变量. 该函数被 motor_Init() 调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void motor_InitVar(void)
{
    t_MotorApp.UART_Cmd.State = STOP;
    t_MotorApp.UART_Cmd.SpdRef = 0;
    t_MotorApp.UART_Cmd.PosRef = 0;

    t_MotorApp.UART_RxBuff.UartRxComplete = 0;
    t_MotorApp.UART_RxBuff.DataDecodingState = 0;
}


/*
*********************************************************************************************************
*	函 数 名: motor_FocTaskRun
*	功能说明: 电机FOC控制任务函数. 该函数需要被主任务1ms调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void motor_FocTaskRun(void)
{
	static uint8_t MotorState = 0;

	/* 电机数据上传，供VF上位机显示*/
	uart_MotorDataSend();	

	/* 解析UART接收的命令，更新控制状态和目标值 */
	usart_dataparse();	

	/* 根据解析结果更新电机控制状态 */
	uart_MotorDataParse();

	/* 判断按键状态更新，改变电机控制状态 */
	motor_StateKeyUpdata();		

	/* 根据控制状态启动或停止电机，状态由KEY3，或串口数据控制 */
	if(MotorState != t_MotorApp.UART_Cmd.State)
	{
		if(t_MotorApp.UART_Cmd.State == START)
		{
			Motor_StartTSK();
		}
		else
		{
			Motor_StopTSK();
		}
	}
	MotorState = t_MotorApp.UART_Cmd.State;

	/* 检测过流保护，进入故障状态 */
	if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK))
	{
		t_MotorApp.UART_Cmd.State = FAULT;
		__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
	}

	t_MotorApp.FOCVars.VBus = Get_Vbus();
	rtU.v_bus = t_MotorApp.FOCVars.VBus;
}

/*
*********************************************************************************************************
*	函 数 名: motor_FocVarsUpdate
*	功能说明: 更新电机FOC相关变量. 该函数需要被ADC回调函数调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void motor_FocVarsUpdate(void){
	static uint8_t cnt;
	static uint8_t OffsetCalibration = 0;
	static uint16_t IaU16, IbU16, IcU16;
	
	/* 前10次采样进行电流传感器偏移校准，后续采样根据偏移值计算实际电流值 */
	if (OffsetCalibration == 0)
	{
		cnt++;
		t_MotorApp.FOCVars.IaOffset += hadc1.Instance->JDR1;
		t_MotorApp.FOCVars.IbOffset += hadc2.Instance->JDR1;
		t_MotorApp.FOCVars.IcOffset += hadc1.Instance->JDR2;
		if (cnt >= 10)
		{
		OffsetCalibration = 1;
		t_MotorApp.FOCVars.IaOffset = t_MotorApp.FOCVars.IaOffset / 10;
		t_MotorApp.FOCVars.IbOffset = t_MotorApp.FOCVars.IbOffset / 10;
		t_MotorApp.FOCVars.IcOffset = t_MotorApp.FOCVars.IcOffset / 10;
		}
	}
	else
	{
		IaU16 = hadc1.Instance->JDR1;
		IbU16 = hadc2.Instance->JDR1;
		IcU16 = hadc1.Instance->JDR2;

		
		/* 根据ADC采样值和偏移值计算实际电流值，单位为安培 */
		t_MotorApp.FOCVars.Ia = (IaU16 - t_MotorApp.FOCVars.IaOffset) * 0.02197265625f;
		t_MotorApp.FOCVars.Ib = (IbU16 - t_MotorApp.FOCVars.IbOffset) * 0.02197265625f;
		t_MotorApp.FOCVars.Ic = (IcU16 - t_MotorApp.FOCVars.IcOffset) * 0.02197265625f;
		

		/* VF强脱启动电机 */
		VF_step();	

		TIM1->CCR1 = rtY.tABC[0];
		TIM1->CCR2 = rtY.tABC[1];
		TIM1->CCR3 = rtY.tABC[2];
		TIM1->CCR5 = rtY.tABC[2];
	}
}

/*
*********************************************************************************************************
*	函 数 名: Motor_StopTSK
*	功能说明: 停止电机任务函数，停止PWM输出。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void Motor_StopTSK(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/*
*********************************************************************************************************
*	函 数 名: Motor_StartTSK
*	功能说明: 启动电机任务函数，启动PWM输出。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void Motor_StartTSK(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/*
*********************************************************************************************************
*	函 数 名: Get_Vbus
*	功能说明: 获取母线电压。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static float Get_Vbus(void)
{
    uint16_t VBusU16 = 0;
    if (HAL_ADC_Start(&hadc2) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc2, 2) == HAL_OK) {  // 2ms超时
            VBusU16 = HAL_ADC_GetValue(&hadc2);
        }
    }
    return (float)VBusU16 * 3.3f / 4096.0f * 26.0f;
}

/*
*********************************************************************************************************
*	函 数 名: motor_StateKeyUpdata
*	功能说明: 根据按键状态更新电机控制状态。该函数被 motor_FocTaskRun() 调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void motor_StateKeyUpdata(void){

    uint8_t ucKeyCode;		/* 按键代码 */

    ucKeyCode = bsp_GetKey();	/* 读取键值, 无键按下时返回 KEY_NONE = 0 */
	if (ucKeyCode != KEY_NONE)
	{
		switch (ucKeyCode)
		{
			case KEY_DOWN_K1:			/* K1键按下 */
				
				break;
			case KEY_DOWN_K2:			/* K2键按下 */
				
				break;
			case KEY_DOWN_K3:			/* K3键按下 */
				
				t_MotorApp.UART_Cmd.State = (t_MotorApp.UART_Cmd.State == START) ? STOP : START;
				break;
			default:
				/* 其它的键值不处理 */
				break;
		}
	
	}

} 

/* --------------------------回调函数 ----------------------------*/
/*
*********************************************************************************************************
*	函 数 名: HAL_ADCEx_InjectedConvCpltCallback
*	功能说明: ADC注入通道转换完成回调函数，在该函数中调用motor_FocVarsUpdate()更新FOC相关变量，并执行FOC控制算法。
*	形    参: 
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if (hadc == &hadc1)
  {
    motor_FocVarsUpdate();
  }
}


