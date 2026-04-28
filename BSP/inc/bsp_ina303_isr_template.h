/*
*********************************************************************************************************
*   文件名称 : bsp_ina303_isr_template.h
*   说    明 : ADC 中断服务函数模板
*              请将这些代码复制到 Core/Src/stm32f4xx_it.c 的对应位置
*              注意：使用本模板后，应删除或注释 CUBEMX 生成的 HAL_ADCEx_InjectedConvCpltCallback
*********************************************************************************************************
*/

/* ===================== 复制到 stm32f4xx_it.c 顶部（Include 区域）==================== */
/* USER CODE BEGIN Includes */
#include "bsp_ina303.h"         /* INA303 驱动头文件 */
#include "app_motor_startup.h"  /* MOTOR_FastControlTask() 声明 */
/* USER CODE END Includes */

/* ===================== 复制到 ADC_IRQHandler 中 ===================== */
/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
    /* ADC1 JEOC：三相电流（20kHz，必须处理） */
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC)) {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
        BSP_INA303_ADC1_JEOC_IRQHandler();
    }

    /* ADC2 JEOC：三相 BEMF（仅在 INA303_USE_ADC2_BEMF=1 时处理） */
#if INA303_USE_ADC2_BEMF
    if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOC)) {
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
        BSP_INA303_ADC2_JEOC_IRQHandler();
    }
#endif

    /* 注入组完成后调用 20kHz FOC 快速控制任务 */
    if (BSP_INA303_IsInjectedDone()) {
        MOTOR_FastControlTask();
    }
  /* USER CODE END ADC_IRQn 0 */

  /* HAL 默认处理（EOC, OVR, AWD 等其他标志） */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  HAL_ADC_IRQHandler(&hadc3);
}

/* ===================== 复制到 TIM5_IRQHandler / HAL_TIM_PeriodElapsedCallback 中 ===================== */
/**
  * @brief  TIM 周期中断回调
  * @note   TIM5 1kHz 中断：FOC 慢任务 + INA303 滤波/保护/ADC3 轮询
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim5) {
        MOTOR_SlowTask_1kHz();    /* 状态机、频率斜坡 */
        BSP_INA303_FastTask();    /* 滤波、物理量计算、保护、ADC3 轮询 */
    }
}

/* ===================== 可选：发电模式检测前启动 ADC2 BEMF ===================== */
/* 在需要 BEMF 时（如 4s 后停止 PWM 进入发电检测前）调用： */
// BSP_INA303_ADC2_Start();

/***************************** (END OF FILE) *********************************/
