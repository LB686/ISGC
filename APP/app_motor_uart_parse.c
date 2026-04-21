/**
 * @brief    电机串口通讯协议
 * @file     app_motor_uart_parse.c
 * @details  
 * @mainpage
 * @author   LB
 * @version  V1.0
 * @date     2026/04/14
 * @license  Copyright (c) 2024-2026 浙江华奕航空科技有限公司.All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "../../app/app_motor_uart_parse.h"

/* Private variables ---------------------------------------------------------*/
static uint16_t s_usart3_rx_index = 0;

/**
 * @brief UART调试发送缓冲区
 * @details 32字节，前28字节存放7个float数据，最后4字节保留初始值0x7F800000(+inf)作为帧尾同步标志
 */
uint8_t uart_debug_buff[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80, 0x7F
};

/* Private function prototypes -----------------------------------------------*/
static float my_strtof(const char *str);
static uint8_t split_string(const char *str, char delimiter, char **left, char **right);

/* Public functions ---------------------------------------------------------*/
/*
*********************************************************************************************************
*	函 数 名: uart_MotorDataSend
*	功能说明: 将电机FOC变量打包到UART发送缓冲区，并通过DMA发送
*	形    参: 无
*	返 回 值: 无
*	备    注: 发送32字节(8个float)，其中前7个为实时数据，最后4字节为帧尾标志(+inf)
*********************************************************************************************************
*/
void uart_MotorDataSend(void)
{
    float temp_f[7];

    temp_f[0] = (float)t_MotorApp.FOCVars.Ia;
    temp_f[1] = (float)t_MotorApp.FOCVars.Ib;
    temp_f[2] = (float)rtY.tABC[0];
    temp_f[3] = (float)rtY.tABC[1];
    temp_f[4] = (float)rtY.tABC[2];
    temp_f[5] = 0.0f;
    temp_f[6] = 0.0f;

    /* 仅覆盖前28字节，保留最后4字节的帧尾同步标志 */
    memcpy(uart_debug_buff, (uint8_t *)temp_f, sizeof(temp_f));

    usart3_send((uint8_t *)uart_debug_buff, sizeof(uart_debug_buff));
}

/*
*********************************************************************************************************
*	函 数 名: uart_MotorDataParse
*	功能说明: 解析UART接收到的字符串命令，目前支持P1=xxx格式的启动/停止控制
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void uart_MotorDataParse(void)
{
    char *frame_data;
    float parsed_value;

    if (t_MotorApp.UART_RxBuff.DataDecodingState == 1)
    {
        if (split_string((char *)t_MotorApp.UART_RxBuff.UartRxBuffer, '=', NULL, &frame_data) == 0)
        {
            parsed_value = my_strtof(frame_data);

            if (t_MotorApp.UART_RxBuff.UartRxBuffer[0] == 'P' &&
                t_MotorApp.UART_RxBuff.UartRxBuffer[1] == '1')
            {
                if ((uint8_t)parsed_value == 0x01)
                {
                    t_MotorApp.UART_Cmd.State = START;
                }
                else
                {
                    t_MotorApp.UART_Cmd.State = STOP;
                }
            }

            t_MotorApp.UART_RxBuff.DataDecodingState = 0;
            memset(t_MotorApp.UART_RxBuff.UartRxBuffer, 0, UART_BUFFER_SIZE);
        }
    }
}

/*
*********************************************************************************************************
*	函 数 名: usart3_frame_assemble
*	功能说明: 解析串口3接收到的单个字节，完成命令组帧，由usart_dataparse（）函数调用。
*	形    参: b -- 从FIFO中取出的字节数据
*	返 回 值: 无
*	备    注: 以换行符(\r 或 \n)作为一帧结束标志，收到后设置 DataDecodingState
*********************************************************************************************************
*/
void usart3_frame_assemble (uint8_t b)
{
    /* 缓冲区溢出保护：超限时清空重来 */
    if (s_usart3_rx_index >= UART_BUFFER_SIZE)
    {
        s_usart3_rx_index = 0;
        memset(t_MotorApp.UART_RxBuff.UartRxBuffer, 0, UART_BUFFER_SIZE);
    }

    /* 遇到换行符认为一帧结束 */
    if (b == '\n' || b == '\r')
    {
        t_MotorApp.UART_RxBuff.UartRxBuffer[s_usart3_rx_index] = '\0';
        t_MotorApp.UART_RxBuff.DataDecodingState = 1;
        s_usart3_rx_index = 0;
    }
    else
    {
        t_MotorApp.UART_RxBuff.UartRxBuffer[s_usart3_rx_index++] = b;
    }
}

/*
*********************************************************************************************************
*	函 数 名: split_string
*	功能说明: 在字符串中查找指定分隔符，返回分隔符左右两部分的指针
*	形    参: str        -- 输入字符串
*             delimiter  -- 分隔符字符
*             left       -- 输出参数，左半部分指针
*             right      -- 输出参数，右半部分指针
*	返 回 值: 0 -- 找到分隔符
*             1 -- 未找到分隔符
*********************************************************************************************************
*/
static uint8_t split_string(const char *str, char delimiter, char **left, char **right)
{
    if (left != NULL)
    {
        *left = (char *)str;
    }
    if (right != NULL)
    {
        *right = NULL;
    }

    for (; *str != '\0'; str++)
    {
        if (*str == delimiter)
        {
            if (right != NULL)
            {
                *right = (char *)(str + 1);
            }
            return 0;
        }
    }

    return 1;
}

/*
*********************************************************************************************************
*	函 数 名: my_strtof
*	功能说明: 手动实现字符串转float，支持整数、小数和科学计数法
*	形    参: str -- 待转换的字符串指针
*	返 回 值: 转换后的float值
*	备    注: 适用于未提供标准库strtof的嵌入式环境
*********************************************************************************************************
*/
static float my_strtof(const char *str)
{
    float result = 0.0f;
    float sign = 1.0f;
    float fraction = 1.0f;
    int exponent = 0;
    int exp_sign = 1;
    int state = 0; /* 0: 整数部分, 1: 小数部分, 2: 指数部分 */

    /* 跳过前导空白字符 */
    while (*str == ' ' || *str == '\t' || *str == '\n' ||
           *str == '\v' || *str == '\f' || *str == '\r')
    {
        str++;
    }

    /* 处理正负号 */
    if (*str == '-')
    {
        sign = -1.0f;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }

    /* 解析数字 */
    while (*str != '\0')
    {
        if (*str >= '0' && *str <= '9')
        {
            if (state == 0)
            {
                result = result * 10.0f + (*str - '0');
            }
            else if (state == 1)
            {
                fraction /= 10.0f;
                result += (*str - '0') * fraction;
            }
            else if (state == 2)
            {
                exponent = exponent * 10 + (*str - '0');
            }
        }
        else if (*str == '.' && state == 0)
        {
            state = 1;
        }
        else if ((*str == 'e' || *str == 'E') && state < 2)
        {
            state = 2;
            str++;
            if (*str == '-')
            {
                exp_sign = -1;
                str++;
            }
            else if (*str == '+')
            {
                str++;
            }
            continue;
        }
        else
        {
            break;
        }
        str++;
    }

    /* 应用指数 */
    if (state == 2)
    {
        while (exponent > 0)
        {
            if (exp_sign > 0)
            {
                result *= 10.0f;
            }
            else
            {
                result /= 10.0f;
            }
            exponent--;
        }
    }

    return sign * result;
}
