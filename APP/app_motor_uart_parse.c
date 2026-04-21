/*
*********************************************************************************************************
*
*   模块名称 : 电机串口协议解析应用模块
*   文件名称 : app_motor_uart_parse.c
*   版    本 : V1.0
*   说    明 : 基于 2000W 启发一体系统 RS232 协议进行帧解析与命令分发。
*              采用状态机逐字节解析帧格式，支持 CRC16(Modbus) 校验。
*              命令解析采用"查表+函数指针"方式，新增命令无需修改解析主干逻辑。
*   修改记录 :
*       版本号  日期         作者      说明
*       V1.0    2026-04-21   xxx      应用创建
*
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "../../app/app_motor_uart_parse.h"
#include <string.h>

/* Private define -------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* 帧解析状态机 */
typedef enum {
    FRAME_STATE_IDLE = 0,   /* 等待帧头 */
    FRAME_STATE_LEN,        /* 接收长度 */
    FRAME_STATE_TYPE,       /* 接收类型 */
    FRAME_STATE_DATA,       /* 接收数据域 */
    FRAME_STATE_CRC_L,      /* 接收 CRC 低字节 */
    FRAME_STATE_CRC_H,      /* 接收 CRC 高字节 */
    FRAME_STATE_TAIL,       /* 等待帧尾 */
} FRAME_PARSE_STATE_E;

/* 命令解析表项类型 */
typedef struct {
    uint8_t     cmdType;            /* 命令码 */
    void        (*parseFunc)(const uint8_t *data, uint8_t len); /* 解析函数指针 */
} tagMOTOR_CmdTableItem_T;

/* Private variables ----------------------------------------------------------*/

static FRAME_PARSE_STATE_E          s_frameState;                       /* 当前解析状态 */
static uint8_t                      s_rxBuf[MOTOR_UART_MAX_DATA_LEN];   /* 连续接收缓存(LEN+TYPE+DATA) */
static uint8_t                      s_rxIndex;                          /* 接收缓存写入索引 */
static uint8_t                      s_rxLen;                            /* LEN 字段值(TYPE+DATA长度) */
static uint8_t                      s_crcL;                             /* 接收到的 CRC 低字节 */
static uint8_t                      s_crcH;                             /* 接收到的 CRC 高字节 */

static tagMOTOR_CmdTableItem_T      s_telemetry;                        /* 最新遥测数据副本 */
static uint8_t                      s_telUpdated;                       /* 遥测更新标志 */

static MOTOR_RxCallback_t           s_rxCallback;                       /* 通用接收回调 */
static MOTOR_TelUpdateCallback_t    s_telCallback;                      /* 遥测更新回调 */

/* Private function prototypes ------------------------------------------------*/

static uint16_t MOTOR_UART_CalcCRC16(const uint8_t *data, uint16_t len);
static void     MOTOR_DispatchCmd(uint8_t cmdType, const uint8_t *data, uint8_t len);
static void     MOTOR_FrameComplete(void);

/* 各命令解析函数前向声明 */

static void MOTOR_ParseStart(const uint8_t *data, uint8_t len);
static void MOTOR_ParseSetI(const uint8_t *data, uint8_t len);
static void MOTOR_ParseSetSpd(const uint8_t *data, uint8_t len);
static void MOTOR_ParseTelemetry(const uint8_t *data, uint8_t len);

/* 小端数据解析辅助函数 */
static int16_t  MOTOR_ParseInt16LE(const uint8_t *p);
static uint16_t MOTOR_ParseUInt16LE(const uint8_t *p);
static int32_t  MOTOR_ParseInt32LE(const uint8_t *p);
static uint32_t MOTOR_ParseUInt32LE(const uint8_t *p);

/* ===============================================================================================
 *                                  命令解析表(核心可扩展点)
 * 说明: 新增命令时，只需编写对应的 parseFunc 函数，并在此表中添加一行映射关系即可。
 *       无需修改状态机主干逻辑，实现命令解析的"插件化"。
 * =============================================================================================== */
static const tagMOTOR_CmdTableItem_T s_cmdTable[] = {
    {MOTOR_CMD_TEL,     MOTOR_ParseTelemetry},
    {MOTOR_CMD_START,   MOTOR_ParseStart},
    {MOTOR_CMD_SET_I,   MOTOR_ParseSetI},
    {MOTOR_CMD_SET_SPD, MOTOR_ParseSetSpd}, 
    /* 在此追加新命令 */
};

#define CMD_TABLE_SIZE  (sizeof(s_cmdTable) / sizeof(s_cmdTable[0]))

/* Private functions ----------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: MOTOR_UART_CalcCRC16
*   功能说明: 计算 Modbus CRC-16 校验值
*   形    参: data  待校验数据指针
*             len   数据长度(字节)
*   返 回 值: CRC16 结果
*********************************************************************************************************
*/
static uint16_t MOTOR_UART_CalcCRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ MOTOR_UART_CRC_POLY;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_ParseInt16LE / MOTOR_ParseUInt16LE / MOTOR_ParseInt32LE / MOTOR_ParseUInt32LE
*   功能说明: 小端序数据解析辅助函数
*********************************************************************************************************
*/
static int16_t MOTOR_ParseInt16LE(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t MOTOR_ParseUInt16LE(const uint8_t *p)
{
    return ((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t MOTOR_ParseInt32LE(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static uint32_t MOTOR_ParseUInt32LE(const uint8_t *p)
{
    return ((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
            ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* ===============================================================================================
 *                                  命令解析函数实现
 * =============================================================================================== */

/*
*********************************************************************************************************
*   函 数 名: MOTOR_ParseStart
*   功能说明: 启动命令响应解析，具体功能待实现
*   形    参: data  数据域指针
*             len   数据域长度
*   返 回 值: 无
*********************************************************************************************************
*/
static void MOTOR_ParseStart(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    /* 根据协议，启动命令数据域为 0x00017318，可在此扩展响应处理 */
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_ParseSetI
*   功能说明: 设置电流命令响应解析，具体功能待实现
*   形    参: data  数据域指针
*             len   数据域长度
*   返 回 值: 无
*********************************************************************************************************
*/
static void MOTOR_ParseSetI(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    /* 数据域为 int32 × 0.01A，如需确认返回值可在此解析 */
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_ParseSetSpd
*   功能说明: 设置转速命令响应解析，具体功能待实现
*   形    参: data  数据域指针
*             len   数据域长度
*   返 回 值: 无
*********************************************************************************************************
*/
static void MOTOR_ParseSetSpd(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    /* 数据域为 int32 × 1rpm，如需确认返回值可在此解析 */
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_ParseTelemetry
*   功能说明: 遥测数据帧解析(57字节数据域)
*   形    参: data  数据域指针
*             len   数据域长度
*   返 回 值: 无
*********************************************************************************************************
*/
static void MOTOR_ParseTelemetry(const uint8_t *data, uint8_t len)
{
    if (len < 57) {
        return; /* 长度不足保护 */
    }

    s_telemetry.mos_temp      = MOTOR_ParseInt16LE(&data[0]);    /* BYTE1-2 */
    s_telemetry.mot_temp      = MOTOR_ParseInt16LE(&data[2]);    /* BYTE3-4 */
    s_telemetry.phase_current = MOTOR_ParseInt32LE(&data[4]);    /* BYTE5-8 */
    s_telemetry.bus_current   = MOTOR_ParseInt32LE(&data[8]);    /* BYTE9-12 */
    s_telemetry.id_current    = MOTOR_ParseInt32LE(&data[12]);   /* BYTE13-16 */
    s_telemetry.iq_current    = MOTOR_ParseInt32LE(&data[16]);   /* BYTE17-20 */
    s_telemetry.duty          = MOTOR_ParseUInt16LE(&data[20]);  /* BYTE21-22 */
    s_telemetry.elec_speed    = MOTOR_ParseInt32LE(&data[22]);   /* BYTE23-26 */
    s_telemetry.bus_voltage   = MOTOR_ParseUInt16LE(&data[26]);  /* BYTE27-28 */
    /* BYTE29-52 保留/未定义，预留扩展 */
    s_telemetry.fault         = data[52];                        /* BYTE53 */
    s_telemetry.position      = MOTOR_ParseUInt32LE(&data[53]);  /* BYTE54-57 */

    s_telUpdated = 1;

    /* 触发遥测更新回调(若已注册) */
    if (s_telCallback != NULL) {
        s_telCallback(&s_telemetry);
    }
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_DispatchCmd
*   功能说明: 根据命令码查表分发到对应的解析函数
*   形    参: cmdType  命令码
*             data     数据域指针
*             len      数据域长度
*   返 回 值: 无
*********************************************************************************************************
*/
static void MOTOR_DispatchCmd(uint8_t cmdType, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    for (i = 0; i < CMD_TABLE_SIZE; i++) {
        if (s_cmdTable[i].cmdType == cmdType) {
            if (s_cmdTable[i].parseFunc != NULL) {
                s_cmdTable[i].parseFunc(data, len);
            }
            break;
        }
    }

    /* 触发通用接收回调(若已注册)，无论命令是否在表中均可被上层感知 */
    if (s_rxCallback != NULL) {
        s_rxCallback((MOTOR_CMD_TYPE_E)cmdType, data, len);
    }
}

/*
*********************************************************************************************************
*   函 数 名: MOTOR_FrameComplete
*   功能说明: 帧接收完成后的 CRC 校验与命令分发
*   形    参: 无(使用模块静态变量)
*   返 回 值: 无
*********************************************************************************************************
*/
static void MOTOR_FrameComplete(void)
{
    uint16_t calcCrc;
    uint16_t recvCrc;
    uint8_t  cmdType;
    uint8_t  dataLen;

    /* CRC 计算范围: LEN + TYPE + DATA，长度 = s_rxLen + 1(LEN字段本身) */
    calcCrc = MOTOR_UART_CalcCRC16(s_rxBuf, (uint16_t)(s_rxLen + 1));
    recvCrc = ((uint16_t)s_crcH << 8) | s_crcL;

    if (calcCrc != recvCrc) {
        /* CRC 校验失败，可在此添加错误计数或日志 */
        return;
    }

    /* 提取命令码与数据域长度 */
    cmdType = s_rxBuf[1];               /* TYPE 字段固定在 s_rxBuf[1] */
    dataLen = (s_rxLen > 0) ? (s_rxLen - 1) : 0; /* DATA 长度 = LEN - TYPE(1) */

    MOTOR_DispatchCmd(cmdType, &s_rxBuf[2], dataLen);
}

/* Public functions -----------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: APP_MotorUartParse_Init
*   功能说明: 初始化串口解析模块
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MotorUartParse_Init(void)
{
    APP_MotorUartParse_Reset();
}

/*
*********************************************************************************************************
*   函 数 名: APP_MotorUartParse_Reset
*   功能说明: 复位解析器状态机与缓存
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MotorUartParse_Reset(void)
{
    s_frameState = FRAME_STATE_IDLE;
    s_rxIndex    = 0;
    s_rxLen      = 0;
    s_crcL       = 0;
    s_crcH       = 0;
    s_telUpdated = 0;
    s_rxCallback = NULL;
    s_telCallback = NULL;
    memset(s_rxBuf, 0, sizeof(s_rxBuf));
    memset(&s_telemetry, 0, sizeof(s_telemetry));
}

/*
*********************************************************************************************************
*   函 数 名: APP_MotorUartParse_Process
*   功能说明: 串口单字节处理入口，建议在轮询或中断上下文中逐字节调用
*   形    参: chr  接收到的字节
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MotorUartParse_Process(uint8_t chr)
{
    switch (s_frameState) {
        case FRAME_STATE_IDLE:
            if (chr == MOTOR_UART_FRAME_HEAD) {
                s_frameState = FRAME_STATE_LEN;
            }
            break;

        case FRAME_STATE_LEN:
            s_rxBuf[0] = chr;
            s_rxLen    = chr;
            s_rxIndex  = 1;
            if (s_rxLen == 0 || s_rxLen > (MOTOR_UART_MAX_DATA_LEN + 1)) {
                /* LEN 非法或超长保护 */
                s_frameState = FRAME_STATE_IDLE;
            } else {
                s_frameState = FRAME_STATE_TYPE;
            }
            break;

        case FRAME_STATE_TYPE:
            s_rxBuf[s_rxIndex++] = chr;
            if (s_rxLen == 1) {
                /* 仅有 TYPE，无 DATA，直接跳转 CRC */
                s_frameState = FRAME_STATE_CRC_L;
            } else {
                s_frameState = FRAME_STATE_DATA;
            }
            break;

        case FRAME_STATE_DATA:
            s_rxBuf[s_rxIndex++] = chr;
            if (s_rxIndex >= (s_rxLen + 1)) {
                /* LEN + TYPE + DATA 接收完毕 */
                s_frameState = FRAME_STATE_CRC_L;
            }
            break;

        case FRAME_STATE_CRC_L:
            s_crcL = chr;
            s_frameState = FRAME_STATE_CRC_H;
            break;

        case FRAME_STATE_CRC_H:
            s_crcH = chr;
            s_frameState = FRAME_STATE_TAIL;
            break;

        case FRAME_STATE_TAIL:
            if (chr == MOTOR_UART_FRAME_TAIL) {
                MOTOR_FrameComplete();
            }
            /* 无论帧尾是否正确，都重置状态机准备接收下一帧 */
            s_frameState = FRAME_STATE_IDLE;
            break;

        default:
            s_frameState = FRAME_STATE_IDLE;
            break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_MotorUartParse_GetTelemetry
*   功能说明: 获取最新解析到的遥测数据指针
*   形    参: 无
*   返 回 值: 遥测数据结构体指针(只读，内部数据随新帧自动更新)
*********************************************************************************************************
*/
const MOTOR_Telemetry_t* APP_MotorUartParse_GetTelemetry(void)
{
    return &s_telemetry;
}

/*
*********************************************************************************************************
*   函 数 名: APP_MotorUartParse_SendCmd
*   功能说明: 按照协议格式组装并发送一帧数据
*   形    参: cmd   命令类型
*             data  数据域指针(无数据时可传 NULL)
*             len   数据域长度(字节)
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MotorUartParse_SendCmd(MOTOR_CMD_TYPE_E cmd, const uint8_t *data, uint8_t len)
{
    uint8_t  txBuf[MOTOR_UART_MAX_DATA_LEN + 6];
    uint16_t crc;
    uint8_t  idx = 0;
    uint8_t  payloadLen;

    if (len > MOTOR_UART_MAX_DATA_LEN) {
        return;
    }

    payloadLen = len + 1;   /* TYPE(1) + DATA(len) */

    txBuf[idx++] = MOTOR_UART_FRAME_HEAD;
    txBuf[idx++] = payloadLen;
    txBuf[idx++] = (uint8_t)cmd;

    if (len > 0 && data != NULL) {
        memcpy(&txBuf[idx], data, len);
        idx += len;
    }

    /* CRC 计算范围: LEN + TYPE + DATA，长度 = payloadLen + 1 */
    crc = MOTOR_UART_CalcCRC16(&txBuf[1], (uint16_t)(payloadLen + 1));
    txBuf[idx++] = (uint8_t)(crc & 0xFF);       /* CRC 低字节 */
    txBuf[idx++] = (uint8_t)((crc >> 8) & 0xFF);/* CRC 高字节 */
    txBuf[idx++] = MOTOR_UART_FRAME_TAIL;

    usart3_send(txBuf, idx);
}

/*
*********************************************************************************************************
*   函 数 名: APP_MotorUartParse_RegisterRxCallback
*   功能说明: 注册通用命令接收回调(高可扩展钩子)
*   形    参: cb  回调函数指针，NULL 表示注销
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MotorUartParse_RegisterRxCallback(MOTOR_RxCallback_t cb)
{
    s_rxCallback = cb;
}

/*
*********************************************************************************************************
*   函 数 名: APP_MotorUartParse_RegisterTelCallback
*   功能说明: 注册遥测数据更新回调(高可扩展钩子)
*   形    参: cb  回调函数指针，NULL 表示注销
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MotorUartParse_RegisterTelCallback(MOTOR_TelUpdateCallback_t cb)
{
    s_telCallback = cb;
}

/***************************** (END OF FILE) *********************************/
