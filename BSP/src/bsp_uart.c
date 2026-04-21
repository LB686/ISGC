/*
*********************************************************************************************************
*
*	模块名称 : 串口+FIFO驱动模块
*	文件名称 : bsp_uart.c
*	版    本 : V1.0
*	说    明 : 
*	修改记录 :
*		版本号  日期         作者      说明
*		V1.0    2025-03-19   libing   驱动创建
*
*
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "../../bsp/inc/bsp_uart.h"

#include "usart.h"

#include "../../bsp/bsp.h"
#include "../../app/app_motor_uart_parse.h"

/* Private define-------------------------------------------------------------*/



/* Private variables----------------------------------------------------------*/
static void init_usart3_tx_fifo(void);                 //数据发送缓冲区初始化函数

static void read_usart3_state(void);                   //读取串口3发送缓存区状态

static void init_usart3_rx_fifo(void);                  //数据缓冲区初始化函数

static void usart3_got_byte(uint8_t b);                //获取数据字节

static void usart3_loop(void);                         //串口3数据周期处理函数

/* Public variables-----------------------------------------------------------*/

#if UART3_EN == 1
    uint8_t  usart3_data;                               //串口2数据接收字节
    uint8_t  usart3_rx_buf[USART3_RX_FIFO_SIZE];        //串口2接收数组
    struct   ring_buffer usart3_rx_buffer;              //数据接收fifo
    
    uint8_t  usart3_dma_tx_buf[USART3_TX_FIFO_SIZE];    //__attribute__((at(0x24004800)));    //DMA发送buffer    
    uint8_t  usart3_tx_buf[USART3_TX_FIFO_SIZE];        //发送fifo内部buffer
    struct   ring_buffer usart3_tx_buffer;              //数据发送fifo
#endif

/* Private function prototypes------------------------------------------------*/     

/*
*********************************************************************************************************
*	函 数 名: init_usart3_tx_fifo
*	功能说明: 清空串口发送缓存区及初始化发送FIFO
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void init_usart3_tx_fifo(void) {
  memset(usart3_tx_buf, 0 ,USART3_TX_FIFO_SIZE);
  memset(usart3_dma_tx_buf, 0, USART3_TX_FIFO_SIZE);
  fifo_init(&usart3_tx_buffer, usart3_tx_buf, USART3_TX_FIFO_SIZE);
}

/*
*********************************************************************************************************
*	函 数 名: usart3_send
*	功能说明: 清空串口DMA发送缓存区及初始化发送FIFO
*	形    参: *buff       待发送数据指针
*             len         待发送数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void usart3_send(const uint8_t *buff, uint16_t len) {
  if ((huart3.gState == HAL_UART_STATE_READY) && fifo_is_empty(&usart3_tx_buffer)) {
    memcpy(usart3_dma_tx_buf, buff, len);
      
    #if USART3_TX_DMA_EN == 1
    HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&huart3, usart3_dma_tx_buf, len);
    #else
    HAL_StatusTypeDef res = HAL_UART_Transmit_IT(&huart3, usart3_dma_tx_buf, len);  
    #endif  
      
    SET_BIT(huart3.Instance->CR1, USART_CR1_TCIE);
    } 
  else{
    fifo_push(&usart3_tx_buffer, buff, len);
  }
}


/*
*********************************************************************************************************
*	函 数 名: read_usart_state
*	功能说明: 读取各个串口忙状态，如果有数据未发送完成，则重新发送
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void read_usart_state(void){
    read_usart3_state();
    
    
}

static void read_usart3_state(void){
  if (!fifo_is_empty(&usart3_tx_buffer)) {
    if (huart3.gState == HAL_UART_STATE_READY) {
      uint32_t len = fifo_take_all(&usart3_tx_buffer, usart3_dma_tx_buf);
      #if USART3_TX_DMA_EN == 1
        HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&huart3, usart3_dma_tx_buf, len);
      #else
        HAL_StatusTypeDef res = HAL_UART_Transmit_IT(&huart3, usart3_dma_tx_buf, len);  
      #endif   
      SET_BIT(huart3.Instance->CR1, USART_CR1_TCIE);      
      HAL_UART_Receive_IT(&huart3, &usart3_data, 1); 
    }
  }
}

/*
*********************************************************************************************************
*	函 数 名: init_usart3_rx_fifo
*	功能说明: 串口接收FIFO初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void init_usart3_rx_fifo(void) {
	memset(usart3_rx_buf, 0, USART3_RX_FIFO_SIZE);
	fifo_init(&usart3_rx_buffer, usart3_rx_buf, USART3_RX_FIFO_SIZE);

}

/*
*********************************************************************************************************
*	函 数 名: usart3_got_byte
*	功能说明: 串口1接收1字节，在串口接收中断中调用
*	形    参: b       接收到的字节数据
*	返 回 值: 无
*********************************************************************************************************
*/
static void usart3_got_byte(uint8_t b) {
	 if (!fifo_push_byte(&usart3_rx_buffer, b)){
		 
	 }		
}

/*
*********************************************************************************************************
*	函 数 名: usart_dataparse
*	功能说明: 串口数据解析
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void usart_dataparse(void){
    usart3_loop();
}
static void usart3_loop(void){
	uint8_t chr;
	while(!fifo_is_empty(&usart3_rx_buffer)) {
		if (fifo_pop_byte(&usart3_rx_buffer, &chr)) {
			usart3_frame_assemble(chr);    //数据解析
		}
	}
}

/**************************************************************
==> 功  能：串口配置
***************************************************************/
void usart_configure(void){
    HAL_UART_Receive_IT(&huart3, &usart3_data, 1);    //开启串口1接收中断   
   
	  init_usart3_rx_fifo();

    init_usart3_tx_fifo();   
    
}


/***************************************************************
==> 功  能：UART回调函数
***************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	UNUSED(huart);
    if(huart == &huart3){            
        usart3_got_byte(usart3_data);
		HAL_UART_Receive_IT(&huart3,(uint8_t *)&usart3_data,1);    
    
    }else{
    
    }
}  
    
/***************************************************************
==> 功  能：UART错误处理，出现UART错误时自动处理，无需调用
***************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	// 清除所有错误标志
	__HAL_UART_CLEAR_PEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_OREFLAG(huart);
	// 重新初始化 UART
	if (HAL_UART_DeInit(huart) != HAL_OK) {
        Error_Handler(); // 如果串口失能失败，调用错误处理函数
	}
	if (HAL_UART_Init(huart) != HAL_OK) {
		Error_Handler(); // 如果初始化失败，调用错误处理函数
	}
    if(huart->Instance == USART3){
		if (HAL_UART_Receive_IT(&huart3, &usart3_data, 1) != HAL_OK) {
        Error_Handler(); 
    }
	}         
}


/***************************** (END OF FILE) *********************************/


