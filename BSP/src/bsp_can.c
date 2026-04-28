/**
 * @brief    BSP模块(For STM32F4)
 * @file     bsp.c
 * @details  这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 来包含所有的外设驱动模块。
 *			 bsp = Borad surport packet 板级支持包
 * @mainpage 
 * @author   LB
 * @email 
 * @version  V1.0
 * @date     2024/06/23
 * @license  Copyright (c) 2024-2024 浙江华奕航空科技有限公司.All rights reserved.
 */
#include "../../bsp/inc/bsp_can.h"

#include "can.h"


//#define CAN1_ENABLE HAL_GPIO_WritePin(ECAN1_GPIO_Port,ECAN1_Pin, GPIO_PIN_SET)
//#define CAN2_ENABLE HAL_GPIO_WritePin(ECAN2_GPIO_Port,ECAN2_Pin, GPIO_PIN_SET)

CAN_FilterTypeDef     sFilterConfig1;
CAN_FilterTypeDef     sFilterConfig2;

CAN_TxHeaderTypeDef   Can1TxHeader;
CAN_RxHeaderTypeDef   Can1RxHeader;

CAN_TxHeaderTypeDef   Can2TxHeader;
CAN_RxHeaderTypeDef   Can2RxHeader;
//函数声明
static void can1_config_filter(void);
static uint8_t can1_receive_msg(void);
//static void can2_config_filter(void);
//static uint8_t can2_receive_msg(void);
/**************************************************************
==> 功  能：CAN1过滤器配置函数
***************************************************************/
static void can1_config_filter(void) 
{
//	CAN_FilterTypeDef  sFilterConfig1;
	sFilterConfig1.FilterBank = 0;
	sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig1.FilterIdHigh = 0x0000;
	sFilterConfig1.FilterIdLow = 0x0000;
	sFilterConfig1.FilterMaskIdHigh = 0x0000;
	sFilterConfig1.FilterMaskIdLow = 0x0000;
	sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig1.FilterActivation = ENABLE;
	sFilterConfig1.SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1);
}
//static void can2_config_filter(void) 
//{
////	CAN_FilterTypeDef  sFilterConfig2;
//	sFilterConfig2.FilterBank = 14;
//	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
//	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig2.FilterIdHigh = 0x0000;
//	sFilterConfig2.FilterIdLow = 0x0000;
//	sFilterConfig2.FilterMaskIdHigh = 0x0000;
//	sFilterConfig2.FilterMaskIdLow = 0x0000;
//	sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
//	sFilterConfig2.FilterActivation = ENABLE;
//	sFilterConfig2.SlaveStartFilterBank = 14;
//	
//	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2);
//}
/**************************************************************
==> 功  能：重新激活CAN
***************************************************************/
void can_reactivate(CAN_HandleTypeDef *hcan) {
  HAL_CAN_Stop(hcan);  // 停止CAN模块
  HAL_CAN_Start(hcan); // 启动CAN模块
}
/**************************************************************
==> 功  能：CAN初始化配置
***************************************************************/
void can_configure(void)
{
	//CAN1使能
	//CAN1_ENABLE;
	can1_config_filter();
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	//CAN2使能
//	CAN2_ENABLE;
//	can2_config_filter();
	
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);	
//	HAL_CAN_Start(&hcan2);	
}
/**************************************************************
==> 功  能：CAN1发送函数
***************************************************************/
uint8_t can1_send_msg_to_fc(uint32_t id, uint8_t buf[8], uint8_t dlc) 
{	
	uint16_t can1_error = 0;
	CAN_TxHeaderTypeDef can1_header;
	can1_header.ExtId = id;
	can1_header.IDE = CAN_ID_EXT;
	can1_header.RTR = CAN_RTR_DATA;
	can1_header.DLC = dlc;
	uint32_t retry = 0;
	uint32_t tx_box = 0;
	while ((HAL_CAN_AddTxMessage(&hcan1, &can1_header, buf, &tx_box) != HAL_OK) && (retry < 0xfff)) 
	{
		retry++;
	}
	if (retry == 0xfff) 
	{
		can1_error++;
		can_reactivate(&hcan1);
	}
	return 1;
}
/**************************************************************
==> 功  能：CAN2发送函数
***************************************************************/
//uint8_t can2_send_msg_to_fc(uint32_t id, uint8_t buf[8], uint8_t dlc) 
//{	
//	uint16_t can2_error = 0;
//	CAN_TxHeaderTypeDef can2_header;
//	can2_header.ExtId = id;
//	can2_header.IDE = CAN_ID_EXT;
//	can2_header.RTR = CAN_RTR_DATA;
//	can2_header.DLC = dlc;
//	uint32_t retry = 0;
//	uint32_t tx_box = 0;
//	while ((HAL_CAN_AddTxMessage(&hcan2, &can2_header, buf, &tx_box) != HAL_OK) && (retry < 0xfff)) 
//	{
//		retry++;
//	}
//	if (retry == 0xfff) 
//	{
//		can2_error++;
//		can_reactivate(&hcan2);
//	}
//	return 1;
//}
/**************************************************************
==> 功  能：CAN1接收函数
***************************************************************/
static uint8_t can1_receive_msg(void){
	uint8_t buf[8];
	uint8_t res = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can1RxHeader, buf);	
	if(res != HAL_OK) {
		can_reactivate(&hcan1);
		return 0;
	}
	if (Can1RxHeader.IDE == CAN_ID_EXT) {
		//can_rec_data(Can1RxHeader.ExtId);    //超时时间处理
		
		can1_send_msg_to_fc(Can1RxHeader.ExtId,buf,8);
	} else {
		
	}
	return 1;
}
/**************************************************************
==> 功  能：CAN2接收函数
***************************************************************/
//static uint8_t can2_receive_msg(void){
//	uint8_t buf[8];
//	uint8_t res = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Can2RxHeader, buf);	
//	if(res != HAL_OK) {
//		can_reactivate(&hcan2);
//		return 0;
//	}
//	if (Can2RxHeader.IDE == CAN_ID_EXT) {
//		can_rec_data(Can2RxHeader.ExtId);
//	} else {
//		
//	}
//	return 1;
//}

/**************************************************************
==> 功  能：中断处理回调函数
***************************************************************/
void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(hcan->Instance == CAN1)
	{
		can1_receive_msg();		
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
	}else if(hcan->Instance == CAN2)
	{
//		can2_receive_msg();
//		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	}else{	
		
	}
}
