#ifndef __CAN_DRIVE_H
#define __CAN_DRIVE_H

#include "can.h"

void can_configure(void);

uint8_t can1_send_msg_to_fc(uint32_t id, uint8_t buf[8], uint8_t dlc); 
uint8_t can2_send_msg_to_fc(uint32_t id, uint8_t buf[8], uint8_t dlc); 

#endif
