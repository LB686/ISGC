 /**
 * @brief    铁电存储器断开配置驱动头文件
 * @file     spi_fm25v10_config.h
 * @license  Copyright (c) 2018-2023 浙江华奕航空科技有限公司.All rights reserved.
 */
#ifndef __SPI_FM25V10_CONFIG_H
#define __SPI_FM25V10_CONFIG_H

#include "stm32h7xx_hal.h"
#include "spi.h"



extern const GPIO_TypeDef * fm25v10_cs_gpio_port;
extern const uint16_t fm25v10_cs_gpio_pin;
extern SPI_HandleTypeDef hspi4;

#endif
