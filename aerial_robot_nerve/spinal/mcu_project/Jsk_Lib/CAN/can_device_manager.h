/*
 * can_device_manager.h
 *
 *  Created on: 2016/10/26
 *      Author: anzai
 */

#ifndef APPLICATION_CAN_CAN_DEVICE_MANAGER_H_
#define APPLICATION_CAN_CAN_DEVICE_MANAGER_H_

#include "can_core.h"
#include "can_device.h"
#include "cmsis_os.h"

typedef struct
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
} can_msg;

namespace CANDeviceManager
{
	void init(CAN_HandleTypeDef* hcan, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	void addDevice(CANDevice& device);
  	void useRTOS(osMailQId* handle);
	void tick(int cycle /* ms */);
	bool connected();
  	void receiveMessage();
	void CAN_START();
}

#endif /* APPLICATION_CAN_CAN_DEVICE_MANAGER_H_ */