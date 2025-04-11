/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "param.h"
#include "log.h"
#include "app_channel.h"

#include "pm.h"
#include "estimator_kalman.h"
#include "sensors_bmi088_bmp3xx.h"
#include "sensors.h"

struct PacketRX {
  	bool start;
	bool sample;
} __attribute__((packed));

struct PacketTX {
	float batteryVoltage;
	float pressure_max;
	float pressure_min;
	float pressure;
	uint32_t procent;
} __attribute__((packed));

void appMain() {
	struct PacketRX rxPacket;
	struct PacketTX txPacket;

	paramVarId_t idmoterena = paramGetVarId("motorPowerSet", "enable");
	paramSetInt(idmoterena, 1);
	paramVarId_t idm1 = paramGetVarId("motorPowerSet", "m1");
	paramVarId_t idm2 = paramGetVarId("motorPowerSet", "m2");
	paramVarId_t idm3 = paramGetVarId("motorPowerSet", "m3");
	paramVarId_t idm4 = paramGetVarId("motorPowerSet", "m4");

	logVarId_t pressure = logGetVarId("baro", "pressure");
	// txPacket.pressure_max = logGetFloat(pressure);
	// txPacket.pressure_min = logGetFloat(pressure);
	
	uint16_t max = 65534;
	txPacket.procent = 0;
	uint16_t thrust_value = (txPacket.procent*max) / 100;

  	while(1) {
		if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), APPCHANNEL_WAIT_FOREVER)) {
			
			// Get battery voltage and pressure
			txPacket.batteryVoltage = pmGetBatteryVoltage();
			txPacket.pressure = logGetFloat(pressure);

			if (rxPacket.sample){
				if (txPacket.pressure_max < 100.0f) {
					txPacket.pressure_max = txPacket.pressure;
				}
				if (txPacket.pressure_min < 100.0f) {
					txPacket.pressure_min = txPacket.pressure;
				}
	
				// Set asl bound
				for (int i = 0; i < 1000; i++){
					if (txPacket.pressure > txPacket.pressure_max){
						txPacket.pressure_max = txPacket.pressure;
					}
					else if (txPacket.pressure < txPacket.pressure_min){
						txPacket.pressure_min = txPacket.pressure;
					}
				}
			}

			if (rxPacket.start){
				while (true)
				{
					vTaskDelay(1000);
					txPacket.pressure = logGetFloat(pressure);
					if (txPacket.pressure > txPacket.pressure_min){
						txPacket.procent += 1;
					}
					else {
						txPacket.procent = 0;
					}
					thrust_value = (txPacket.procent*max) / 100;
	
					paramSetInt(idm1, thrust_value);
					paramSetInt(idm2, thrust_value);
					paramSetInt(idm3, thrust_value);
					paramSetInt(idm4, thrust_value);
					appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
				}
			}
			
			// Send data packet to PC
			appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
			
    	}
  	}
}