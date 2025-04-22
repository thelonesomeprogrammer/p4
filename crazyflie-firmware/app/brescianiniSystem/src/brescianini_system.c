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
#include "supervisor.h"
#include "system.h"

#include "pm.h"
#include "estimator_kalman.h"
#include "sensors_bmi088_bmp3xx.h"
#include "sensors.h"
#include "commander.h"

struct PacketRX {
	struct{float x, y, z;}position;
	struct{float x, y, z;}velocity;
	// struct{float x, y, z;}acceleration; // Removed acceleration to reduce data size
	float yaw;
	// float yawRate; // Removed yawRate to reduce data size
} __attribute__((packed));

struct PacketTX {
	struct{float x, y, z;}position;
	float batteryVoltage;
	bool debugState;
} __attribute__((packed));

void appMain() {
	// Wait for the system to be fully initialized
	systemWaitStart();

	struct PacketRX rxPacket;
	struct PacketTX txPacket;

	if (!supervisorRequestArming(true)) {
		txPacket.debugState = false;
	} else {
		txPacket.debugState = true;
	}

	setpoint_t mySetpoint;

    // memset(&mySetpoint, 0, sizeof(setpoint_t));

  	while(1) {
		if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), APPCHANNEL_WAIT_FOREVER)) {
			
			// Set all desired values used in controller
			mySetpoint.position.x = rxPacket.position.x;
			mySetpoint.position.y = rxPacket.position.y;
			mySetpoint.position.z = rxPacket.position.z;

			mySetpoint.velocity.x = rxPacket.velocity.x;
			mySetpoint.velocity.y = rxPacket.velocity.y;
			mySetpoint.velocity.z = rxPacket.velocity.z;

			mySetpoint.acceleration.x = 0.1;
			mySetpoint.acceleration.y = 0.1;
			mySetpoint.acceleration.z = 0.1;

			mySetpoint.attitude.yaw = rxPacket.yaw;
			mySetpoint.attitudeRate.yaw = 0;

			commanderSetSetpoint(&mySetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
				
			point_t pos;
			estimatorKalmanGetEstimatedPos(&pos);
			txPacket.position.x = pos.x;
			txPacket.position.y = pos.y;
			txPacket.position.z = pos.z;
			txPacket.batteryVoltage = pmGetBatteryVoltage();

			// Send data packet to PC
			appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
    	}
		vTaskDelay(pdMS_TO_TICKS(10));
  	}
}