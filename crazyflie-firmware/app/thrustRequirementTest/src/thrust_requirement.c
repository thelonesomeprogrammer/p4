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
#include "pm.h"

#include "app_channel.h"


struct testPacketRX {
  	bool start;
} __attribute__((packed));

void appMain() {
	struct testPacketRX rxPacket;

	paramVarId_t idmoterena = paramGetVarId("motorPowerSet", "enable");
	paramSetInt(idmoterena, 1);
	paramVarId_t idm1 = paramGetVarId("motorPowerSet", "m1");
	paramVarId_t idm2 = paramGetVarId("motorPowerSet", "m2");
	paramVarId_t idm3 = paramGetVarId("motorPowerSet", "m3");
	paramVarId_t idm4 = paramGetVarId("motorPowerSet", "m4");

	uint16_t max = 65534;
	uint16_t procent = 50;
	uint16_t mode = 4;
	uint16_t thrust_value = (procent*max) / 100;

	// paramSetInt(idm1, thrust_value);
	// paramSetInt(idm2, thrust_value);
	// paramSetInt(idm3, thrust_value);
	// paramSetInt(idm4, thrust_value);
  	while(1) {
		if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), APPCHANNEL_WAIT_FOREVER)) {

			// if (rxPacket.start) {
			// 	thrust_value = (procent*max) / 100;
			// }

			if (mode == 0) {
				paramSetInt(idm1, thrust_value);
				paramSetInt(idm2, 0);
				paramSetInt(idm3, 0);
				paramSetInt(idm4, 0);
			} else if (mode == 1) {
				paramSetInt(idm1, 0);
				paramSetInt(idm2, thrust_value);
				paramSetInt(idm3, 0);
				paramSetInt(idm4, 0);
			} else if (mode == 2) {
				paramSetInt(idm1, 0);
				paramSetInt(idm2, 0);
				paramSetInt(idm3, thrust_value);
				paramSetInt(idm4, 0);
			} else if (mode == 3) {
				paramSetInt(idm1, 0);
				paramSetInt(idm2, 0);
				paramSetInt(idm3, 0);
				paramSetInt(idm4, thrust_value);
			} else if (mode == 4) {
				paramSetInt(idm1, thrust_value);
				paramSetInt(idm2, thrust_value);
				paramSetInt(idm3, thrust_value);
				paramSetInt(idm4, thrust_value);
			}

			appchannelSendDataPacket
    	}
  	}
}
