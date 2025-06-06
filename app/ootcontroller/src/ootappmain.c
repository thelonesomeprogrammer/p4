#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "oot_msg_types.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "app_channel.h"
#include "log.h"
#include "param.h"
#include "supervisor.h"
#include "system.h"

#define DEBUG_MODULE "ootpid"
#include "debug.h"
#include "math3d.h"

#include "commander.h"
#include "pm.h"

setpoint_t mySetpoint;
state_t myState;
int32_t compareResult = 0;

// extern void resetAllPid();

bool newState = false;

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  // Wait for the system to be fully initialized
  systemWaitStart();

  struct PacketRX rxPacket;
  // struct PacketTX txPacket;

  TickType_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

  // logVarId_t x = logGetVarId("kalman", "stateX");
  // logVarId_t y = logGetVarId("kalman", "stateY");
  // logVarId_t z = logGetVarId("kalman", "stateZ");

  // logVarId_t x = logGetVarId("ctrltarget", "x");
  // logVarId_t y = logGetVarId("ctrltarget", "y");
  // logVarId_t z = logGetVarId("ctrltarget", "z");

  // logVarId_t x = logGetVarId("controller", "roll");
  // logVarId_t y = logGetVarId("controller", "pitch");
  // logVarId_t z = logGetVarId("controller", "yaw");

  // logVarId_t x = logGetVarId("stateEstimate", "x");
  // logVarId_t y = logGetVarId("stateEstimate", "y");
  // logVarId_t z = logGetVarId("stateEstimate", "z");
  //
  // logVarId_t roll = logGetVarId("stateEstimate", "roll");
  // logVarId_t pitch = logGetVarId("stateEstimate", "pitch");
  // logVarId_t yaw = logGetVarId("stateEstimate", "yaw");

  // logVarId_t thrust = logGetVarId("controller", "cmd_thrust");

  memset(&mySetpoint, 0, sizeof(setpoint_t));

  mySetpoint.mode.x = modeAbs;
  mySetpoint.mode.y = modeAbs;
  mySetpoint.mode.z = modeAbs;
  mySetpoint.mode.yaw = modeAbs;

  while (1) {
    if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
      mySetpoint.position.x = rxPacket.pos.x;
      mySetpoint.position.y = rxPacket.pos.y;
      mySetpoint.position.z = rxPacket.pos.z;
      if (rxPacket.pos.z == 0 && rxPacket.pos.x == 0 && rxPacket.pos.y == 0) {
        memset(&mySetpoint, 0, sizeof(setpoint_t));

        mySetpoint.mode.x = modeAbs;
        mySetpoint.mode.y = modeAbs;
        mySetpoint.mode.z = modeAbs;
        mySetpoint.mode.yaw = modeAbs;
      }

      // resetAllPid();

      commanderSetSetpoint(&mySetpoint, 3);
    }

    // txPacket.position.x = logGetFloat(x);
    // txPacket.position.y = logGetFloat(y);
    // txPacket.position.z = logGetFloat(z);
    // txPacket.attitude.roll = logGetFloat(roll);
    // txPacket.attitude.pitch = logGetFloat(pitch);
    // txPacket.attitude.yaw = logGetFloat(yaw);

    // txPacket.batteryVoltage = pmGetBatteryVoltage();

    // if (supervisorAreMotorsAllowedToRun()) {
    //   txPacket.debugState = true;
    // }
    // else {
    //   txPacket.debugState = false;
    // }

    // Send data packet to PC
    // appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
  }
}
