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

#include "commander.h"
#include "pm.h"

setpoint_t mySetpoint;
state_t myState;

static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  // Wait for the system to be fully initialized
  systemWaitStart();

  struct PacketRX rxPacket;
  struct PacketTX txPacket;

  // logVarId_t x = logGetVarId("kalman", "stateX");
  // logVarId_t y = logGetVarId("kalman", "stateY");
  // logVarId_t z = logGetVarId("kalman", "stateZ");

  // logVarId_t x = logGetVarId("ctrltarget", "x");
  // logVarId_t y = logGetVarId("ctrltarget", "y");
  // logVarId_t z = logGetVarId("ctrltarget", "z");

  logVarId_t x = logGetVarId("stateEstimate", "x");
  logVarId_t y = logGetVarId("stateEstimate", "y");
  logVarId_t z = logGetVarId("stateEstimate", "z");

  logVarId_t thrust = logGetVarId("controller", "cmd_thrust");

  memset(&mySetpoint, 0, sizeof(setpoint_t));

  while (1) {
    if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
      // Set all desired values used in controller

      if (rxPacket.messageType == false) {
        myState.position.x = rxPacket.v1.x;
        myState.position.y = rxPacket.v1.y;
        myState.position.z = rxPacket.v1.z;

        myState.velocity.x = rxPacket.v2.x;
        myState.velocity.y = rxPacket.v2.y;
        myState.velocity.z = rxPacket.v2.z;
      } else if (rxPacket.messageType == true) {
        myState.attitude.roll = rxPacket.v1.x;
        myState.attitude.pitch = -rxPacket.v1.y;
        myState.attitude.yaw = rxPacket.v1.z;

        mySetpoint.position.x = rxPacket.v2.x;
        mySetpoint.position.y = rxPacket.v2.y;
        mySetpoint.position.z = rxPacket.v2.z;
      }

      mySetpoint.mode.x = modeAbs;
      mySetpoint.mode.y = modeAbs;
      mySetpoint.mode.z = modeAbs;
      mySetpoint.mode.yaw = modeAbs;

      commanderSetSetpoint(&mySetpoint, 3);
    }
    txPacket.batteryVoltage = pmGetBatteryVoltage();

    txPacket.position.x = logGetFloat(x);
    txPacket.position.y = logGetFloat(y);
    txPacket.position.z = logGetFloat(z);
    // txPacket.position.x = mySetpoint.position.x;
    // txPacket.position.y = mySetpoint.position.y;
    // txPacket.position.z = mySetpoint.position.z;
    txPacket.cmd_thrust = logGetFloat(thrust);

    if (supervisorAreMotorsAllowedToRun()) {
      txPacket.debugState = true;
    } else {
      txPacket.debugState = false;
    }

    // Send data packet to PC
    appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void estimatorOutOfTreeInit(void) {

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
}

bool estimatorOutOfTreeTest(void) { return true; }

void estimatorOutOfTree(state_t *state, const stabilizerStep_t stabilizerStep) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &myState, sizeof(state_t));
  xSemaphoreGive(dataMutex);
}

/*
############################################################## stabilizerStep_t
stabilizerStep_t = uint32_t
###############################################################state_t
typedef struct state_s {
  attitude_t attitude;      // deg (legacy CF2 body coordinate system, where
pitch is inverted) quaternion_t Quaternion; point_t position;
velocity_tvelocity;      // m/s
acc_t acc;                // Gs (but acc.z without considering gravity)
} state_t;
###############################################################
###############################################################setpoint_t
typedef struct setpoint_s {
  uint32_t timestamp;

  attitude_t attitude;      // deg
  attitude_t attitudeRate;  // deg/s
  quaternion_t attitudeQuaternion;
  float thrust;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acceleration;       // m/s^2
  jerk_t jerk;              // m/s^3
  bool velocity_body;       // true if velocity is given in body frame; false if
velocity is given in world frame

  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
} setpoint_t;
###############################################################
*/
