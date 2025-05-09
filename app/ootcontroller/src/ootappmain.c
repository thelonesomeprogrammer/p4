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
#include "estimator_kalman.h"
#include "pm.h"

setpoint_t mySetpoint;
state_t myState;
int32_t compareResult = 0;

bool newState = false;

// static SemaphoreHandle_t dataMutex;
// static StaticSemaphore_t dataMutexBuffer;

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  // Wait for the system to be fully initialized
  systemWaitStart();

  struct PacketRX rxPacket;
  struct PacketTX txPacket;

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

  logVarId_t x = logGetVarId("stateEstimate", "x");
  logVarId_t y = logGetVarId("stateEstimate", "y");
  logVarId_t z = logGetVarId("stateEstimate", "z");

  logVarId_t roll = logGetVarId("stateEstimate", "roll");
  logVarId_t pitch = logGetVarId("stateEstimate", "pitch");
  logVarId_t yaw = logGetVarId("stateEstimate", "yaw");

  // logVarId_t thrust = logGetVarId("controller", "cmd_thrust");

  memset(&mySetpoint, 0, sizeof(setpoint_t));
  memset(&myState, 0, sizeof(state_t));

  mySetpoint.mode.x = modeAbs;
  mySetpoint.mode.y = modeAbs;
  mySetpoint.mode.z = modeAbs;
  mySetpoint.mode.yaw = modeAbs;

  myState.acc.x = 0;
  myState.acc.y = 0;
  myState.acc.z = 0;

  while (1) {
    if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
      if (rxPacket.messageType == false) {
        myState.position.x = rxPacket.v1.x;
        myState.position.y = rxPacket.v1.y;
        myState.position.z = rxPacket.v1.z;

        myState.velocity.x = rxPacket.v2.x;
        myState.velocity.y = rxPacket.v2.y;
        myState.velocity.z = rxPacket.v2.z;
      } 
      else if (rxPacket.messageType == true) {
        struct vec v = mkvec(rxPacket.v1.x, -rxPacket.v1.y, rxPacket.v1.z);
        struct quat q = rpy2quat(v);
        myState.attitudeQuaternion.x = q.x;
        myState.attitudeQuaternion.y = q.y;
        myState.attitudeQuaternion.z = q.z;
        myState.attitudeQuaternion.w = q.w;

        myState.attitude.roll = rxPacket.v1.x;
        myState.attitude.pitch = -rxPacket.v1.y;
        myState.attitude.yaw = rxPacket.v1.z;

        mySetpoint.position.x = rxPacket.v2.x;
        mySetpoint.position.y = rxPacket.v2.y;
        mySetpoint.position.z = rxPacket.v2.z;
      }
      commanderSetSetpoint(&mySetpoint, 3);
      newState = true;
    }

    txPacket.position.x = logGetFloat(x);
    txPacket.position.y = logGetFloat(y);
    txPacket.position.z = logGetFloat(z);
    txPacket.attitude.roll = logGetFloat(roll);
    txPacket.attitude.pitch = logGetFloat(pitch);
    txPacket.attitude.yaw = logGetFloat(yaw);

    txPacket.compareResult = compareResult;

    // txPacket.batteryVoltage = pmGetBatteryVoltage();

    // if (supervisorAreMotorsAllowedToRun()) {
    //   txPacket.debugState = true;
    // } 
    // else {
    //   txPacket.debugState = false;
    // }

    // Send data packet to PC
    appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));

    vTaskDelay(pdMS_TO_TICKS(50));
    // vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
  }
}

void estimatorOutOfTreeInit(void) {
  estimatorKalmanInit();
  // dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
}

bool estimatorOutOfTreeTest(void) { return true; }

void estimatorOutOfTree(state_t *state, const stabilizerStep_t stabilizerStep) {

  estimatorKalman(state, stabilizerStep);

  // xSemaphoreTake(dataMutex, portMAX_DELAY);

  if (newState == false) {
    return;
  }
  newState = false;
  // Directly update the state structure
  state->attitudeQuaternion.x = myState.attitudeQuaternion.x;
  state->attitudeQuaternion.y = myState.attitudeQuaternion.y;
  state->attitudeQuaternion.z = myState.attitudeQuaternion.z;
  state->attitudeQuaternion.w = myState.attitudeQuaternion.w;

  state->position.x = myState.position.x;
  state->position.y = myState.position.y;
  state->position.z = myState.position.z;

  state->velocity.x = myState.velocity.x;
  state->velocity.y = myState.velocity.y;
  state->velocity.z = myState.velocity.z;

  state->attitude.roll = myState.attitude.roll;
  state->attitude.pitch = myState.attitude.pitch;
  state->attitude.yaw = myState.attitude.yaw;
  compareResult = memcmp(&state, &myState, sizeof(state_t));
  // xSemaphoreGive(dataMutex);
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
