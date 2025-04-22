#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "param.h"
#include "log.h"
#include "app_channel.h"
#include "supervisor.h"
#include "system.h"


#define DEBUG_MODULE "ootpid"
#include "debug.h"

#include "pm.h"
#include "estimator_kalman.h"
#include "commander.h"
#include "controller.h"
#include "controller_brescianini.h"
#include "controller_pid.h"

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
  float cmd_thrust;
	bool debugState;
} __attribute__((packed));

setpoint_t mySetpoint;
int8_t test = 0;

// We need an appMain() function, but we will not really use it. Just let it
// quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  // Wait for the system to be fully initialized
	systemWaitStart();

  struct PacketRX rxPacket;
	struct PacketTX txPacket;

  logVarId_t x = logGetVarId("kalman", "stateX");
  logVarId_t y = logGetVarId("kalman", "stateY");
  logVarId_t z = logGetVarId("kalman", "stateZ");

  // logVarId_t x = logGetVarId("ctrltarget", "x");
  // logVarId_t y = logGetVarId("ctrltarget", "y");
  // logVarId_t z = logGetVarId("ctrltarget", "z");
  logVarId_t thrust = logGetVarId("controller", "cmd_thrust");
  
  memset(&mySetpoint, 0, sizeof(setpoint_t));

  while (1) {
    test += 1;
    if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
			// Set all desired values used in controller
      // mySetpoint.thrust = 20.0f;

			mySetpoint.position.x = rxPacket.position.x;
			mySetpoint.position.y = rxPacket.position.y;
			mySetpoint.position.z = rxPacket.position.z;

			mySetpoint.velocity.x = rxPacket.velocity.x;
			mySetpoint.velocity.y = rxPacket.velocity.y;
			mySetpoint.velocity.z = rxPacket.velocity.z;

			// mySetpoint.acceleration.x = 0.1;
			// mySetpoint.acceleration.y = 0.1;
			// mySetpoint.acceleration.z = 0.1;

			mySetpoint.attitude.yaw = rxPacket.yaw;
			// mySetpoint.attitudeRate.yaw = 0;

      mySetpoint.mode.x = modeAbs;
      mySetpoint.mode.y = modeAbs;
      mySetpoint.mode.z = modeAbs;
      // mySetpoint.mode.roll = modeAbs;
      // mySetpoint.mode.pitch = modeAbs;
      mySetpoint.mode.yaw = modeAbs;

			commanderSetSetpoint(&mySetpoint, 3);
    }
    txPacket.batteryVoltage = pmGetBatteryVoltage();

    txPacket.position.x = logGetFloat(x);
    txPacket.position.y = logGetFloat(y);
    txPacket.position.z = logGetFloat(z);
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


void ootPidInit(ootpid_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->output = 0.0f;
}

void controllerOutOfTreeInit() {
  DEBUG_PRINT("ootpid: controllerOutOfTreeInit()\n");

  controllerBrescianiniInit();
  // controllerPidInit();

  // Initialize the PID controllers
  ootPidInit(&ootpids[0], 1.0f, 0.1f, 0.01f);  // Roll rate
  ootPidInit(&ootpids[1], 1.0f, 0.1f, 0.01f);  // Pitch rate
  ootPidInit(&ootpids[2], 1.0f, 0.1f, 0.01f);  // Yaw rate
  ootPidInit(&ootpids[3], 1.0f, 0.1f, 0.01f);  // Thrust rate
  ootPidInit(&ootpids[4], 1.0f, 0.1f, 0.01f);  // Roll
  ootPidInit(&ootpids[5], 1.0f, 0.1f, 0.01f);  // Pitch
  ootPidInit(&ootpids[6], 1.0f, 0.1f, 0.01f);  // Yaw
  ootPidInit(&ootpids[7], 1.0f, 0.1f, 0.01f);  // Thrust
  ootPidInit(&ootpids[8], 1.0f, 0.1f, 0.01f);  // X velocity
  ootPidInit(&ootpids[9], 1.0f, 0.1f, 0.01f);  // Y velocity
  ootPidInit(&ootpids[10], 1.0f, 0.1f, 0.01f); // Z velocity
  ootPidInit(&ootpids[11], 1.0f, 0.1f, 0.01f); // X position
  ootPidInit(&ootpids[12], 1.0f, 0.1f, 0.01f); // Y position
  ootPidInit(&ootpids[13], 1.0f, 0.1f, 0.01f); // Z position
}


void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
  const sensorData_t *sensors, const state_t *state,
  const stabilizerStep_t stabilizerStep)
{
  // setpoint_t setpointCopy = *setpoint;

  // setpointCopy.position.x = mySetpoint.position.x;
  // setpointCopy.position.y = mySetpoint.position.y;
  // setpointCopy.position.z = mySetpoint.position.z;

  // setpointCopy.velocity.x = mySetpoint.velocity.x;
  // setpointCopy.velocity.y = mySetpoint.velocity.y;
  // setpointCopy.velocity.z = mySetpoint.velocity.z;

  // setpointCopy.attitude.yaw = mySetpoint.attitude.yaw;

  // const setpoint_t *setpointPtr = &setpointCopy;

  // // commanderSetSetpoint(setpointPtr, 3);

  controllerBrescianini(control, setpoint, sensors, state, stabilizerStep);
  // controllerPid(control, setpoint, sensors, state, stabilizerStep);
}
bool controllerOutOfTreeTest() { return true; }

/*
#############################################################control_t
ypedef enum control_mode_e {
  controlModeLegacy      = 0,
        controlModeForceTorque = 1,
        controlModeForce       = 2,
        }control_mode_t;


typedef struct control_s {
  union {
    // controlModeLegacy
    struct {
      int16_t roll;
      int16_t pitch;
      int16_t yaw;
      float thrust;
    };

    // controlModeForceTorque
    // Note: Using SI units for a controller makes it hard to tune it for
different platforms. The normalized force API
    // is probably a better option.
    struct {
      float thrustSi;  // N
      union { // Nm
        float torque[3];
        struct {
          float torqueX;
          float torqueY;
          float torqueZ;
        };
      };
    };

    // controlModeForce
    float normalizedForces[STABILIZER_NR_OF_MOTORS]; // 0.0 ... 1.0
  };

  control_mode_t controlMode;
} control_t;
##############################################################
##############################################################sensorData_t

typedef struct sensorData_s {
  Axis3f acc;               // Gs
  Axis3f gyro;              // deg/s
  Axis3f mag;               // gauss
  baro_t baro;
#ifdef LOG_SEC_IMU
  Axis3f accSec;            // Gs
  Axis3f gyroSec;           // deg/s
#endif
  uint64_t interruptTimestamp;
} sensorData_t;

###############################################################
###############################################################state_t
typedef struct state_s {
  attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted) 
  quaternion_t attitudeQuaternion; 
  point_t position;
  velocity_t velocity;      // m/s
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

// void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
//                          const sensorData_t *sensors, const state_t *state,
//                          const stabilizerStep_t stabilizerStep)
// {
  
//   control->controlMode = controlModeLegacy;
//   // Calculate the error
// }

// void ootpidstep(ootpid_t *ootpid, float error, float dt) {
//   // Calculate the integral
//   ootpid->integral += error * dt;

//   // Calculate the derivative
//   float derivative = (error - ootpid->last_error) / dt;

//   // Calculate the and set the output
//   ootpid->output = ootpid->kp * error + ootpid->ki * ootpid->integral +
//                    ootpid->kd * derivative;

//   // Save the last error
//   ootpid->last_error = error;
// }

LOG_GROUP_START(ootpid)
LOG_ADD(LOG_UINT8, test, &test)
LOG_GROUP_STOP(ootpid)
