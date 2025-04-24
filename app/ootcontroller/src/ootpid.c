#include <stdbool.h>
#include <stdint.h>

#include "oot_msg_types.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "ootpid"
#include "debug.h"

#include "controller.h"
#include "controller_pid.h"

// void ootPidInit(ootpid_t *pid, float kp, float ki, float kd) {
//   pid->kp = kp;
//   pid->ki = ki;
//   pid->kd = kd;
//   pid->integral = 0.0f;
//   pid->last_error = 0.0f;
//   pid->output = 0.0f;
// }

void controllerOutOfTreeInit() {
  DEBUG_PRINT("ootpid: controllerOutOfTreeInit()\n");

  // controllerBrescianiniInit();
  controllerPidInit();

  // Initialize the PID controllers
  // ootPidInit(&ootpids[0], 1.0f, 0.1f, 0.01f);  // Roll rate
  // ootPidInit(&ootpids[1], 1.0f, 0.1f, 0.01f);  // Pitch rate
  // ootPidInit(&ootpids[2], 1.0f, 0.1f, 0.01f);  // Yaw rate
  // ootPidInit(&ootpids[3], 1.0f, 0.1f, 0.01f);  // Thrust rate
  // ootPidInit(&ootpids[4], 1.0f, 0.1f, 0.01f);  // Roll
  // ootPidInit(&ootpids[5], 1.0f, 0.1f, 0.01f);  // Pitch
  // ootPidInit(&ootpids[6], 1.0f, 0.1f, 0.01f);  // Yaw
  // ootPidInit(&ootpids[7], 1.0f, 0.1f, 0.01f);  // Thrust
  // ootPidInit(&ootpids[8], 1.0f, 0.1f, 0.01f);  // X velocity
  // ootPidInit(&ootpids[9], 1.0f, 0.1f, 0.01f);  // Y velocity
  // ootPidInit(&ootpids[10], 1.0f, 0.1f, 0.01f); // Z velocity
  // ootPidInit(&ootpids[11], 1.0f, 0.1f, 0.01f); // X position
  // ootPidInit(&ootpids[12], 1.0f, 0.1f, 0.01f); // Y position
  // ootPidInit(&ootpids[13], 1.0f, 0.1f, 0.01f); // Z position
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const stabilizerStep_t stabilizerStep) {
  controllerPid(control, setpoint, sensors, state, stabilizerStep);
}

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

bool controllerOutOfTreeTest() { return true; }

/*
############################################################## stabilizerStep_t
stabilizerStep_t = uint32_t
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
  attitude_t attitude;
        //(legacy CF2 body coordinate system, where pitch is inverted)
        quaternion_t attitudeQuaternion;
        point_t position;
        velocity_t velocity;      // m/s
        acc_t acc;                // Gs
        //(but acc.z without considering gravity)
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
