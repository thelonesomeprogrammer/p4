#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "param.h"

#define DEBUG_MODULE "ootpid"
#include "debug.h"

#include "controller.h"

// We need an appMain() function, but we will not really use it. Just let it
// quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1) {
    vTaskDelay(M2T(2000));
  }
}

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float output;
} ootpid_t;

ootpid_t ootpids[7];

void pidInit(ootpid_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->output = 0.0f;
}

void controllerOutOfTreeInit() {
  DEBUG_PRINT("ootpid: controllerOutOfTreeInit()\n");
  // Initialize the PID controllers
  pidInit(&ootpids[0], 1.0f, 0.1f, 0.01f);  // Roll rate
  pidInit(&ootpids[1], 1.0f, 0.1f, 0.01f);  // Pitch rate
  pidInit(&ootpids[2], 1.0f, 0.1f, 0.01f);  // Yaw rate
  pidInit(&ootpids[3], 1.0f, 0.1f, 0.01f);  // Thrust rate
  pidInit(&ootpids[4], 1.0f, 0.1f, 0.01f);  // Roll
  pidInit(&ootpids[5], 1.0f, 0.1f, 0.01f);  // Pitch
  pidInit(&ootpids[6], 1.0f, 0.1f, 0.01f);  // Yaw
  pidInit(&ootpids[7], 1.0f, 0.1f, 0.01f);  // Thrust
  pidInit(&ootpids[8], 1.0f, 0.1f, 0.01f);  // X velocity
  pidInit(&ootpids[9], 1.0f, 0.1f, 0.01f);  // Y velocity
  pidInit(&ootpids[10], 1.0f, 0.1f, 0.01f); // Z velocity
  pidInit(&ootpids[11], 1.0f, 0.1f, 0.01f); // X position
  pidInit(&ootpids[12], 1.0f, 0.1f, 0.01f); // Y position
  pidInit(&ootpids[13], 1.0f, 0.1f, 0.01f); // Z position
}

bool controllerOutOfTreeTest() { return true; }

/*
#############################################################control_t
ypedef enum control_mode_e {
  controlModeLegacy      = 0, // legacy mode with int16_t roll, pitch, yaw and
float thrust controlModeForceTorque = 1, controlModeForce       = 2, }
control_mode_t;

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
  attitude_t attitude;      // deg (legacy CF2 body coordinate system, where
pitch is inverted) quaternion_t attitudeQuaternion; point_t position;         //
m velocity_t velocity;      // m/s acc_t acc;                // Gs (but acc.z
without considering gravity) } state_t;
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

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const uint32_t tick) {

  control->controlMode = controlModeLegacy;

  // Calculate the error
}

void ootpidstep(ootpid_t *ootpid, float error, float dt) {
  // Calculate the integral
  ootpid->integral += error * dt;

  // Calculate the derivative
  float derivative = (error - ootpid->last_error) / dt;

  // Calculate the and set the output
  ootpid->output = ootpid->kp * error + ootpid->ki * ootpid->integral +
                   ootpid->kd * derivative;

  // Save the last error
  ootpid->last_error = error;
}
