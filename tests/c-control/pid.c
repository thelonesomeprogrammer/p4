#include "cf_types.h"

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float output;
} ootpid_t;

ootpid_t ootpids[7];
float error[3];

void ootPidInit(ootpid_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->output = 0.0f;
}

void controllerOutOfTreeInit() {
  // Initialize the PID controllers
  ootPidInit(&ootpids[0], 0.65f, 0.15f, 0.51f);    // x
  ootPidInit(&ootpids[1], -0.65f, -0.15f, -0.51f); // y
  ootPidInit(&ootpids[2], 0.24f, 0.084f, 0.17f);   // z
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

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const stabilizerStep_t stabilizerStep) {

  control->controlMode = controlModeForceTorque;

  if (RATE_DO_EXECUTE(100, stabilizerStep)) {
    // error
    error[0] = setpoint->position.x - state->position.x;
    error[1] = setpoint->position.y - state->position.y;
    error[2] = setpoint->position.z - state->position.z;

    // dt
    float dt = 0.01f; // Assuming a fixed time step for simplicity

    // Update the PID controllers
    ootpidstep(&ootpids[0], error[0], dt); // x
    ootpidstep(&ootpids[1], error[1], dt); // y
    ootpidstep(&ootpids[2], error[2], dt); // z
  }
}
