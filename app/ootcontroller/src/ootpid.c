#include "stabilizer_types.h"
#include "controller_brescianini.h"
#include <math.h>
#include <stdio.h>

typedef struct {
  float z;
  float p;
  float k;
  float output;
  float lastOutput;
  float lastInput;
} lead_t;

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float output;
} ootpid_t;

ootpid_t ootpids[3];
lead_t lead[2];
float error[5];

void ootPidInit(ootpid_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->output = 0.0f;
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
void leadinit(lead_t *lead, float z, float p, float k) {
  lead->z = z;
  lead->p = p;
  lead->k = k;
  lead->lastInput = 0.0f;
  lead->lastOutput = 0.0f;
  lead->output = 0.0f;
}

void leadupdate(lead_t *lead, float input, float dt) {
  // Update the lead filter state (K*(u[k]*(1+z*T)-u[k-1])+y[k-1])/(1+p*T)
  lead->output = (lead->k * (input * (1 + lead->z * dt) - lead->lastInput) +
                  lead->lastOutput) /
                 (1 + lead->p * dt);
}

void controllerOutOfTreeInit() {
  // Initialize the PID controllers
  ootPidInit(&ootpids[0], 0.65f, 0.15f, 0.51f);    // x
  ootPidInit(&ootpids[1], -0.65f, -0.15f, -0.51f); // y
  ootPidInit(&ootpids[2], 0.24f, 0.084f, 0.17f);   // z
  // Initialize the lead filter
  leadinit(&lead[0], 11.0f, 180.0f, 0.35f); // roll
  leadinit(&lead[1], 11.0f, 180.0f, 0.35f); // pitch
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const stabilizerStep_t stabilizerStep) {

  control->controlMode = controlModeForceTorque;

  // dt
  float dt = 0.01f; // Assuming a fixed time step for simplicity

  if (RATE_DO_EXECUTE(100, stabilizerStep)) {
    // error
    error[0] = setpoint->position.x - state->position.x;
    error[1] = setpoint->position.y - state->position.y;
    error[2] = setpoint->position.z - state->position.z;

    // Update the PID controllers
    ootpidstep(&ootpids[0], error[0], dt); // x
    ootpidstep(&ootpids[1], error[1], dt); // y
    ootpidstep(&ootpids[2], error[2], dt); // z
  }
  if (RATE_DO_EXECUTE(250, stabilizerStep)) {

    // roll + pitch error
    error[3] = ootpids[0].output - state->attitude.roll;
    error[4] = ootpids[1].output - state->attitude.pitch;

    // Update the lead controllers
    leadupdate(&lead[0], error[3], dt); // roll
    leadupdate(&lead[1], error[4], dt); // pitch
  }

  control->thrustSi = ootpids[2].output; // N
  control->torqueX = lead[0].output;     // Nm
  control->torqueY = lead[1].output;     // Nm
  control->torqueZ = 0.0f;               // Nm
}

bool controllerOutOfTreeTest(){
  return true;
}

// void controllerOutOfTreeInit() {
//   controllerBrescianiniInit();
// }

// void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
//   const sensorData_t *sensors, const state_t *state,
//   const stabilizerStep_t stabilizerStep) {

//   controllerBrescianini(control, setpoint, sensors, state, stabilizerStep);
// }
