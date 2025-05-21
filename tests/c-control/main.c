// main.c that takes a string and prints it to the console
#include "cf_types.h"
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f

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

ootpid_t ootpids[5];
lead_t lead[2];
float error[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float thrust = 0.0f;
float torque[2] = {0.0f, 0.0f};
float xy[2] = {0.0f, 0.0f};
float gerror[2] = {0.0f, 0.0f};

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
  // ootpid->integral += error * dt;

  // Calculate the derivative
  float derivative = (ootpid->last_error - error) / dt;

  // Calculate the and set the output
  ootpid->output = ootpid->kp * error + ootpid->ki * ootpid->integral +
                   ootpid->kd * derivative;

  printf(
      "Error: %f, lasterror: %f, prop_term: %f, deriv_term: %f, output: %f \n",
      error, ootpid->last_error, ootpid->kp * error, ootpid->kd * derivative,
      ootpid->output);

  // Save the last error
  ootpid->last_error = error;
}
void leadinit(lead_t *lead, float z, float p, float k) {
  lead->z = z;
  lead->p = p;
  lead->k = k;
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
  ootPidInit(&ootpids[0], 0.15f, 0.0f, 0.11f);   // x
  ootPidInit(&ootpids[1], 0.15f, 0.0f, 0.11f);   // y
  ootPidInit(&ootpids[2], 0.24f, 0.084f, 0.17f); // z
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

  gerror[0] = setpoint->position.x - state->position.x;
  gerror[1] = setpoint->position.y - state->position.y;
  error[2] = setpoint->position.z - state->position.z;

  error[0] = gerror[0] * cosf(state->attitude.yaw / 180.0f * PI) -
             gerror[1] * sinf(state->attitude.yaw / 180.0f * PI);
  error[1] = gerror[0] * sinf(state->attitude.yaw / 180.0f * PI) +
             gerror[1] * cosf(state->attitude.yaw / 180.0f * PI);

  // Update the PID controllers
  printf("Start\n");
  ootpidstep(&ootpids[0], error[0], dt); // x
  ootpidstep(&ootpids[1], error[1], dt); // y
  ootpidstep(&ootpids[2], error[2], dt); // z

  xy[0] = -ootpids[0].output;
  xy[1] = -ootpids[1].output;

  // // clamping
  // if (xy[0] > 0.01f) {
  //   xy[0] = 0.01f;
  // } else if (xy[0] < -0.01f) {
  //   xy[0] = -0.01f;
  // }
  // if (xy[1] > 0.01f) {
  //   xy[1] = 0.01f;
  // } else if (xy[1] < -0.01f) {
  //   xy[1] = -0.01f;
  // }

  control->torqueY = xy[1];
  control->torqueX = xy[0];
}

int main(int argc, char *argv[]) {
  if (argc < 1) {
    return 1;
  }
  // Initialize the controller
  controllerOutOfTreeInit();
  // Create a control structure
  control_t control;
  // Create a setpoint structure
  setpoint_t setpoint;
  // Create a sensor data structure
  sensorData_t sensors;
  // Create a state structure
  state_t state;

  // Initialize the setpoint
  setpoint.position.x = 1.0f;
  setpoint.position.y = 1.0f;
  setpoint.position.z = 1.0f;
  setpoint.attitude.roll = 0.0f;
  setpoint.attitude.pitch = 0.0f;
  setpoint.attitude.yaw = 0.0f;

  // Initialize the state
  state.position.x = 0.0f;
  state.position.y = 0.0f;
  state.position.z = 0.0f;
  state.attitude.roll = 0.0f;
  state.attitude.pitch = 0.0f;
  state.attitude.yaw = 1.0f;
  state.velocity.x = 0.0f;
  state.velocity.y = 0.0f;
  state.velocity.z = 0.0f;
  state.acc.x = 0.0f;
  state.acc.y = 0.0f;
  state.acc.z = 0.0f;

  // Initialize the control
  control.thrustSi = 0.0f;
  control.torqueX = 0.0f;
  control.torqueY = 0.0f;
  control.torqueZ = 0.0f;

  // print header
  // printf("torqueX torqueY position.x position.y gerr.x gerr.y err.x
  // err.y\n");

  for (int i = 0; i < 100; i++) {
    // Call the controller
    controllerOutOfTree(&control, &setpoint, &sensors, &state, i);
    // Simulate the step
    if (control.torqueX > 0.0f) {
      state.position.x -= 0.01f;
    } else if (control.torqueX < 0.0f) {
      state.position.x += 0.01f;
    }
    // Print the control output and state
    // printf("%f, %f, %f, %f, %f, %f, %f, %f\n", control.torqueX,
    // control.torqueY,
    //        state.position.x, state.position.y, gerror[0], gerror[1],
    //        error[0], error[1]);
  }
  return 0;
}
