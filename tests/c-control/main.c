// main.c that takes a string and prints it to the console
#include "cf_types.h"
#include <stdio.h>

extern void simulate_step(state_t *state, const control_t *control);

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

    // roll + pitch error
    error[3] = ootpids[0].output - state->attitude.roll;
    error[4] = ootpids[1].output - state->attitude.pitch;

    // Update the lead controllers
    leadupdate(&lead[0], error[3], dt); // roll
    leadupdate(&lead[1], error[4], dt); // pitch

    control->thrustSi += ootpids[2].output; // N
    control->torqueX += lead[0].output;     // Nm
    control->torqueY += -lead[1].output;    // Nm
    control->torqueZ += 0.0f;               // Nm

    // positive thrust
    if (control->thrustSi < 0.0f) {
      control->thrustSi = 0.0f;
    }
  }
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
  state.attitude.yaw = 0.0f;
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
  printf("Control Output: thrustSi torqueX torqueY torqueZ; State: position.x "
         "position.y position.z attitude.roll attitude.pitch attitude.yaw\n");

  for (int i = 0; i < 500; i++) {
    // Call the controller
    controllerOutOfTree(&control, &setpoint, &sensors, &state, i);
    // Simulate the step
    simulate_step(&state, &control);
    // Print the control output and state
    printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,\n", control.thrustSi,
           control.torqueX, control.torqueY, control.torqueZ, state.position.x,
           state.position.y, state.position.z, state.attitude.roll,
           state.attitude.pitch, state.attitude.yaw);
  }
  printf("%i\n", 60);
  return 0;
}
