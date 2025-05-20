#include "controller_brescianini.h"
#include "log.h"
#include "stabilizer_types.h"
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
float error[5];
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

void resetAllPid() {
  for (int i = 0; i < 3; i++) {
    ootpids[i].integral = 0.0f;
    ootpids[i].last_error = 0.0f;
  }
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
  // Update the last input and output
  lead->lastInput = input;
  lead->lastOutput = lead->output;
}

void controllerOutOfTreeInit() {
  // Initialize the PID controllers
  ootPidInit(&ootpids[0], -0.65f, -0.15f, -0.51f); // x
  ootPidInit(&ootpids[1], -0.65f, -0.15f, -0.51f); // y
  ootPidInit(&ootpids[2], 0.24f, 0.084f, 0.17f);   // z
                                                   //
  ootPidInit(&ootpids[3], 0.0f, 0.0f, 0.0006f);    // roll
  ootPidInit(&ootpids[4], 0.0f, 0.0f, 0.0008f);    // pitch
  // Initialize the lead filter
  leadinit(&lead[0], 5.0f, 60.0f, 0.04f); // roll
  leadinit(&lead[1], 5.0f, 60.0f, 0.07f); // pitch
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const stabilizerStep_t stabilizerStep) {

  control->controlMode = controlModeForceTorque;

  // dt
  float dt = 0.002f; // Assuming a fixed time step for simplicity

  if (RATE_DO_EXECUTE(500, stabilizerStep) && setpoint->position.x != 0.0f &&
      setpoint->position.y != 0.0f && setpoint->position.z != 0.0f) {
    // error
    gerror[0] = setpoint->position.x - state->position.x;
    gerror[1] = setpoint->position.y - state->position.y;
    error[2] = setpoint->position.z - state->position.z;

    error[0] = gerror[0] * cosf(state->attitude.yaw / 180.0f * PI) +
               gerror[1] * sinf(state->attitude.yaw / 180.0f * PI);
    error[1] = gerror[0] * -sinf(state->attitude.yaw / 180.0f * PI) +
               gerror[1] * cosf(state->attitude.yaw / 180.0f * PI);

    // Update the PID controllers
    ootpidstep(&ootpids[0], error[0], dt); // x
    ootpidstep(&ootpids[1], error[1], dt); // y
    ootpidstep(&ootpids[2], error[2], dt); // z

    xy[0] = ootpids[0].output;
    xy[1] = ootpids[1].output;

    // clamping
    if (xy[0] > 0.1f) {
      xy[0] = 0.1f;
    } else if (xy[0] < -0.1f) {
      xy[0] = -0.1f;
    }
    if (xy[1] > 0.1f) {
      xy[1] = 0.1f;
    } else if (xy[1] < -0.1f) {
      xy[1] = -0.1f;
    }

    // roll + pitch error
    error[3] = xy[1] - (state->attitude.roll / 180.0f * PI);
    error[4] = xy[0] + (state->attitude.pitch / 180.0f * PI);

    // error[3] = 0 - (state->attitude.roll / 180.0f * PI);
    // error[4] = 0 + (state->attitude.pitch / 180.0f * PI);

    // Update the lead controllers
    leadupdate(&lead[0], error[3], dt); // roll
    leadupdate(&lead[1], error[4], dt); // pitch
    torque[0] = lead[0].output;
    torque[1] = lead[1].output;

    ootpidstep(&ootpids[3], error[3], dt); // roll
    ootpidstep(&ootpids[4], error[4], dt); // pitch
    torque[0] += ootpids[3].output;
    torque[1] += ootpids[4].output;

    // Update the thrust
    thrust = 0.40f + ootpids[2].output; // N
    // thrust = 0.40f;

    // clamp torques
    if (torque[0] > 0.005f) {
      torque[0] = 0.005f;
    } else if (torque[0] < -0.005f) {
      torque[0] = -0.005f;
    }

    if (torque[1] > 0.005f) {
      torque[1] = 0.005f;
    } else if (torque[1] < -0.005f) {
      torque[1] = -0.005f;
    }
  }
  control->thrustSi = thrust;   // N
  control->torqueX = torque[0]; // torque[0]; // Nm
  control->torqueY = torque[1]; // Nm
  control->torqueZ = 0.0f;      // Nm
}

bool controllerOutOfTreeTest() { return true; }

// void controllerOutOfTreeInit() {
//   controllerBrescianiniInit();
// }

// void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
//   const sensorData_t *sensors, const state_t *state,
//   const stabilizerStep_t stabilizerStep) {

//   controllerBrescianini(control, setpoint, sensors, state, stabilizerStep);
// }

LOG_GROUP_START(con)
// log add pids
LOG_ADD(LOG_FLOAT, x, &ootpids[0].output)
LOG_ADD(LOG_FLOAT, y, &ootpids[1].output)
LOG_ADD(LOG_FLOAT, z, &ootpids[2].output)

// log add lead
LOG_ADD(LOG_FLOAT, r, &lead[0].output)
LOG_ADD(LOG_FLOAT, p, &lead[1].output)

// log add error
LOG_ADD(LOG_FLOAT, err_x, &error[0])
LOG_ADD(LOG_FLOAT, err_y, &error[1])
LOG_ADD(LOG_FLOAT, err_z, &error[2])
LOG_ADD(LOG_FLOAT, err_r, &error[3])
LOG_ADD(LOG_FLOAT, err_p, &error[4])

LOG_GROUP_STOP(con)
