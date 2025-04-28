#include <stdbool.h>

struct PacketRX {
  bool messageType;
  struct {
    float x, y, z;
  } v1;
  struct {
    float x, y, z;
  } v2;

  // union
  // {
  //   struct{float x, y, z;}state_pos;
  //   struct{float x, y, z;}state_att;
  // }v1;

  // union
  // {
  //   struct{float x, y, z;}state_vel;
  //   struct{float x, y, z;}setpoint_pos;
  // }v2;
} __attribute__((packed));

struct PacketTX {
  struct {
    float x, y, z;
  } position;
  float batteryVoltage;
  float cmd_thrust;
  bool debugState;
} __attribute__((packed));
