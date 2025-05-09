struct PacketRX {
  struct {
    float x, y, z;
  } pos;
} __attribute__((packed));

struct PacketTX {
  struct {
    float x, y, z;
  } position;
  struct {
    float roll, pitch, yaw;
  } attitude;
  int8_t compareResult;

  // float batteryVoltage;
  // float cmd_thrust;
  // bool debugState;
} __attribute__((packed));
