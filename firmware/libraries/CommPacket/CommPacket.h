#ifndef CommPacket
#define CommPacket

#include <Arduino.h>

const uint8_t MAGIC_BYTE = 0x37;
const uint8_t TELEMETRY_MAGIC_BYTE = 0xaa;

const uint8_t NO_PARAMETER = B0000;
const uint8_t COMPLEMENTARY_GAIN = B0001;
const uint8_t GYRO_HIGHPASS = B0010; // CTRL_REG3_G  HPCF_G
const uint8_t XL_LOWPASS = B0011; // CTRL_REG7_XL DCF
const uint8_t PITCH_P = B0100;
const uint8_t PITCH_I = B0101;
const uint8_t PITCH_D = B0110;
const uint8_t PITCH_TRIM = B0111;
const uint8_t YAW_P = B1000;
const uint8_t YAW_I = B1001;
const uint8_t YAW_D = B1010;
const uint8_t YAW_TRIM = B1011;
const uint8_t ROLL_P = B1100;
const uint8_t ROLL_I = B1101;
const uint8_t ROLL_D = B1110;
const uint8_t ROLL_TRIM = B1111;

const uint8_t FRONT_LEFT = 0x00;
const uint8_t FRONT_RIGHT = 0x01;
const uint8_t BACK_LEFT = 0x02;
const uint8_t BACK_RIGHT = 0x03;

const uint8_t PITCH_ID = 0x00;
const uint8_t ROLL_ID = 0x01;
const uint8_t YAW_ID = 0x02;
const uint8_t THROTTLE_ID = 0x03;

struct PIDValues {
  float p;
  float i;
  float d;
  float trim;
};

struct ParameterValues {
  PIDValues pitch_values;
  PIDValues roll_values;
  PIDValues yaw_values;
  float complementary;
  uint8_t gyro_highpass;
  uint8_t accel_lowpass_ratio;
};

union ParameterValue {
  float continuous_gain;
  uint8_t register_value;
};

struct Packet {
  uint8_t magicNum;
  uint8_t misc;
  ParameterValue gain_value;
  uint8_t throttle;
  int8_t pitch;
  int8_t roll;
  int8_t yaw;
};

struct Telemetry {
  uint8_t magicNum;
  bool armed;
  float pitch_err;
  float roll_err;
  float yaw_err;
};

void buildPacket(Packet& packet, bool armed, bool single_motor_mode, uint8_t motor_select, uint8_t parameter_select, ParameterValue gain_value, uint8_t throttle, int8_t pitch,  int8_t roll, int8_t yaw);

void buildTelemetry(Telemetry& telemetry, bool armed, float pitch_err, float roll_err, float yaw_err);
void setParametersFromPacket(Packet& packet, ParameterValues& parameters);

inline bool checkPacket(const Packet &p) {return p.magicNum == MAGIC_BYTE;}
inline bool isArmed(const Packet &p) {return p.misc & 0x80;}
inline bool isSingleMotor(const Packet &p) {return p.misc & 0x40;}
inline uint8_t getSelectedMotor(const Packet &p) {return (p.misc & 0x30) >> 4;}
inline uint8_t getSelectedParameter(const Packet &p) {return p.misc & 0x0F;}

inline bool checkTelemetry(const Telemetry &telemetry) {return telemetry.magicNum == TELEMETRY_MAGIC_BYTE;}
inline bool isTelemetryArmed(const Telemetry &telemetry) {return telemetry.armed;}


#endif
