#include "Arduino.h"
#include "CommPacket.h"

void buildPacket(Packet& packet, bool armed, bool single_motor_mode, uint8_t motor_select, uint8_t parameter_select, ParameterValue gain_value, uint8_t throttle, int8_t pitch,  int8_t roll, int8_t yaw) {
  packet.magicNum = MAGIC_BYTE;
  packet.misc = (armed << 7) | (single_motor_mode << 6) | (motor_select << 4) | parameter_select;
  packet.gain_value = gain_value;
  packet.throttle = throttle;
  packet.pitch = pitch;
  packet.roll = roll;
  packet.yaw = yaw;
}

void setParametersFromPacket(Packet& packet, ParameterValues& parameters) {
  if ((getSelectedParameter(packet) & B1100) == B0000) {
    switch (getSelectedParameter(packet)) {
      case COMPLEMENTARY_GAIN:
        parameters.complementary = packet.gain_value.continuous_gain;
        break;
      case GYRO_HIGHPASS:
        parameters.gyro_highpass = packet.gain_value.register_value;
        break;
      case XL_LOWPASS:
        parameters.accel_lowpass_ratio = packet.gain_value.register_value;
        break;
    }
  } else {
    PIDValues* values;
    if ((getSelectedParameter(packet) & B1100) == B0100) {
      values = &parameters.pitch_values;
    } else if ((getSelectedParameter(packet) & B1100) == B1000) {
      values = &parameters.yaw_values;
    } else {
      values = &parameters.roll_values;
    }
    
    switch (getSelectedParameter(packet) & B11) {
      case B00:
        values->p = packet.gain_value.continuous_gain;
        break;
      case B01:
        values->i = packet.gain_value.continuous_gain;
        break;
      case B10:
        values->d = packet.gain_value.continuous_gain;
        break;
      case B11:
        values->trim = packet.gain_value.continuous_gain;
        break;
    }
  }
}

void buildTelemetry(Telemetry& telemetry, bool armed, float pitch_err, float roll_err, float yaw_err) {
  telemetry.magicNum = TELEMETRY_MAGIC_BYTE;
  telemetry.armed = armed;
  telemetry.pitch_err = pitch_err;
  telemetry.roll_err = roll_err;
  telemetry.yaw_err = yaw_err;
}
