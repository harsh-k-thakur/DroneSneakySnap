#include "Arduino.h"
#include "RadioPacket.h"

void build_packet(Packet& packet, bool armed, uint8_t throttle, int8_t pitch, int8_t roll, int8_t yaw)
{
	packet.magic_number = MAGIC_BYTE;
	packet.features = (armed << 7);
	packet.throttle = throttle;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.yaw = yaw;
}

void setParametersFromPacket(Packet& packet, ParameterValues& parameters)
{
	if ((getSelectedParameter(packet) & B1100) == B0000)
	{
		switch (getSelectedParameter(packet))
		{
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
	}
	else 
	{
		PIDValues* values;
		if ((getSelectedParameter(packet) & B1100) == B0100) 
		{
			values = &parameters.pitch_values;
		}
		else if ((getSelectedParameter(packet) & B1100) == B1000)
		{
			values = &parameters.yaw_values;
		}
		else
		{
			values = &parameters.roll_values;
		}

		if ((getSelectedParameter(packet) & B11) == B00)
		{
			values->p = packet.gain_value.continuous_gain;
		}
		else if ((getSelectedParameter(packet) & B11) == B01)
		{
			values->i = packet.gain_value.continuous_gain;
		}
		else
		{
      			values->d = packet.gain_value.continuous_gain;
		}
	}
}

uint8_t getSelectedParameter(Packet &p)
{
	return p.features & 0x0F;
}

bool check_packet(Packet& packet)
{
	packet.magic_number == MAGIC_BYTE;
}

void build_telemetry(Telemetry& telemetry, bool armed)
{
	telemetry.magic_number = TELEMETRY_MAGIC_BYTE;
	telemetry.armed = armed;
}

bool check_telemetry(Telemetry& telemetry)
{
	return telemetry.magic_number == TELEMETRY_MAGIC_BYTE;
}

bool is_telemetry_armed(Telemetry & telemetry)
{
	return telemetry.armed;
}

bool is_armed(Packet& packet)
{
	return packet.features & 0x0F;
}