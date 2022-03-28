#ifndef RadioPacket

#define RadioPacket
#include <Arduino.h>

// ------ ***** ------ ***** ------ ***** ------ ***** ------
const uint8_t MAGIC_BYTE = 0x37;
const uint8_t TELEMETRY_MAGIC_BYTE = 0xaa;

const uint8_t COMPLEMENTARY_GAIN = B0001;
const uint8_t GYRO_HIGHPASS = B0010;
const uint8_t XL_LOWPASS = B0011;

const uint8_t PITCH_P = B0100;
const uint8_t PITCH_I = B0101;
const uint8_t PITCH_D = B0111;
const uint8_t YAW_P = B1000;
const uint8_t YAW_I = B1001;
const uint8_t YAW_D = B1011;
const uint8_t ROLL_P = B1100;
const uint8_t ROLL_I = B1101;
const uint8_t ROLL_D = B1111;

const uint8_t FRONT_LEFT = 0x00;
const uint8_t FRONT_RIGHT = 0x01;
const uint8_t BACK_LEFT = 0x02;
const uint8_t BACK_RIGHT = 0x03;

const uint8_t PITCH_ID = 0x00;
const uint8_t ROLL_ID = 0x01;
const uint8_t YAW_ID = 0x02;
const uint8_t THROTTLE_ID = 0x03;

// ------ ***** ------ ***** ------ ***** ------ ***** ------
struct Telemetry 
{
	uint8_t magic_number;
	bool armed;
};

struct PIDValues
{
	float p;
	float i;
	float d;
	float t;
};


struct ParameterValues
{
	PIDValues pitch_values;
	PIDValues roll_values;
	PIDValues yaw_values;
	float complementary;
	uint8_t gyro_highpass;
	uint8_t accel_lowpass_ratio;
};

union ParameterValue
{
	float continuous_gain;
	uint8_t register_value;
};

struct Packet
{
	uint8_t magic_number;
	uint8_t features;
	ParameterValue gain_value;
	uint8_t throttle;
	int8_t pitch;
	int8_t roll;
	int8_t yaw;
};


// ------ ***** ------ ***** ------ ***** ------ ***** ------
void build_packet(Packet& packet, bool armed, uint8_t throttle, int8_t pitch, int8_t roll, int8_t yaw);

bool check_packet(Packet& packet);
bool is_armed(Packet& packet);

uint8_t getSelectedParameter(const Packet &p);

void build_telemetry(Telemetry& telemetry, bool armed);
bool check_telemetry(Telemetry & telemetry);
bool is_telemetry_armed(Telemetry & telemetry);



#endif

