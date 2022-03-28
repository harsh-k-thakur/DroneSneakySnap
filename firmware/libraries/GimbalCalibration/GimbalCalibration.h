#ifndef GimbalCalibration

#define GimbalCalibration
#include <Arduino.h>
#include <EEPROM.h>

// ------ ***** ------ ***** ------ ***** ------ ***** ------

const int DEADBAND = 8;

const int THROTTLE_MAP_MIN = 0;
const int THROTTLE_MAP_MAX = 255;
const int PITCH_MAP_MIN = -127;
const int PITCH_MAP_MAX = 127;
const int ROLL_MAP_MIN = -127;
const int ROLL_MAP_MAX = 127;
const int YAW_MAP_MIN = -127;
const int YAW_MAP_MAX = 127;

// ------ ***** ------ ***** ------ ***** ------ ***** ------
struct CalibrationValues
{
	int min_throttle;
	int max_throttle;
	int min_pitch;
	int center_pitch;
	int max_pitch;
	int min_roll;
	int center_roll;
	int max_roll;
	int min_yaw;
	int center_yaw;
	int max_yaw;
};

struct GimbalValues
{
	int throttle;
	int pitch;
	int roll;
	int yaw;
};

// ------ ***** ------ ***** ------ ***** ------ ***** ------
void load_gimbal_values(CalibrationValues& saved_values, int address);
void save_gimbal_values(CalibrationValues& saved_values, int address);
void read_raw_gimbal_values(GimbalValues& raw);
void empty_calibration(CalibrationValues& saved_values, GimbalValues& raw);
void perfom_calibration(CalibrationValues& saved_values, GimbalValues& raw);
void scale_gimbals(CalibrationValues& saved_values, GimbalValues& raw, GimbalValues& scaled);
int map_pry(int value, int min_value, int center_value, int max_value, int min, int max);
bool is_armed(GimbalValues& scaled);



#endif

