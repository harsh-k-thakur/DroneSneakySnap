#include "Arduino.h"
#include "GimbalCalibration.h"
#include "quad_remote.h"

// ------ ***** ------ ***** ------ ***** ------ ***** ------
unsigned long last_calibration_update = 0;


// ------ ***** ------ ***** ------ ***** ------ ***** ------
void load_gimbal_values(CalibrationValues& saved_values, int address)
{
	EEPROM.get(address, saved_values);
}


// ------ ***** ------ ***** ------ ***** ------ ***** ------
void save_gimbal_values(CalibrationValues& saved_values, int address)
{
	EEPROM.put(address, saved_values);
}


// ------ ***** ------ ***** ------ ***** ------ ***** ------
void read_raw_gimbal_values(GimbalValues& raw)
{
	raw.throttle = analogRead(PIN_THROTTLE);
	raw.pitch = analogRead(PIN_PITCH);
	raw.roll = analogRead(PIN_ROLL);
	raw.yaw = analogRead(PIN_YAW);
}

// ------ ***** ------ ***** ------ ***** ------ ***** ------
void empty_calibration(CalibrationValues& saved_values, GimbalValues& raw)
{
	saved_values.min_throttle = raw.throttle;
	saved_values.max_throttle = raw.throttle;
	saved_values.min_pitch = raw.pitch;
	saved_values.max_pitch = raw.pitch;
	saved_values.center_pitch = raw.pitch;
	saved_values.min_roll = raw.roll;
	saved_values.max_roll = raw.roll;
	saved_values.center_roll = raw.roll;
	saved_values.min_yaw = raw.yaw;
	saved_values.max_yaw = raw.yaw;
	saved_values.center_yaw = raw.yaw;
}


// ------ ***** ------ ***** ------ ***** ------ ***** ------
void perfom_calibration(CalibrationValues& values, GimbalValues& raw)
{
	if (raw.throttle > values.max_throttle) {
		values.max_throttle =  raw.throttle;
		last_calibration_update = millis();
	} else if (raw.throttle < values.min_throttle) {
		values.min_throttle =  raw.throttle;
		last_calibration_update = millis();
	}

	if (raw.pitch > values.max_pitch) {
		values.max_pitch =  raw.pitch;
		last_calibration_update = millis();
	} else if (raw.pitch < values.min_pitch) {
		values.min_pitch =  raw.pitch;
		last_calibration_update = millis();
	}

	if (raw.roll > values.max_roll) {
		values.max_roll =  raw.roll;
		last_calibration_update = millis();
	}  else if (raw.roll < values.min_roll) {
		values.min_roll =  raw.roll;
		last_calibration_update = millis();
	}

	if (raw.yaw > values.max_yaw) {
		values.max_yaw =  raw.yaw;
		last_calibration_update = millis();
	} else if (raw.yaw < values.min_yaw) {
		values.min_yaw =  raw.yaw;
		last_calibration_update = millis();
	}
}

// ------ ***** ------ ***** ------ ***** ------ ***** ------
int map_pry(int value, int min_value, int center_value, int max_value, int min, int max)
{
	// 667 < 673
	// 1023 (center = 647 )
	// Serial.println(center_value + DEADBAND);
	if (value > (center_value + DEADBAND)) {
		return constrain(map(value, (center_value+DEADBAND), max_value, 0, max), 0, max);
	} else if (value < (center_value - DEADBAND)) {
		return constrain(map(value, min_value, (center_value-DEADBAND), min, 0), min, 0);
	} else {
		return 0;
	}
}


// ------ ***** ------ ***** ------ ***** ------ ***** ------
void scale_gimbals(CalibrationValues& original, GimbalValues& raw, GimbalValues& scaled)
{
	scaled.throttle = constrain(map(raw.throttle, original.min_throttle, original.max_throttle, THROTTLE_MAP_MIN, THROTTLE_MAP_MAX), THROTTLE_MAP_MIN, THROTTLE_MAP_MAX);
	scaled.pitch = -map_pry(raw.pitch, original.min_pitch, original.center_pitch, original.max_pitch, PITCH_MAP_MIN, PITCH_MAP_MAX);
	scaled.roll = map_pry(raw.roll, original.min_roll, original.center_roll, original.max_roll, ROLL_MAP_MIN, ROLL_MAP_MAX);
	scaled.yaw = -map_pry(raw.yaw, original.min_yaw, original.center_yaw, original.max_yaw, YAW_MAP_MIN, YAW_MAP_MAX);
	// Serial.print(raw.yaw); Serial.print(" "); Serial.print(original.min_yaw); Serial.print(" "); Serial.print(original.max_yaw); Serial.print(" "); Serial.println(original.center_yaw);
}

// ------ ***** ------ ***** ------ ***** ------ ***** ------
bool is_armed(GimbalValues& scaled)
{
	if (scaled.throttle == THROTTLE_MAP_MIN && 
		scaled.pitch == PITCH_MAP_MIN && 
		scaled.roll == ROLL_MAP_MAX  &&
		scaled.yaw == YAW_MAP_MIN)
	{
		return true;
	}
	return false;
}