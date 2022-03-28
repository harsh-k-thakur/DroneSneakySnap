#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include <CommPacket.h>
#include <Debouncer.h>
#include <EEPROM.h>

// Constants
const int DEADBAND = 10;
const int PACKET_PERIOD_MS = 50;
const int CALIBRATION_TIMEOUT_MS = 5000; // How long to wait with no calibration value changes before telling the user they're done
const int DEBOUNCE_TIME_MS = 200;
const int NO_TELEMETRY_DISARM_TIMEOUT = 2000;
const float PID_ADJUSTMENT_GRANULARITY = 0.05;
const float TRIM_ADJUSTMENT_GRANULARITY = 1.0;


// States
const byte DISARMED = 0;
const byte CALIBRATION = 1;
const byte ARMED = 2;

// Parameter adjustment UI categories
const byte SENSING = 0;
const byte PITCH_LOOP = 1;
const byte ROLL_LOOP = 2;
const byte YAW_LOOP = 3;

const byte NUM_CATEGORIES = 4;

// Parameters
const byte P = 0;
const byte I = 1;
const byte D = 2;
// None is 0
const byte COMPLEMENTARY = 1;
const byte GYRO_HP = 2;
const byte XL_LP_RATIO = 3;

// struct for calibration values. rest_* means the value the stick has when it's not being touched.
struct CalibrationValues {
  int min_throttle;
  int max_throttle;
  int min_yaw;
  int rest_yaw;
  int max_yaw;
  int min_roll;
  int rest_roll;
  int max_roll;
  int min_pitch;
  int rest_pitch;
  int max_pitch;
};

CalibrationValues calibration;
ParameterValues parameters;


// Predeclarations for callback functions
void btn_one_pressed(bool down);
void btn_two_pressed(bool down);
void btn_up_pressed(bool down);
void btn_down_pressed(bool down);
void btn_left_pressed(bool down);
void btn_right_pressed(bool down);
void btn_center_pressed(bool down);

// State
byte state = DISARMED;

// Parameter selection
byte parameter_category;
byte selected_parameter;

// Scaled values for the axes
byte throttle_value;
int8_t yaw_value;
int8_t roll_value;
int8_t pitch_value;

// Raw values from the gimbals for each axis
int throttle_raw;
int yaw_raw;
int roll_raw;
int pitch_raw;

// For single motor mode, where only one motor on the quadcopter is run.
bool single_motor_mode = false;
byte motor_select = FRONT_LEFT;

// For trim UI
bool adjust_trim = false;
byte which_trim = PITCH_TRIM;

// Most recent telemetry data from the quadcopter
Telemetry active_telemetry;

// Local variables we don't want to reallocate every loop
Packet p;
uint8_t b[sizeof(active_telemetry)];
char newFirstLine[17];
char newSecondLine[17];

// Display strings
// 16 character wide, 2-line display
char displayLine1[17];
char displayLine2[17];
unsigned long displayRGB;

// Last time a packet was sent to the quadcopter
unsigned long last_send_time = 0;
// Last time the calibration data were updated.
unsigned long last_calibration_update = 0;
// For debouncing
Debouncer btn1_debouncer = Debouncer(DEBOUNCE_TIME_MS);
Debouncer btn2_debouncer = Debouncer(DEBOUNCE_TIME_MS);
Debouncer btn_left_debouncer = Debouncer(DEBOUNCE_TIME_MS);
Debouncer btn_right_debouncer = Debouncer(DEBOUNCE_TIME_MS);
Debouncer btn_up_debouncer = Debouncer(DEBOUNCE_TIME_MS);
Debouncer btn_center_debouncer = Debouncer(DEBOUNCE_TIME_MS);
Debouncer btn_down_debouncer = Debouncer(DEBOUNCE_TIME_MS);
Debouncer knob_debouncer = Debouncer(DEBOUNCE_TIME_MS);
// Last time a telemetry packet was recieved
unsigned long last_telemetry_time = 0;

// EEPROM addresses
const int CALIBRATION_EEPROM_ADDR = 0;
const int PARAMETER_EEPROM_ADDR = CALIBRATION_EEPROM_ADDR + sizeof(calibration);
const int PARAMETER_CATEGORY_EEPROM_ADDR = PARAMETER_EEPROM_ADDR + sizeof(parameters);
const int PARAMETER_SELECTION_EEPROM_ADDR = PARAMETER_CATEGORY_EEPROM_ADDR + sizeof(parameter_category);

void setup() {
  const int SERIAL_BAUD = 19200;        // Baud rate for serial port 
  Serial.begin(SERIAL_BAUD);           // Start up serial
  delay(100);
  quad_remote_setup();
  rfBegin(12);

  // Read EEPROM values
  EEPROM.get(CALIBRATION_EEPROM_ADDR, calibration);
  EEPROM.get(PARAMETER_EEPROM_ADDR, parameters);
  EEPROM.get(PARAMETER_CATEGORY_EEPROM_ADDR, parameter_category);
  EEPROM.get(PARAMETER_SELECTION_EEPROM_ADDR, selected_parameter);
//  parameters = {
//    {0.55,  0.10, 0.20, 0.0},     //pitch
//    {0.55,  0.10, 0.20, 0.0},     //roll
//    {3.3,   0.0,  5.0,  0.0},     //yaw
//    0.95,
//    B1001,
//    B00
//  };
  parameter_category = SENSING;
  selected_parameter = 0;

//  parameters.pitch_values.trim = 0;
//  parameters.roll_values.trim = 0;
//  parameters.yaw_values.trim = 0;
  

  state = DISARMED;
  // Button 1 is the calibration button
  btn1_cb = btn_one_pressed;
  btn2_cb = btn_two_pressed;
        
  knobs_update_cb = knobs_update; 
  knob1_btn_cb = knob_pressed;
  btn_up_cb =  btn_up_pressed;
  btn_down_cb = btn_down_pressed;
  btn_left_cb =  btn_left_pressed;
  btn_right_cb = btn_right_pressed;
  btn_center_cb = btn_center_pressed;
}

void loop() {
  read_gimbals();
  recieve_telemetry();
  check_telemetry_timeout();

  // State-dependent routines
  if (state == ARMED) {
    scale_gimbals();
  } else if (state == CALIBRATION) {
    do_calibration();
  } else if (state == DISARMED) {
    scale_gimbals();
    check_for_arm();
  }

  // Only send one packet per PACKET_PERIOD_MS
  if (millis() - last_send_time >  PACKET_PERIOD_MS) {
    send_packet();
  }

  set_display();
  delay(2);
}

/**
 * Toggle calibration mode. Resets saved calibration data when entering calibration mode.
 */
void btn_one_pressed(bool down) {
  // Only on-press
  if (!down || !btn1_debouncer.debounce()) {
    return;
  }
  
  if (state == DISARMED) {
    state = CALIBRATION;

    // Set this so display doesn't immediately prompt to end calibration
    last_calibration_update = millis();

    // Reset data and read rest values
    calibration = {
      throttle_raw,
      throttle_raw,
      yaw_raw,
      yaw_raw,
      yaw_raw,
      roll_raw,
      roll_raw,
      roll_raw,
      pitch_raw,
      pitch_raw,
      pitch_raw
    };
  } else if (state == CALIBRATION) {
    state = DISARMED;
    EEPROM.put(CALIBRATION_EEPROM_ADDR, calibration);
  }
}

void btn_two_pressed(bool down) {
  // Only on-press
  if (!down || !btn2_debouncer.debounce()) {
    return;
  }
  
  if (single_motor_mode && motor_select == FRONT_LEFT) {
    single_motor_mode = false;
  } else {
    single_motor_mode = true;
    motor_select = (motor_select + 1) & B00000011;
  }
}

void btn_left_pressed(bool down) {
  // Only on-press
  if (!down || !btn_left_debouncer.debounce()) {
    return;
  }

  if (adjust_trim) {
    which_trim = ROLL_TRIM;
    parameters.roll_values.trim -= TRIM_ADJUSTMENT_GRANULARITY;
  } else {
    if (parameter_category == 0) {
      parameter_category = NUM_CATEGORIES;
    }
    parameter_category = parameter_category - 1;
    
    // lower max to 2 in case we're switching from sensing to PID
    selected_parameter = constrain(selected_parameter, 0, 2);
    
    EEPROM.put(PARAMETER_CATEGORY_EEPROM_ADDR, parameter_category);
  }
}

void btn_right_pressed(bool down) {
  // Only on-press
  if (!down || !btn_right_debouncer.debounce()) {
    return;
  }

  if (adjust_trim) {
    which_trim = ROLL_TRIM;
    parameters.roll_values.trim += TRIM_ADJUSTMENT_GRANULARITY;
  } else {
    parameter_category = (parameter_category + 1) % NUM_CATEGORIES;
    
    // lower max to 2 in case we're switching from sensing to PID
    selected_parameter = constrain(selected_parameter, 0, 2);
    
    EEPROM.put(PARAMETER_CATEGORY_EEPROM_ADDR, parameter_category);
  }
}

void btn_down_pressed(bool down) {
  // Only on-press
  if (!down || !btn_down_debouncer.debounce()) {
    return;
  }

  if (adjust_trim) {
    which_trim = PITCH_TRIM;
    parameters.pitch_values.trim -= TRIM_ADJUSTMENT_GRANULARITY;
  } else {
    if (selected_parameter == 0) {
      if (parameter_category == SENSING) {
        selected_parameter = 4;
      } else {
        selected_parameter = 3;
      }
    }
    selected_parameter = selected_parameter - 1;
    
    EEPROM.put(PARAMETER_SELECTION_EEPROM_ADDR, selected_parameter);
  }
}

void btn_up_pressed(bool down) {
  // Only on-press
  if (!down || !btn_up_debouncer.debounce()) {
    return;
  }

  if (adjust_trim) {
    which_trim = PITCH_TRIM;
    parameters.pitch_values.trim += TRIM_ADJUSTMENT_GRANULARITY;
  } else {
    selected_parameter = selected_parameter + 1;
    if (parameter_category == SENSING) {
      selected_parameter %= 4;
    } else {
      selected_parameter %= 3;
    }
    
    EEPROM.put(PARAMETER_SELECTION_EEPROM_ADDR, selected_parameter);
  }
}

void btn_center_pressed(bool down) {
  // Only on-press
  if (!down || !btn_center_debouncer.debounce()) {
    return;
  }

  // Toggle
  adjust_trim = !adjust_trim;
}

void knobs_update() {
  int knobPos = knob1.getCurrentPos();

  // We only care about movement, not its position
  knob1.setCurrentPos(0);

  // Disable knob while calibrating
  if (state == CALIBRATION) {
    return;
  }

  if (adjust_trim) {
    which_trim = YAW_TRIM;
    parameters.yaw_values.trim += TRIM_ADJUSTMENT_GRANULARITY * knobPos;
  } else if (parameter_category == SENSING) {
    switch (selected_parameter) {
      case COMPLEMENTARY:
        parameters.complementary = parameters.complementary + 0.01 * knobPos;
        break;
      case GYRO_HP:
        parameters.gyro_highpass = (parameters.gyro_highpass + knobPos) % 10;
        break;
      case XL_LP_RATIO:
        parameters.accel_lowpass_ratio = (parameters.accel_lowpass_ratio + knobPos) % 4;
        break;
    }
  } else {
    // Select loop
      PIDValues* pid_values;
      switch (parameter_category) {
        case PITCH_LOOP:
          pid_values = &(parameters.pitch_values);
          break;
        case YAW_LOOP:
          pid_values = &(parameters.yaw_values);
          break;
        case ROLL_LOOP:
          pid_values = &(parameters.roll_values);
          break;
      }
      
      // Modify specific parameter
      switch(selected_parameter) {
        case P:
          pid_values->p = max(0.0, pid_values->p + PID_ADJUSTMENT_GRANULARITY * knobPos);
          break;
        case I:
          pid_values->i = max(0.0, pid_values->i + PID_ADJUSTMENT_GRANULARITY * knobPos);
          break;
        case D:
          pid_values->d = max(0.0, pid_values->d + PID_ADJUSTMENT_GRANULARITY * knobPos);
          break;
      }
  }
}

// Save parameters
void knob_pressed(bool down) {
  // Only on-press
  if (!down || !knob_debouncer.debounce()) {
    return;
  }
  
  EEPROM.put(PARAMETER_EEPROM_ADDR, parameters);
}

/**
 * Get the raw gimbal values, into *_raw
 */
void read_gimbals() {
  throttle_raw = analogRead(PIN_THROTTLE);
  yaw_raw = analogRead(PIN_YAW);
  roll_raw = analogRead(PIN_ROLL);
  pitch_raw = analogRead(PIN_PITCH);
}

/**
 * Scale raw gimbal values and store to *_value
 */
void scale_gimbals() {
  throttle_value = constrain(map(throttle_raw, calibration.min_throttle, calibration.max_throttle, 0, 255), 0, 255);
  yaw_value = -asymmetric_map(yaw_raw, calibration.min_yaw, calibration.rest_yaw, calibration.max_yaw, DEADBAND, 127);
  roll_value = -asymmetric_map(roll_raw, calibration.min_roll, calibration.rest_roll, calibration.max_roll, DEADBAND, 127);
  pitch_value = -asymmetric_map(pitch_raw, calibration.min_pitch, calibration.rest_pitch, calibration.max_pitch, DEADBAND, 127);
}

/**
 * Check if the sticks are in the arming position, and arm if they are.
 */
void check_for_arm() {
  if (throttle_value == 0 && yaw_value == -127 && roll_value == -127 && pitch_value == -127) {
    state = ARMED;
  }
}

/**
 * Update the in-memory calibration values based on current gimbal values
 */
void do_calibration() {
  if (throttle_raw > calibration.max_throttle) {
    calibration.max_throttle = throttle_raw;
    last_calibration_update = millis();
  } else if (throttle_raw < calibration.min_throttle) {
    calibration.min_throttle = throttle_raw;
    last_calibration_update = millis();
  }
  
  if (yaw_raw > calibration.max_yaw) {
    calibration.max_yaw = yaw_raw;
    last_calibration_update = millis();
  } else if (yaw_raw < calibration.min_yaw) {
    calibration.min_yaw = yaw_raw;
    last_calibration_update = millis();
  }
  
  if (roll_raw > calibration.max_roll) {
    calibration.max_roll = roll_raw;
    last_calibration_update = millis();
  } else if (roll_raw < calibration.min_roll) {
    calibration.min_roll = roll_raw;
    last_calibration_update = millis();
  }
  
  if (pitch_raw > calibration.max_pitch) {
    calibration.max_pitch = pitch_raw;
    last_calibration_update = millis();
  } else if (pitch_raw < calibration.min_pitch) {
    calibration.min_pitch = pitch_raw;
    last_calibration_update = millis();
  }
}

/**
 * Send a packet to the quadcopter
 */
void send_packet() {
  byte parameter_ID = NO_PARAMETER;
  ParameterValue parameter_value;
  if (adjust_trim) {
    parameter_ID = which_trim;
    switch (which_trim) {
      case PITCH_TRIM:
        parameter_value.continuous_gain = parameters.pitch_values.trim;
        break;
      case ROLL_TRIM:
        parameter_value.continuous_gain = parameters.roll_values.trim;
        break;
      case YAW_TRIM:
        parameter_value.continuous_gain = parameters.yaw_values.trim;
        break;
    }
  } else {
    switch (parameter_category) {
      case SENSING:
        switch(selected_parameter) {
          case COMPLEMENTARY:
            parameter_ID = COMPLEMENTARY_GAIN;
            parameter_value.continuous_gain = parameters.complementary;
            break;
          case GYRO_HP:
            parameter_ID = GYRO_HIGHPASS;
            parameter_value.register_value = parameters.gyro_highpass;
            break;
          case XL_LP_RATIO:
            parameter_ID = XL_LOWPASS;
            parameter_value.register_value = parameters.accel_lowpass_ratio;
            break;
        }
        break;
      case PITCH_LOOP:
        switch(selected_parameter) {
          case P:
            parameter_ID = PITCH_P;
            parameter_value.continuous_gain = parameters.pitch_values.p;
            break;
          case I:
            parameter_ID = PITCH_I;
            parameter_value.continuous_gain = parameters.pitch_values.i;
            break;
          case D:
            parameter_ID = PITCH_D;
            parameter_value.continuous_gain = parameters.pitch_values.d;
            break;
        }
        break;
      case YAW_LOOP:
        switch(selected_parameter) {
          case P:
            parameter_ID = YAW_P;
            parameter_value.continuous_gain = parameters.yaw_values.p;
            break;
          case I:
            parameter_ID = YAW_I;
            parameter_value.continuous_gain = parameters.yaw_values.i;
            break;
          case D:
            parameter_ID = YAW_D;
            parameter_value.continuous_gain = parameters.yaw_values.d;
            break;
        }
        break;
      case ROLL_LOOP:
        switch(selected_parameter) {
          case P:
            parameter_ID = ROLL_P;
            parameter_value.continuous_gain = parameters.roll_values.p;
            break;
          case I:
            parameter_ID = ROLL_I;
            parameter_value.continuous_gain = parameters.roll_values.i;
            break;
          case D:
            parameter_ID = ROLL_D;
            parameter_value.continuous_gain = parameters.roll_values.d;
            break;
        }
        break;
    }
  }
  buildPacket(p, state == ARMED, single_motor_mode, motor_select, parameter_ID, parameter_value, throttle_value, pitch_value, roll_value, yaw_value);
  rfWrite((byte*) &p, sizeof(p));
  last_send_time = millis();
}

/**
 * Read a telemetry packet from the radio into active_telemetry, if one's been recieved
 */
void recieve_telemetry() {
  int len;
  
  len = rfAvailable();
  if (len >= sizeof(active_telemetry)) {
    rfRead(b, sizeof(active_telemetry));
    if (checkTelemetry(((Telemetry*) b)[0])) {
      active_telemetry = ((Telemetry*) b)[0];

      // Disarm if quad had been disarmed
      if (!isTelemetryArmed(active_telemetry) && state == ARMED) {
        state = DISARMED;
      }

      Serial.print(active_telemetry.pitch_err);
      Serial.print(" ");
      Serial.print(active_telemetry.roll_err);
      Serial.print(" ");
      Serial.println(active_telemetry.yaw_err);
      
      last_telemetry_time = millis();
    } else {
      // Clear buffer if we get a bad packet, to avoid misalignment
      rfFlush();
    }
  }
}

/**
 * Check if it's been too long since the remote recieved telemetry from the quad, and disarm if so
 */
void check_telemetry_timeout() {
  if (millis() - last_telemetry_time >= NO_TELEMETRY_DISARM_TIMEOUT && state == ARMED) {
    state = DISARMED;
    // Send packet immediately to broadcast the disarm
    send_packet();

    // Reset trims
    parameters.pitch_values.trim = 0;
    parameters.roll_values.trim = 0;
  }
}

/**
 * All code related to changing the LCD display
 */
void set_display() {
  // Calculate what the LCD should show based on the remote state
  strcpy(newFirstLine, "");
  strcpy(newSecondLine, "");
  unsigned long newColor = displayRGB;
  
  if (state == CALIBRATION) {
    newColor = 0x800080;   // Purple
    memcpy(newFirstLine, "Calibration", 12);
    if (millis() - last_calibration_update >= CALIBRATION_TIMEOUT_MS) {
      strcpy(newSecondLine, "complete");
    } else {
      strcpy(newSecondLine, "ongoing");
    }
  } else {
    if (state == ARMED) {
      newColor = 0xFF0000;     // Red
      strcpy(newFirstLine, "Throttle");
      strcpy(newSecondLine, String(throttle_value).c_str());
    } else if (state == DISARMED) {
      newColor = 0x0000FF;    // Blue
      strcpy(newFirstLine, "Disarmed");
      if (millis() - last_telemetry_time >= NO_TELEMETRY_DISARM_TIMEOUT) {
        strcpy(newSecondLine, "disconnected");
      }
    }

    if (adjust_trim) {
      strcpy(newFirstLine, "Ptch Roll Yaw");
      String trim_str = String(parameters.pitch_values.trim, 1);
      // Pad out to 5
      for (int i = trim_str.length(); i < 5; i++) {
        trim_str.concat(F(" "));
      }
      trim_str.concat(String(parameters.roll_values.trim, 1));
      // Pad out to 10
      for (int i = trim_str.length(); i < 10; i++) {
        trim_str.concat(F(" "));
      }
      trim_str.concat(String(parameters.yaw_values.trim, 1));
      strcpy(newSecondLine, trim_str.c_str());
    } else {
      if (parameter_category == SENSING) {
        switch (selected_parameter) {
          case COMPLEMENTARY:
            strcpy(newFirstLine, "Complementary");
            strcpy(newSecondLine, String(parameters.complementary, 2).c_str());
            break;
          case GYRO_HP:
            strcpy(newFirstLine, "Gyro highpass");
            // Fixed-width binary string encoding
            for (byte i = 0; i < 4; i++) {
              if (parameters.gyro_highpass & (1 << (3 - i))) {
                newSecondLine[i] = '1';
              } else {
                newSecondLine[i] = '0';
              }
            }
            newSecondLine[4] = 0;
            break;
          case XL_LP_RATIO:
            strcpy(newFirstLine, "XL LP cutoff");
            switch (parameters.accel_lowpass_ratio) {
              case 0:
                strcpy(newSecondLine, "ODR/50");
                break;
              case 1:
                strcpy(newSecondLine, "ODR/100");
                break;
              case 2:
                strcpy(newSecondLine, "ODR/9");
                break;
              case 3:
                strcpy(newSecondLine, "ODR/400");
                break;
            }
            break;
        }
      } else {
        // Select loop
        PIDValues* pid_values;
        switch (parameter_category) {
          case PITCH_LOOP:
            strcpy(newFirstLine, "Pitch ");
            pid_values = &(parameters.pitch_values);
            break;
          case YAW_LOOP:
            strcpy(newFirstLine, "Yaw ");
            pid_values = &(parameters.yaw_values);
            break;
          case ROLL_LOOP:
            strcpy(newFirstLine, "Roll ");
            pid_values = &(parameters.roll_values);
            break;
        }
        
        // Show specific parameter
        switch(selected_parameter) {
          case P:
            strcat(newFirstLine, "P");
            strcpy(newSecondLine, String(pid_values->p, 2).c_str());
            break;
          case I:
            strcat(newFirstLine, "I");
            strcpy(newSecondLine, String(pid_values->i, 2).c_str());
            break;
          case D:
            strcat(newFirstLine, "D");
            strcpy(newSecondLine, String(pid_values->d, 2).c_str());
            break;
        }
      }
    }
  
    if (single_motor_mode) {
      switch (motor_select) {
        case FRONT_LEFT:
          strcat(newFirstLine, " FL");
          break;
        case FRONT_RIGHT:
          strcat(newFirstLine, " FR");
          break;
        case BACK_LEFT:
          strcat(newFirstLine, " BL");
          break;
        case BACK_RIGHT:
          strcat(newFirstLine, " BR");
          break;
      }
    }
  }

  // Only call the LCD update functions if the display should change, since it gets overloaded (and drops messages) easily
  if (strncmp(newFirstLine, displayLine1, 16) != 0 || strncmp(newSecondLine, displayLine2, 16) != 0) {
    strcpy(displayLine1, newFirstLine);
    strcpy(displayLine2, newSecondLine);
    Serial.println(newFirstLine);
    Serial.println(newSecondLine);
    lcd.clear();
    lcd.print(newFirstLine);
    lcd.setCursor(0, 1);
    lcd.print(newSecondLine);
  }

//  Serial.print(throttle_value);
//  Serial.print(" ");
//  Serial.print(pitch_value);
//  Serial.print(" ");
//  Serial.print(roll_value);
//  Serial.print(" ");
//  Serial.println(yaw_value);

  if (newColor != displayRGB) {
    lcd.setBacklight(newColor);
    displayRGB = newColor;
  }
}

/**
 * like map(), but for an input that's asymmetric around a midpoint. If value is within +/- deadband of from_mid, maps it to 0. 
 * Otherwise, if it's below from_mid, linearly scales it from the range [from_min, from_mid - deadband] to [-output_max, 0]
 * If it's above from_mid, linearly scales it from the range [from_mid+deadband, from_max] to [0, output_max].
 * The overall function y = assymmetric_map(x, ...) looks like:
 * 
 *       /
 *    __/
 *   /
 *  /
 * 
 * Output is constrained to always be within [-output_max, output_max]
 */
int asymmetric_map(int value, int from_min, int from_mid, int from_max, int deadband, int output_max) {
  if (value > from_mid + deadband) {
    return constrain(map(value, from_mid + deadband, from_max, 0, output_max), 0, output_max);
  } else if (value < from_mid - deadband) {
    return constrain(map(value, from_min, from_mid - deadband, -output_max, 0), -output_max, 0);
  } else {
    return 0;
  }
}
