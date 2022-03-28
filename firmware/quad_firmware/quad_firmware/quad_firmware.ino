#include <radio.h>
#include <CommPacket.h>
#include <Logger.h>
#include <Despiker.h>
#include <EEPROM.h>

// Libraries required for getting the data from the IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>

// Defining the constants used by the LSM9DS1 (IMU)
#define FDS (1 << 2)

//Constants
#define FRONT_LEFT_MOTOR_PIN 4
#define BACK_RIGHT_MOTOR_PIN 5
#define BACK_LEFT_MOTOR_PIN 3
#define FRONT_RIGHT_MOTOR_PIN 8

const unsigned long TELEMETRY_PERIOD_MS = 100;
const unsigned long NO_PACKET_DISARM_TIMEOUT = 600;

// EVERYTHING CLOCKWISE POSITIVE
// LIKE FOR REAL EVERYTHING
// CHECK!
// Pitch axis: look down the stick with battery connector on the right (i.e. look from the quadcopter's right)
// Roll axis: look from the front
// Yaw axis: look from the top
const int PITCH_RATE_GYRO_SIGN = 1;
const int PITCH_ACCEL_SIGN = 1;
const int ROLL_RATE_GYRO_SIGN = 1;
const int ROLL_ACCEL_SIGN = 1;
const int YAW_RATE_SIGN = 1;

// For calibration
const int CALIBRATION_SAMPLES = 200;
const int CALIBRATION_DISCARD_SAMPLES = 100;
const int CALIBRATION_PERIOD_MS = 4;

const float GYRO_SPIKE_SIZE = 16;
const float MAX_ERROR_INTEGRAL = 100;

// Creating an instance lsm for communicating with the IMU using I2C
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

// Memory for the offsets
float pitch_accel_offset = 0.0;
float pitch_rate_gyro_offset = 0.0;
float roll_accel_offset = 0.0;
float roll_rate_gyro_offset = 0.0;
float yaw_rate_gyro_offset = 0.0;

// Memory for the estimated data and combination
float pitch = 0.00;
float roll = 0.00;
float yaw_rate = 0.00;

// Despikers
Despiker pitch_despiker = Despiker(GYRO_SPIKE_SIZE);
Despiker roll_despiker = Despiker(GYRO_SPIKE_SIZE);
Despiker yaw_despiker = Despiker(GYRO_SPIKE_SIZE);

// Memory for the PID loop
struct PIDMemory {
  float last_error;
  float error_integral;
};
PIDMemory pitch_memory;
PIDMemory yaw_memory;
PIDMemory roll_memory;

// Parameter values
ParameterValues parameters;

// The most recently recieved valid packet
Packet active_packet;

struct MotorOutputs {
  int front_left;
  int front_right;
  int back_left;
  int back_right;
};

const MotorOutputs OFF = {0, 0, 0, 0};

// Whether the quad is currently armed
bool armed;

// Last time a telemetry packet was sent
unsigned long last_telemetry_time = 0;
// Last time a packet was recieved from the remote
unsigned long last_packet_time = 0;
// Last time we read the data from IMU
unsigned long last_imu_time = 0;
// Last time we ran PID
unsigned long last_PID_time = 0;

// Logging
Logger dt_logger = Logger("time");

Logger yaw_rate_logger = Logger("yaw_rate");
Logger pitch_logger = Logger("pitch");
Logger roll_logger = Logger("roll");

Logger pitch_term_logger = Logger("pitch_term");
Logger roll_term_logger = Logger("roll_term");
Logger yaw_term_logger = Logger("yaw_term");

Logger pitch_integral_logger = Logger("pitch_integral");
Logger roll_integral_logger = Logger("roll_integral");
Logger yaw_integral_logger = Logger("yaw_integral");

Logger pitch_error_logger = Logger("pitch_error");
Logger roll_error_logger = Logger("roll_error");
Logger yaw_error_logger = Logger("yaw_error");

Logger pitch_derivative_logger = Logger("pitch_derivative");
Logger roll_derivative_logger = Logger("roll_derivative");
Logger yaw_derivative_logger = Logger("yaw_derivative");

Logger pitch_gyro_logger = Logger("pitch_rate_gyro");
Logger raw_pitch_gyro_logger = Logger("raw_pitch_gyro");
Logger pitch_accel_logger = Logger("pitch_accel");
Logger pitch_rate_offset_logger = Logger("pitch_rate_gyro_offset");

Logger roll_gyro_logger = Logger("roll_rate_gyro");
Logger roll_accel_logger = Logger("roll_accel");

Logger raw_yaw_gyro_logger = Logger("raw_yaw_gyro");

Logger throttle_overconstrain_logger = Logger("throttle_overconstrain");
Logger throttle_logger = Logger("throttle");

float pitch_err = 0.0;
float roll_err = 0.0;
float yaw_err = 0.0;

// EEPROM addresses
const int PARAMETER_EEPROM_ADDR = 0;

void setup() {
  const int SERIAL_BAUD = 19200;       // Baud rate for serial port
  Serial.begin(SERIAL_BAUD);           // Start up serial
  pinMode(LED_BUILTIN, OUTPUT);
  rfBegin(12);
  
  run_motors(OFF,false,0);
  armed = false;
  EEPROM.get(PARAMETER_EEPROM_ADDR, parameters);
  

  // Reset trims
//  parameters.pitch_values.trim = 0;
//  parameters.roll_values.trim = 0;
//  parameters.yaw_values.trim = 0;
  
  begin_i2c();                        // This will begin the communication between IMU and Arduino  
  setup_imu();                        // This is used to setup the IMU

  create_offset();
  
  send_telemetry();

  dt_logger.setEnable(false);

  const bool LOG_PITCH = false;
  const bool LOG_ROLL = false;
  const bool LOG_YAW = false;
  
  pitch_logger.setEnable(LOG_PITCH);
  roll_logger.setEnable(LOG_ROLL);
  yaw_rate_logger.setEnable(LOG_YAW);

  
  pitch_term_logger.setEnable(LOG_PITCH);
  roll_term_logger.setEnable(LOG_ROLL);
  yaw_term_logger.setEnable(LOG_YAW);

  pitch_integral_logger.setEnable(LOG_PITCH);
  roll_integral_logger.setEnable(LOG_ROLL);
  yaw_integral_logger.setEnable(LOG_YAW);
  
  pitch_error_logger.setEnable(LOG_PITCH);
  roll_error_logger.setEnable(LOG_ROLL);
  yaw_error_logger.setEnable(LOG_YAW);
  
  pitch_derivative_logger.setEnable(LOG_PITCH);
  roll_derivative_logger.setEnable(LOG_ROLL);
  yaw_derivative_logger.setEnable(LOG_YAW);
  
  
  const bool LOG_THROTTLE = false;
  throttle_overconstrain_logger.setEnable(LOG_THROTTLE);
  throttle_logger.setEnable(LOG_THROTTLE);

  pitch_gyro_logger.setEnable(false);
  pitch_accel_logger.setEnable(false);
  raw_pitch_gyro_logger.setEnable(false);
  pitch_rate_offset_logger.setEnable(false);

  roll_accel_logger.setEnable(false);
  roll_gyro_logger.setEnable(false);

  raw_yaw_gyro_logger.setEnable(false);
  
  last_PID_time = millis();
}


void loop() 
{
  // Clear for logging
  Serial.println("");
  
  read_radio();
  check_packet_timeout();
  send_telemetry_timed();
  read_imu_data();

  MotorOutputs motor_outputs;
  calc_PID(motor_outputs);
//  motor_outputs = {active_packet.throttle, active_packet.throttle, active_packet.throttle, active_packet.throttle};

  if (armed && active_packet.throttle != 0) {
    run_motors(motor_outputs, isSingleMotor(active_packet), getSelectedMotor(active_packet));
  } else {
    run_motors(OFF, false, 0);
  }
}

void begin_i2c()
{
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring or I2C ADDR!"));
    while(1);
    delay(10);
  }
}

/*
 * This is the function to set up the IMU
 * This will help us how we want our IMU to work.
 * What parts of IMU data we want to gets
 * Like the mangnetameter and gyroscope
 */
void setup_imu()
{
  const byte ODR = ODR_476;
  const byte G_HP_D = B00000000;
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  // LSM9DS1_REGISTER_CTRL_REG1_G         = 0x10,
  // This gives us the value that we are using the CTRL_REG1_G(10h)
  // The ODR frequency of 238hz ODR + 63Hz cuttof
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G, ODR | G_BW_G_00 );
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG2_G, G_OUTSEL_HP_LP);
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_G, G_HP_D);
//  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_G, G_HP_EN | parameters.gyro_highpass);


  // Enable the XL (Section 7.23)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set XL ODR
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG6_XL, ODR);

  // Set low-pass XL filter frequency divider (Section 7.25)
  // HR_MODE will turn on high resolution mode  
  // The High Pass Filter in LSM9DS1_REGISTER_CTRL_REG7_XL is already turned on
  // LSM9DS1_REGISTER_CTRL_REG7_XL come with FDS=0 but we turn it on using BW_SCAL_ODR.
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | parameters.accel_lowpass_ratio << 5);
  
  // enable mag continuous (Section 8.7)
  // We don't need magnetic sensor so we turn it off by doing B00000010 or B00000011
  lsm.write8(MAGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_M, B00000011);

  // This only sets range of measurable values for each sensor.  Setting these manually (I.e., without using these functions) will cause incorrect output from the library.
  lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_2000DPS);
}

/*
 * Use this function to as a complementary filter
 * Will update according to the message received from the IC
 */
float complementary_filter(float filtered, float accel_heading, float gyro_heading_rate, float delta_time_seconds){
  // The complementary filters uses the weights
  // This helps us in defining how much importance should be given.
  // To gyroscope and accelerometer
  return (parameters.complementary)*(filtered + (delta_time_seconds*gyro_heading_rate)) + (1.0 - parameters.complementary) * accel_heading;
}

/**
 * Read a new packet from the radio buffer into active_packet, if one's arrived
 */
void read_radio() {
  int len;
  uint8_t b[sizeof(active_packet)];
  
  len = rfAvailable();
  if (len >= sizeof(active_packet)) {
    rfRead(b, sizeof(active_packet));
    if (checkPacket(((Packet*) b)[0])) {
      active_packet = ((Packet*) b)[0];
      armed = isArmed(active_packet);
      last_packet_time = millis();
      setParametersFromPacket(active_packet, parameters);
      if (getSelectedParameter(active_packet) == XL_LOWPASS) {
        lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | parameters.accel_lowpass_ratio << 5);
      } //else if (getSelectedParameter(active_packet) == GYRO_HIGHPASS) {
        //lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_G, G_HP_EN | parameters.gyro_highpass);
      //}
    } else {
      // Clear buffer if we get a bad packet, to avoid misalignment
      rfFlush();
    }
  }
}

/**
 * Check if it's been too long since the quad recieved a packet from the remote, and disarm if so
 */
void check_packet_timeout() {
  if (millis() - last_packet_time >= NO_PACKET_DISARM_TIMEOUT) {
    armed = false;
    EEPROM.put(PARAMETER_EEPROM_ADDR, parameters);
    // Send telemetry immediately to broadcast the disarm
    send_telemetry();
  }
}

/*
 * This particular function of the code is useful for creating the offset
 * Assuming we start every flight from a flat surface
 * If this is not the case, we will hard code the offsets in our code.
 * The values are as follows:-
 * acc_pitch_offset = 1.16;
 * gyro_pitch_rate_offset = 2.62;
 */
void create_offset() {
  quad_data_t orientation;
  int samples = 0;

  float pitch_accel_sum = 0;
  float pitch_gyro_sum = 0;
  float roll_accel_sum = 0;
  float roll_gyro_sum = 0;
  float yaw_gyro_sum = 0;

  while (samples < CALIBRATION_DISCARD_SAMPLES) {
      if (ahrs.getQuadOrientation(&orientation)) {
        samples += 1;
      }
      delay(CALIBRATION_PERIOD_MS);
  }

  samples = 0;
  
  while (samples < CALIBRATION_SAMPLES) {
      if (ahrs.getQuadOrientation(&orientation)) {
//        pitch_accel_sum += orientation.roll;
//        pitch_gyro_sum += pitch_despiker.despike(orientation.roll_rate);
//        roll_accel_sum += orientation.pitch;
//        roll_gyro_sum += roll_despiker.despike(orientation.pitch_rate);
//        yaw_gyro_sum += yaw_despiker.despike(orientation.yaw_rate);
        samples += 1;
      }
      delay(CALIBRATION_PERIOD_MS);
  }
  
  pitch_accel_offset = pitch_accel_sum / CALIBRATION_SAMPLES;
  pitch_rate_gyro_offset = pitch_gyro_sum / CALIBRATION_SAMPLES;
//  pitch_rate_gyro_offset = 0;
  roll_accel_offset = roll_accel_sum / CALIBRATION_SAMPLES;
  roll_rate_gyro_offset = roll_gyro_sum / CALIBRATION_SAMPLES;
//  roll_rate_gyro_offset = 0;
  yaw_rate_gyro_offset = yaw_gyro_sum / CALIBRATION_SAMPLES;
//  yaw_rate_gyro_offset = 0;
//  Serial.print("ACC PITCH = "); Serial.println(pitch_accel_offset);
//  Serial.print("GYRO PITCH = "); Serial.println(pitch_rate_gyro_offset);
//  Serial.print("ACC ROLL = "); Serial.println(roll_accel_offset);
//  Serial.print("GYRO ROLL = "); Serial.println(roll_rate_gyro_offset);
//  Serial.print("GYRO YAW = "); Serial.println(yaw_rate_gyro_offset);
//  delay(1000);
}

void read_imu_data() {
  quad_data_t orientation;
  unsigned long current_imu_time = millis();
  /* 'orientation' should have valid .roll and .pitch fields */
  if (ahrs.getQuadOrientation(&orientation)) {
    
    float pitch_accel = (orientation.roll - pitch_accel_offset) * PITCH_ACCEL_SIGN;
    float roll_accel = (orientation.pitch - roll_accel_offset) * ROLL_ACCEL_SIGN;
    
    float pitch_rate_gyro = (orientation.roll_rate - pitch_rate_gyro_offset) * PITCH_RATE_GYRO_SIGN;
    float roll_rate_gyro = (orientation.pitch_rate - roll_rate_gyro_offset) * ROLL_RATE_GYRO_SIGN;
    yaw_rate = (orientation.yaw_rate - yaw_rate_gyro_offset) * YAW_RATE_SIGN;

    
    const float imu_read_time_seconds = (current_imu_time - last_imu_time) / 1000.0;
    pitch = complementary_filter(pitch, pitch_accel, pitch_rate_gyro, imu_read_time_seconds);
    roll = complementary_filter(roll, roll_accel, roll_rate_gyro, imu_read_time_seconds);

    dt_logger.log(current_imu_time - last_imu_time);
    yaw_rate_logger.log(yaw_rate);
    pitch_logger.log(pitch);
    roll_logger.log(roll);

    raw_pitch_gyro_logger.log(orientation.pitch_rate);
    raw_yaw_gyro_logger.log(orientation.yaw_rate);
    
    pitch_gyro_logger.log(pitch_rate_gyro);
    pitch_accel_logger.log(pitch_accel);
    pitch_rate_offset_logger.log(pitch_rate_gyro_offset);

    roll_accel_logger.log(roll_accel);
    roll_gyro_logger.log(roll_rate_gyro);
    
    last_imu_time = current_imu_time;
  }
}

/**
 * Send telemetry only if enough time has elapsed since the last telemetry packet
 */
void send_telemetry_timed(){
  if (millis() - last_telemetry_time >= TELEMETRY_PERIOD_MS) {
    send_telemetry();
  }
}


/**
 * Send telemetry data back to the remote
 */
void send_telemetry() {
  Telemetry telemetry;
  buildTelemetry(telemetry, armed, pitch_err, roll_err, yaw_err);
  rfWrite((byte*) &telemetry, sizeof(telemetry));
  last_telemetry_time = millis();
}


void calc_PID(MotorOutputs& motor_outputs) {
  unsigned long PID_time = millis();

  const float loop_time_seconds = (PID_time - last_PID_time) / 1000.0;

  // Clear windup
  if (active_packet.throttle == 0 ) {
    pitch_memory.error_integral = 0;
    yaw_memory.error_integral = 0;
    roll_memory.error_integral = 0;
  }
  // const float pitch_term = 0;
  // const float yaw_term = 0;
  // const float roll_term = 0;
  
  const float pitch_term = pure_PID(pitch, (active_packet.pitch + 0.0) / 5.35, loop_time_seconds, pitch_memory, parameters.pitch_values, pitch_err, pitch_derivative_logger);
  const float yaw_term = pure_PID(yaw_rate, (active_packet.yaw + 0.0)/0.7, loop_time_seconds, yaw_memory, parameters.yaw_values, yaw_err, yaw_derivative_logger);
  const float roll_term = pure_PID(roll, (active_packet.roll + 0.0)/5.35, loop_time_seconds, roll_memory, parameters.roll_values, roll_err, roll_derivative_logger);

  pitch_error_logger.log(pitch_err);
  roll_error_logger.log(roll_err);
  yaw_error_logger.log(yaw_err);
  // Serial.println();
  // Serial.print("Pitch: "); Serial.println(pitch_memory.error_integral);
  // Serial.print("Yaw: "); Serial.println(yaw_term);

  const int front_left_term = -pitch_term - yaw_term - roll_term;
  const int front_right_term = -pitch_term + yaw_term + roll_term;
  const int back_left_term = pitch_term + yaw_term - roll_term;
  const int back_right_term = pitch_term -  yaw_term + roll_term;

  const int max_throttle = 255 - max(max(max(max(0, front_left_term), front_right_term), back_left_term), back_right_term);
  const int min_throttle = -min(min(min(min(0, front_left_term), front_right_term), back_left_term), back_right_term);
  int mixed_throttle = 0;

  if (min_throttle > max_throttle) {
    // Average if the PID outputs require impossible throttle
    mixed_throttle = (min_throttle + max_throttle) / 2;
    throttle_overconstrain_logger.log(5);
  } else {
    mixed_throttle = map(active_packet.throttle, 0, 255, min_throttle, max_throttle);
    throttle_overconstrain_logger.log(0);
  }
  
  motor_outputs.front_left = constrain(mixed_throttle + front_left_term, 0, 255);
  motor_outputs.front_right = constrain(mixed_throttle + front_right_term, 0, 255);
  motor_outputs.back_left = constrain(mixed_throttle + back_left_term, 0, 255);
  motor_outputs.back_right = constrain(mixed_throttle + back_right_term, 0, 255);
  
  pitch_term_logger.log(pitch_term);
  roll_term_logger.log(roll_term);
  yaw_term_logger.log(yaw_term);
  throttle_logger.log(mixed_throttle);
  
  pitch_integral_logger.log(pitch_memory.error_integral);
  roll_integral_logger.log(roll_memory.error_integral);
  yaw_integral_logger.log(yaw_memory.error_integral);

  last_PID_time = PID_time;
}

/**
 * Calculate output from a PID loop. This is almost a "pure" function - the only side effect is modifying the PID memory.
 */
float pure_PID(float value, float setpoint, float delta_time, PIDMemory& memory, const PIDValues& gains, float& error, Logger& derivative_logger) {
  /*
  const float error = value - gains.trim - setpoint;
  float error_derivative = 0;
  if (abs(error) >= 1)
  {
    if(abs(error) > abs(memory.last_error))
    {
      memory.error_integral += error * delta_time;
    }
    error_derivative = (error - memory.last_error) / delta_time;
    memory.error_integral = min(memory.error_integral, 100);
  } else {
    error_derivative = 0;
    memory.error_integral = 0;
  }
  memory.last_error = error;
  */
  error = setpoint - (value - gains.trim);
  // LOGGING STATEMENT
//  telemetry_log_value = error;
  const float error_derivative = (error - memory.last_error) / delta_time;
  derivative_logger.log(error_derivative);
//  if (-3 < error < 3) {
  memory.error_integral += error * delta_time;
  memory.error_integral = constrain(memory.error_integral, -MAX_ERROR_INTEGRAL, MAX_ERROR_INTEGRAL);
//  }
  memory.last_error = error;
  
  return gains.p * error + gains.i * memory.error_integral + gains.d * error_derivative;
}

/**
 * Set the diplay LEDs on the quadcopter according to the current state
 */
void set_LEDs() {
  digitalWrite(LED_BUILTIN, armed ? HIGH : LOW);
}

/**
 * Run each of the motors at the specified speeds
 */
void run_motors(MotorOutputs outputs, bool single_motor, byte motor_select) {
  if (!single_motor || motor_select == FRONT_LEFT) {
    analogWrite(FRONT_LEFT_MOTOR_PIN, outputs.front_left);
  } else {
    analogWrite(FRONT_LEFT_MOTOR_PIN, 0);
  }
  if (!single_motor || motor_select == FRONT_RIGHT) {
    analogWrite(FRONT_RIGHT_MOTOR_PIN, outputs.front_right);
  } else {
    analogWrite(FRONT_RIGHT_MOTOR_PIN, 0);
  }
  if (!single_motor || motor_select == BACK_LEFT) {
    analogWrite(BACK_LEFT_MOTOR_PIN, outputs.back_left);
  } else {
    analogWrite(BACK_LEFT_MOTOR_PIN, 0);
  }
  if (!single_motor || motor_select == BACK_RIGHT) {
    analogWrite(BACK_RIGHT_MOTOR_PIN, outputs.back_right);
  } else {
    analogWrite(BACK_RIGHT_MOTOR_PIN, 0);
  }
}
