#include "Arduino.h"
#include "Logger.h"

Logger::Logger(char* name) {
  _name = name;
  _enable = true;
  logged_yet = false;
}

void Logger::setEnable(bool enable) {
  _enable = enable;
}

void Logger::log(const int value) {
  if (!_enable) {
    return;
  }
  
  if (logged_yet) {
    Serial.print(value);
    Serial.print(" ");
  } else {
    Serial.print(_name);
    Serial.print(" ");
    logged_yet = true;
  }
}

void Logger::log(const float value) {
  if (!_enable) {
    return;
  }
  
  if (logged_yet) {
    Serial.print(value);
    Serial.print(" ");
  } else {
    Serial.print(_name);
    Serial.print(" ");
    logged_yet = true;
  }
}

void Logger::log(const unsigned long value) {
  if (!_enable) {
    return;
  }
  
  if (logged_yet) {
    Serial.print(value);
    Serial.print(" ");
  } else {
    Serial.print(_name);
    Serial.print(" ");
    logged_yet = true;
  }
}
