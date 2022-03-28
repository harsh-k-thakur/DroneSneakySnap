#ifndef Logger_h
#define Logger_h

#include "Arduino.h"

class Logger {
  public:
    Logger(char* name);
    void setEnable(bool enable);
    void log(const int value);
    void log(const float value);
    void log(const unsigned long value);
  private:
    bool logged_yet;
    bool _enable;
    char* _name;
};
#endif
