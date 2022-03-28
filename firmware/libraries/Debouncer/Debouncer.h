#ifndef Debouncer_h
#define Debouncer_h

#include "Arduino.h"

class Debouncer {
  public:
    Debouncer(int debounce_time);
    bool debounce();
  private:
    int _debounce_time;
    unsigned long last_fired;
};
#endif
