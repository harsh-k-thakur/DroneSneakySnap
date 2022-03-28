#include "Arduino.h"
#include "Debouncer.h"

Debouncer::Debouncer(int debounce_time) {
  _debounce_time = debounce_time;
  last_fired = 0;
}

bool Debouncer::debounce() {
  unsigned long now = millis();
  if (now - last_fired > _debounce_time) {
    last_fired = now;
    return true;
  }
  return false;
}
