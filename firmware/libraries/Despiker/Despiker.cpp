#include "Arduino.h"
#include "Despiker.h"

Despiker::Despiker(float max_derivative) {
  _max_derivative = max_derivative;
  last_input = 0;
  last_was_spike = true;
}

float Despiker::despike(float input) {
  if (!last_was_spike && abs(input - last_input) > _max_derivative) {
    last_was_spike = true;
    return last_input;
  } else {
    last_was_spike = false;
    last_input = input;
    return input;
  }
}
