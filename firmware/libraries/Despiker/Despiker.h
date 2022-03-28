#ifndef Despiker_h
#define Despiker_h

#include "Arduino.h"

class Despiker {
  public:
    Despiker(float max_derivative);
    float despike(float input);
  private:
    float _max_derivative;
    bool last_was_spike;
    float last_input;
};
#endif
