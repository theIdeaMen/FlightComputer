#ifndef __PESO_TIMER__
#define __PESO_TIMER__

#include <stdint.h>
#include <Arduino.h>

class Timer
{
  private:

    uint16_t _hz;
    uint32_t ms; 

  public:
  
    Timer(uint16_t hz);
    bool ready();
    uint32_t delta();
    void reset();
  
};

#endif __PESO_TIMER__