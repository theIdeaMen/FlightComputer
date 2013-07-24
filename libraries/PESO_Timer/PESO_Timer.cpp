#include <PESO_Timer.h>

Timer::Timer(uint16_t hz)
{
  _hz = hz;
  ms = 0;
}

bool Timer::ready()
{
  if ((millis() - ms) < 1000 / _hz) return false;
  ms = millis();
  
  return true;
}

uint32_t Timer::delta()
{
  return (millis() - ms);
}

void Timer::reset()
{
  ms = millis();
}