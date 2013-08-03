#ifndef __PESO_GPS__
#define __PESO_GPS__

#include "../Adafruit_GPS/Adafruit_GPS.h"
#include <math.h>

class GPS
{
  public:
  
  Adafruit_GPS data;

  uint8_t hour, minute, seconds, year, month, day;
  uint16_t milliseconds;
  float latitude, longitude, altitude;
  float speed;
  bool fix;
  
  GPS();
  void initialize();
  void update();
  
  private:
  
  char lat, lon;
  float coord2decimal(float coord, char dir);
  
};

#endif __PESO_GPS__