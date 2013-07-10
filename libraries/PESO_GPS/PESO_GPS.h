#ifndef __PESO_GPS__
#define __PESO_GPS__

#include "../Adafruit_GPS/Adafruit_GPS.h"

class GPS
{
  public:
  
  Adafruit_GPS data;

  uint8_t hour, minute, seconds, year, month, day;
  uint16_t milliseconds;
  float latitude, longitude, altitude;
  float speed;
  char lat, lon;
  
  GPS();
  void initialize();
  void update();
  
};

#endif __PESO_GPS__