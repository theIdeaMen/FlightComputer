#include <PESO_GPS.h>

GPS::GPS()
{
  hour = 1;
  minute = 1;
  seconds = 1;
  milliseconds = 1;
  day = 1;
  month = 1;
  year = 1;
  latitude = 1;
  lat = '1';
  longitude = 1;
  lon = '1';
  speed = 1;
  altitude = 1;
}

void GPS::initialize()
{
  data = Adafruit_GPS(&Serial1);
  
  data.begin(9600);
  
  data.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  data.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  
  // useInterrupt(true);
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

}

void GPS::update()
{
  if (data.newNMEAreceived()) data.parse(data.lastNMEA());

  if (data.fix)
  {
    hour = data.hour;
    minute = data.minute;
    seconds = data.seconds;
    milliseconds = data.milliseconds;
    day = data.day;
    month = data.month;
    year = data.year;
    latitude = data.latitude;
    lat = data.lat;
    longitude = data.longitude;
    lon = data.lon;
    speed = data.speed;
    altitude = data.altitude;
  }
}
