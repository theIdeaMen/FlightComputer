#include <PESO_GPS.h>

GPS::GPS()
{
  hour = 0;
  minute = 0;
  seconds = 0;
  milliseconds = 0;
  day = 0;
  month = 0;
  year = 0;
  latitude = 0;
  lat = '0';
  longitude = 0;
  lon = '0';
  speed = -1;
  altitude = -1;
}

void GPS::initialize()
{
  data = Adafruit_GPS(&Serial1);
  
  data.begin(9600);
  
  data.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  data.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
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
