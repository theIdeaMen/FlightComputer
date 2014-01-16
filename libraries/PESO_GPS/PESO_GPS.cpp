#include <PESO_GPS.h>

GPS::GPS()
{

}

void GPS::initialize()
{
  data = Adafruit_GPS(&Serial1);
  
  data.begin(9600);
  
  data.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  data.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
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
    
    latitude = coord2decimal(latitude, lat);
    longitude = coord2decimal(longitude, lon);
  }
}

float GPS::coord2decimal(float coord, char dir)
{
  int degrees = coord / 100;
  float minutes = fmod(coord,100) / 60;
  float newCoord = float(degrees) + minutes;
  
  if (dir == 'S' || dir == 'W')
    newCoord = newCoord * (-1);
    
  return newCoord;
}