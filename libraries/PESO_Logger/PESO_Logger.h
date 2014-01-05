#ifndef __PESO_LOGGER__
#define __PESO_LOGGER__

#include <SdFat.h>
#include <stdint.h>

class Logger
{
  private:
    SdFat sd;
    ofstream logfile;
  
    char name[10];
    char buf[256];
    
    // config options
    bool echoOn;             // Echo log to Serial
    char callSign[10];        // The HAM call sign using the command link radio
    long topAltitude;         // The altitude at which there should be a cutdown in meters
  
  public:
    obufstream append;
    
    #define error(s) sd.errorHalt_P(PSTR(s))
    
    Logger();
  
    void echo();
    void initialize(uint8_t chip_select_pin, bool half_speed);
    void record();
    void recordln();  
};

#endif __PESO_LOGGER__