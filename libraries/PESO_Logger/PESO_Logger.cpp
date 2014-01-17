#include "PESO_Logger.h"

Logger::Logger()
{
  // Initialize config options
  echoOn = true;
  callSign = "KD0UFY";
  topAltitude = 29000;
}

// Looks for a config file then opens a new log file
void Logger::initialize(uint8_t chip_select_pin, bool half_speed)
{
  // initialize sd object
  uint8_t spi_speed = SPI_FULL_SPEED;
  if (half_speed) spi_speed = SPI_HALF_SPEED;

  if (!sd.begin(chip_select_pin, spi_speed)) sd.initErrorHalt();

  // Check SD card for the config file
  sprintf(name, "config.txt");
  if (sd.exists(name))
  {
    configFile.open(name);
    if (configFile.is_open())
    {
      readConfigFile();
      configFile.close();
    }
  }

  // select next available log name
  sprintf(name, "log00.csv");
  for (uint8_t i = 0; i < 100; i++)
  {
    name[3] = i/10 + '0';
    name[4] = i%10 + '0';
    
    if (sd.exists(name)) continue;
    logfile.open(name);
    break;
  }
  
  if (!logfile.is_open()) error("Unable to open file.");
  
  // create text buffer stream
  append = obufstream(buf, sizeof(buf));
}

void Logger::echo()
{
  if (echoOn)
    Serial.println(buf);
}

void Logger::record()
{
  // append text buffer to the file
  logfile << buf << flush;
  append.seekp(0);
}

void Logger::recordln()
{
  // append text buffer, plus new line
  logfile << buf << endl << flush;
  append.seekp(0);
}

void Logger::readConfigFile()
{
  char line[20];
  char prop[20];
  char val[20];

  while (!configFile.eof())
  {
    configFile.getline(line, 20);

    sscanf(line, "%s %s", prop, val);
    if (strcmp(prop, "ECHO") == 0)
    {
      if (strcmp(val, "OFF") == 0)
      {
        echoOn = false;
      }
    }

    if (strcmp(prop, "CALLSIGN") == 0)
    {
      callSign = val;
    }

    if (strcmp(prop, "TOPALT") == 0)
    {
      topAltitude = atoi(val);
    }
  }
}