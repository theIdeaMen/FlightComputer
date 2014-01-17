/*********************************************************************
  PESO Rockoon Flight Computer
  Arduino MEGA Firmware
  For use with V1.0 of the Flight Computer hardware

  Copyright (C) 2014 
  by Physics and Engineering Student Organization

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  <http://www.gnu.org/licenses/>.
**********************************************************************/
// Includes
#include <Thread.h>
#include <ThreadController.h>

#include <Wire.h>
#include <I2Cdev.h>

#include <SerialCommand.h>

#include <PESO_Timer.h>

#include <SdFat.h>
#include <PESO_Logger.h>

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <PESO_IMU.h>

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <PESO_GPS.h>

// Pin definitions
#define IMU_INTRPT    36
#define MICROSD_CS    33

#define CUTDOWN_PIN   2
#define SPARE_PIN     3
#define LED_PIN       5

#define RSSI_PIN      1    // Analog pin number

// Objects
Logger logger;    // Logs to microSD over SPI
IMU imu;          // Inertial measurement unit - MPU6050 breakout
GPS gps;          // Global positioning system - Adafruit GPS breakout

/* For when we get the transceiver
ArduinoOutStream Xceiver(Serial2);   // Serial connection to the high power transceiver
SerialCommand GROUND_cmdr(&Serial2); // Command and control from the ground
*/

ArduinoOutStream XBee(Serial3);      // Serial connection to the XBee radio
SerialCommand XBEE_cmdr(&Serial3);   // Command and control using XBee

// Define threads
ThreadController Controller = ThreadController();
Thread IMU_thread = Thread();
Thread GPS_thread = Thread();
Thread COMMS_thread = Thread();
Thread CUTDOWN_thread = Thread();
Thread RUN_LED_thread = Thread();

// Variables
unsigned char led_off_count = 0;
long max_altitude = 0;


void setup()
{
  // Power settle delay
  delay(100);
  
  pinMode(CUTDOWN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(115200);            // Debug interface
  Serial3.begin(9600);             // XBee interface

  logger.initialize(MICROSD_CS, false);
  logger.append << setprecision(6) << "PESO Flight Computer Log File\n";
  logger.append << "IMU,timestamp,temperature,q_w,q_x,q_y,q_z,aa_x,aa_y,aa_z\n";
  logger.append << "GPS,timestamp,time,date,lat,lon,speed,alt\n";
  logger.append << "COMMS,timestamp,rssi,message\n";
  logger.append << "CUTDOWN,timestamp,message";
  logger.echo();
  logger.recordln();

  imu.initialize();
  
  gps.initialize();
  
  // Configure threads
  IMU_thread.onRun(IMU_CB);
  IMU_thread.setInterval(5);
  
  GPS_thread.onRun(GPS_CB);
  GPS_thread.setInterval(1000);
  
  CUTDOWN_thread.onRun(CUTDOWN_CB);
  CUTDOWN_thread.setInterval(2000);
  
  RUN_LED_thread.onRun(RUN_LED_CB);
  RUN_LED_thread.setInterval(500);
  
  Controller.add(&IMU_thread);
  Controller.add(&GPS_thread);
  Controller.add(&CUTDOWN_thread);
  Controller.add(&RUN_LED_thread);

  // Configure ground control commands
  XBEE_cmdr.addCommand("GET",XBEE_GET_CMD);
  XBEE_cmdr.addCommand("SET",XBEE_SET_CMD);
  XBEE_cmdr.addCommand("?",XBEE_LIST_CMD);
  XBEE_cmdr.addDefaultHandler(XBEE_UNKNOWN_CMD);
}

void loop()
{
  // Run threads
  Controller.run();

  // Check for command from ground control
  XBEE_cmdr.readSerial();
}


/*****************
 Thread callbacks
******************/

void IMU_CB()
{
  if (digitalRead(IMU_INTRPT) == LOW)
    return;
 
  // Get data
  imu.update();
  
  // Append data type and time
  logger.append << "IMU," << imu.timestamp << ",";
  
  // Append temperature
  logger.append << (imu.temperature/(float)65536) << ",";
  
  // Append IMU data
  logger.append << (imu.q[0]/(float)1073741824) << "," << (imu.q[1]/(float)1073741824) << "," << (imu.q[2]/(float)1073741824) << "," << (imu.q[3]/(float)1073741824) << ",";
  logger.append << (imu.aa[0]/(float)imu.aa_sens) << "," << (imu.aa[1]/(float)imu.aa_sens) << "," << (imu.aa[2]/(float)imu.aa_sens);
  
  logger.echo();
  logger.recordln();
}
  
void GPS_CB()
{
  // Get data
  gps.update();
  
  // Save max altitude
  if ((long)gps.altitude > max_altitude) { max_altitude = (long)gps.altitude; }

  // Append data type and time
  logger.append << "GPS," << millis() << ",";
  
  // Append GPS data
  logger.append << setfill('0') << setw(2) << int(gps.hour) << ":";
  logger.append << setw(2) << int(gps.minute) << ":";
  logger.append << setw(2) << int(gps.seconds) << "." << setw(3) << int(gps.milliseconds) << ",";
  
  logger.append << setw(2) << int(gps.day) << "/";
  logger.append << setw(2) << int(gps.month) << "/20";
  logger.append << setw(2) << int(gps.year) << ",";
  
  logger.append << gps.latitude << ",";
  logger.append << gps.longitude << ",";
  logger.append << gps.speed << ",";
  logger.append << gps.altitude;
  
  logger.echo();
  logger.recordln();
}

void CUTDOWN_CB()
{
  digitalWrite(CUTDOWN_PIN, LOW);
  if (max_altitude > logger.topAltitude)
  {
    digitalWrite(CUTDOWN_PIN, HIGH);
    
    // Append cutdown message
    logger.append << "CUTDOWN," << millis() << ",";
    logger.append << "Altitude reached: " << max_altitude;
    
    logger.echo();
    logger.recordln();
  }
}

void RUN_LED_CB()
{
  digitalWrite(LED_PIN, LOW);
  if (led_off_count++ > 5)
  {
    digitalWrite(LED_PIN, HIGH);
    led_off_count = 0;
  }
}


/*************************
 Serial command callbacks
**************************/

void XBEE_GET_CMD()
{
  char *arg = XBEE_cmdr.next();

  if (strcmp(arg, "IMU") == 0)
  {
    print_imu(XBee);
  }

  if (strcmp(arg, "GPS") == 0)
  {
    print_gps(XBee);
  }

  if (strcmp(arg, "ECHO") == 0)
  {
    XBee << "Echo is " << (logger.echoOn ? "on":"off") << endl;
  }

  if (strcmp(arg, "CLSGN") == 0)
  {
    XBee << "Call sign is " << logger.callSign << endl;
  }

  if (strcmp(arg, "TALT") == 0)
  {
    XBee << "Cutdown altitude is " << logger.topAltitude << endl;
  }
}

void XBEE_SET_CMD()
{
  char *arg = XBEE_cmdr.next();

  // Set weather the logger should echo to Serial
  if (strcmp(arg, "ECHO") == 0)
  {
    arg = XBEE_cmdr.next();
    if (strcmp(arg, "OFF") == 0)
      logger.echoOn = false;
    else if (strcmp(arg, "ON") == 0)
      logger.echoOn = true;
    XBee << "Echo is " << (logger.echoOn ? "on":"off") << endl;
  }

  // Set the call sign for using the high power transceiver
  if (strcmp(arg, "CLSGN") == 0)
  {
    logger.callSign = XBEE_cmdr.next();
    XBee << "Call sign is " << logger.callSign << endl;
  }

  // Set the altitude at which a cutdown will be issued
  if (strcmp(arg, "TALT") == 0)
  {
    logger.topAltitude = atoi(XBEE_cmdr.next());
    XBee << "Cutdown altitude is " << logger.topAltitude << endl;
  }
}

void XBEE_LIST_CMD()
{
  XBee << "List of commands:\n";
  XBee << "GET [IMU|GPS|ECHO|CLSGN|TALT]\n";
  XBee << "SET [ECHO|CLSGN|TALT] {value}\n";
  XBee << "Please use uppercase\n";
}

void XBEE_UNKNOWN_CMD()
{
  XBee << "Unknown command\n";
  XBee << "Send \"?\" for a list of commands\n";
}


/*************************
 Utility functions
**************************/

void print_imu(ArduinoOutStream os)
{
  os << "IMU at time " << imu.timestamp << endl;
  os << "TEMP: " << (imu.temperature/(float)65536) << endl;
  os << "QUAT: w=" << (imu.q[0]/(float)1073741824);
  os << " x=" << (imu.q[1]/(float)1073741824);
  os << " y=" << (imu.q[2]/(float)1073741824);
  os << " z=" << (imu.q[3]/(float)1073741824) << endl;
  os << "ACCL: x=" << (imu.aa[0]/(float)imu.aa_sens);
  os << " y=" << (imu.aa[1]/(float)imu.aa_sens);
  os << " z=" << (imu.aa[2]/(float)imu.aa_sens) << endl;
}

void print_gps(ArduinoOutStream os)
{
  os << "GPS at time ";
  os << setfill('0') << setw(2) << int(gps.hour) << ":";
  os << setw(2) << int(gps.minute) << ":" << int(gps.seconds);
  os << "." << setw(3) << int(gps.milliseconds) << " ";
  os << setw(2) << int(gps.day) << "/";
  os << setw(2) << int(gps.month) << "/20";
  os << setw(2) << int(gps.year) << endl;
  os << "LAT: " << gps.latitude << endl;
  os << "LON: " << gps.longitude << endl;
  os << "SPD: " << gps.speed << endl;
  os << "ALT: " << gps.altitude << endl;
}
