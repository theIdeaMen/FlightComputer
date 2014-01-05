// Includes
#include <Thread.h>
#include <ThreadController.h>

#include <Wire.h>
#include <I2Cdev.h>
#include <VirtualWire.h>

#include <SerialCommand.h>

#include <PESO_Timer.h>

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <PESO_IMU.h>

#include <SdFat.h>
#include <PESO_Logger.h>

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <PESO_GPS.h>

#include <PESO_Trigger.h>

// Pin definitions
#define IMU_INTRPT    36
#define MICROSD_CS    33

#define CUTDOWN_PIN   5
#define CUT_EXPRMNT1  6
#define CUT_EXPRMNT2  7
#define CUT_EXPRMNT3  8

#define HEARTBEAT     10
#define BUZZER        34
#define COMMAND_LINK  9
#define RSSI_PIN      1    // Analog pin number

#define REMOVE_BEFORE_FLIGHT 33

// Data logging rate
#define SAMPLE_RATE 20      // Must be between 4 and 20 Hz


// Objects
Logger logger;    // Logs to microSD over SPI
IMU imu;          // Inertial measurement unit - MPU6050 breakout
GPS gps;          // Global positioning system - Adafruit GPS breakout

// Define threads
ThreadController Controller = ThreadController();
Thread IMU_thread = Thread();
Thread GPS_thread = Thread();
Thread COMMS_thread = Thread();
Thread CUTDOWN_thread = Thread();

void setup()
{
  Serial.begin(115200);
  
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
  IMU_thread.setInterval(10);
  
  GPS_thread.onRun(GPS_CB);
  GPS_thread.setInterval(100);
  
  COMMS_thread.onRun(COMMS_CB);
  COMMS_thread.setInterval(50);
  
  CUTDOWN_thread.onRun(CUTDOWN_CB);
  CUTDOWN_thread.setInterval(200);
  
  Controller.add(&IMU_thread);
  Controller.add(&GPS_thread);
  Controller.add(&COMMS_thread);
  Controller.add(&CUTDOWN_thread);

}

void loop()
{
  // Run threads
  Controller.run();
  
  // Check for command from ground control

}


// Thread callbacks

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
  
}

void COMMS_CB()
{
  
}

void CUTDOWN_CB()
{
  
}
