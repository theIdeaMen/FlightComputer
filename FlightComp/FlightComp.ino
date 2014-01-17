// Includes
#include <Thread.h>
#include <ThreadController.h>

#include <Wire.h>
#include <I2Cdev.h>
#include <VirtualWire.h>

//#include <SerialCommand.h>

#include <PESO_Timer.h>

#include <SdFat.h>
#include <PESO_Logger.h>

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <PESO_IMU.h>

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <PESO_GPS.h>

#include <PESO_Trigger.h>

// Pin definitions
#define IMU_INTRPT    36
#define MICROSD_CS    33

#define RSSI_PIN      1    // Analog pin number


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
  // Power settle delay
  delay(100);
  
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
  IMU_thread.setInterval(5);
  
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
  delay(1);
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
  // Get data
  gps.update();

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

void COMMS_CB()
{
  Serial.println("COMMS_CB: Begin");
}

void CUTDOWN_CB()
{
  Serial.println("CUTDOWN_CB: Begin");
}

