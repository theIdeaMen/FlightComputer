#include <Wire.h>
#include <I2Cdev.h>

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
#define TRIGGER_PIN 5

Logger logger;
Timer timer(70);

IMU imu;
void imuInterrupt() { imu.interrupt(); } // IMU interrupt


GPS gps;
SIGNAL(TIMER0_COMPA_vect) { char c = gps.data.read(); } // GPS interrupt

Trigger cutter;
#define TRIGGER_PIN 5

void cutCallBack()
{
  logger.append << (millis() * .001) << "\tCutdown!";
  logger.echo();
  logger.recordln();
}

void stopCallBack()
{
  cutter.disable();
  logger.append << (millis() * .001) << "\tCutdown Secured!";
  logger.echo();
  logger.recordln();
}


void setup()
{
  Serial.begin(115200);
  
  Wire.begin();
  
  while(imu.initialize(imuInterrupt))
    delay(100);
  
  logger.initialize(3, false); // Chip Select Pin 3, Full Speed
  
  gps.initialize();
  
  logger.append << "timestamp,q_w,q_x,q_y,q_z,aa_x,aa_y,aa_z,gyro_x,gyro_y,gyro_z,time,date,lat,lon,speed,alt";
  logger.echo();
  logger.recordln();
  
  cutter.initialize(TRIGGER_PIN, 20000, Trigger::ABOVE, 25000, Trigger::ABOVE);
  cutter.onCallBack(&cutCallBack);
  cutter.offCallBack(&stopCallBack);
}

void loop()
{
  
  imu.update();
  gps.update();

  // Wait for set delay
  if (!timer.ready()) return;
  
  //cutter.update(millis());

  // Append time since last update
  logger.append << imu.timestamp << ",";
  
  // Append IMU data
  logger.append << imu.q[0] << "," << imu.q[1] << "," << imu.q[2] << "," << imu.q[3] << ",";
  logger.append << imu.aa[0] << "," << imu.aa[1] << "," << imu.aa[2] << ",";
  logger.append << imu.gyro[0] << "," << imu.gyro[1] << "," << imu.gyro[2] << ",";

  // Append GPS data
  if (gps.hour < 10) logger.append << 0;
  logger.append << int(gps.hour) << ":";
  if (gps.minute < 10) logger.append << 0;
  logger.append << int(gps.minute) << ":";
  if (gps.seconds < 10) logger.append << 0;
  logger.append << int(gps.seconds) << "." << int(gps.milliseconds) << ",";
  
  if (gps.day < 10) logger.append << 0;
  logger.append << int(gps.day) << "/";
  if (gps.month < 10) logger.append << 0;
  logger.append << int(gps.month) << "/20";
  if (gps.year < 10) logger.append << 0;
  logger.append << int(gps.year) << ",";
  
  logger.append << gps.latitude << gps.lat << ",";
  logger.append << gps.longitude << gps.lon << ",";
  logger.append << gps.speed << ",";
  logger.append << gps.altitude << ",";
  
  logger.echo();
  logger.recordln();
  
}
