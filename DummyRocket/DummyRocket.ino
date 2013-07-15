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
#define SAMPLE_RATE 70      // Must be between 4 and 200 Hz

Logger logger;
Timer timer(SAMPLE_RATE);

IMU imu;
void imuInterrupt() { imu.interrupt(); } // IMU interrupt


GPS gps;
SIGNAL(TIMER0_COMPA_vect) { char c = gps.data.read(); } // GPS interrupt

Trigger cutter;

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
  
  while(imu.initialize(imuInterrupt, SAMPLE_RATE))
    delay(100);
  
  logger.initialize(3, false); // Chip Select Pin 3, Full Speed
  
  gps.initialize();
  
  logger.append << setprecision(9) << "timestamp,temperature,q_w,q_x,q_y,q_z,aa_x,aa_y,aa_z,gyro_x,gyro_y,gyro_z,time,date,lat,lon,speed,alt";
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
  
  // Append temperature
  logger.append << (imu.temperature/(float)65536) << ",";
  
  // Append IMU data
  logger.append << (imu.q[0]/(float)1073741824) << "," << (imu.q[1]/(float)1073741824) << "," << (imu.q[2]/(float)1073741824) << "," << (imu.q[3]/(float)1073741824) << ",";
  logger.append << (imu.aa[0]/(float)imu.aa_sens) << "," << (imu.aa[1]/(float)imu.aa_sens) << "," << (imu.aa[2]/(float)imu.aa_sens) << ",";
  logger.append << (imu.gyro[0]/imu.gyro_sens) << "," << (imu.gyro[1]/imu.gyro_sens) << "," << (imu.gyro[2]/imu.gyro_sens) << ",";

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
  logger.append << gps.altitude;
  
  logger.echo();
  logger.recordln();
  
}
