#include <PESO_inc.h>   // Contains many include files and definitions

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
  logger.append << setprecision(6) << "PESO Flight Computer Log File";
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
  imu.update;
  
  // Append data type and time
  logger.append << "IMU," << imu.timestamp << ",";
  
  // Append temperature
  logger.append << (imu.temperature/(float)65536) << ",";
  
  // Append IMU data
  logger.append << (imu.q[0]/(float)1073741824) << "," << (imu.q[1]/(float)1073741824) << "," << (imu.q[2]/(float)1073741824) << "," << (imu.q[3]/(float)1073741824) << ",";
  logger.append << (imu.aa[0]/(float)imu.aa_sens) << "," << (imu.aa[1]/(float)imu.aa_sens) << "," << (imu.aa[2]/(float)imu.aa_sens) << ",";
  
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