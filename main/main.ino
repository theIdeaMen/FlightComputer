#include <PESO_inc.h>

// Objects
Logger logger;    // Logs to microSD card over SPI
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
