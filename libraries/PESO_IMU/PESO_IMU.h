#ifndef __PESO_IMU__
#define __PESO_IMU__

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

class IMU
{
  public:
  
  short mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint16_t packetSize;// expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  
  long q[4];           // [w, x, y, z]         quaternion container
  short gyro[3];       // [x, y, z]            accel sensor measurements
  short aa[3];         // [x, y, z]            accel sensor measurements
  
  short sensors;
  unsigned char more;
  unsigned long timestamp;
  
  volatile bool mpuInterrupt;     // indicates whether MPU interrupt pin has gone high
  
  IMU();
  void initialize(void (*func)());
  void interrupt();
  void update();
  
};

#endif __PESO_IMU__