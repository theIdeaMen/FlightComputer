#include "PESO_IMU.h"

IMU::IMU()
{
  mpuInterrupt = false;
}

int IMU::initialize(void (*func)())
{
  if (mpu_init())
    return -1;
  if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
    return -2;
  if (mpu_set_sample_rate(100))
    return -3;
  if (mpu_set_accel_fsr(8))
    return -4;



  if (dmp_load_motion_driver_firmware())
    return -5;
  else
  {
    //dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    
    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
      return -6;
    if (dmp_set_fifo_rate(100))
      return -7;
    if (mpu_set_dmp_state(1))
      return -8;
    attachInterrupt(0, func, RISING);
    if (mpu_get_int_status(&mpuIntStatus))
      return -9;
    Serial.println("DMP On");
  }

  return 0;
}

void IMU::interrupt()
{
  mpuInterrupt = true;
}

void IMU::update()
{
  if (!mpuInterrupt) return;

  mpuInterrupt = false;
  mpu_get_int_status(&mpuIntStatus);

  if ((mpuIntStatus & 0x10))
  {
    Serial.println("fifo reset");
    mpu_reset_fifo();
    return;
  }
  
  if (!(mpuIntStatus & 0x01)) return;

  dmp_read_fifo(gyro, aa, q, &timestamp, &sensors, &more);
  timestamp = millis();
}
