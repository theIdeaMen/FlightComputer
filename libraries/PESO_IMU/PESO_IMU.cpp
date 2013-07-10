#include "PESO_IMU.h"

IMU::IMU()
{
  packetSize = 0;
  mpuInterrupt = false;
}

void IMU::initialize(void (*func)())
{
  mpu_init();
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(100);
  mpu_set_accel_fsr(8);

  if (dmp_load_motion_driver_firmware() == 0)
  {
    //dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    dmp_set_fifo_rate(100);
    mpu_set_dmp_state(1);
    attachInterrupt(0, func, RISING);
    mpu_get_int_status(&mpuIntStatus);
  }
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
    mpu_reset_fifo();
    return;
  }
  
  if (!(mpuIntStatus & 0x01)) return;
  
  dmp_read_fifo(gyro, aa, q, &timestamp, &sensors, &more);
}
