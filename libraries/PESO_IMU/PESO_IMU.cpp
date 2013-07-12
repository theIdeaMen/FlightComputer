#include "PESO_IMU.h"

IMU::IMU()
{
  mpuInterrupt = false;
}

void IMU::initialize(void (*func)())
{
  Serial.print("\nmpu_init: ");
  Serial.print(mpu_init());
  Serial.print("\nmpu_set_sensors: ");
  Serial.print(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
  Serial.print("\nmpu_set_sample_rate: ");
  Serial.print(mpu_set_sample_rate(100));
  Serial.print("\nmpu_set_accel_fsr: ");
  Serial.print(mpu_set_accel_fsr(8));

  int tmp = dmp_load_motion_driver_firmware();
  Serial.print("\ndmp_load_motion_driver_firmware: ");
  Serial.print(tmp);
  if (tmp == 0)
  {
    //dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    Serial.print("\ndmp_enable_feature: ");
    Serial.print(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL));
    Serial.print("\ndmp_set_fifo_rate: ");
    Serial.print(dmp_set_fifo_rate(100));
    Serial.print("\nmpu_set_dmp_state: ");
    Serial.print(mpu_set_dmp_state(1));
    attachInterrupt(0, func, RISING);
    Serial.print("\nmpu_get_int_status: ");
    Serial.print(mpu_get_int_status(&mpuIntStatus));
    Serial.println("DMP On");
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
    Serial.println("fifo reset");
    mpu_reset_fifo();
    return;
  }
  
  if (!(mpuIntStatus & 0x01)) return;

  dmp_read_fifo(gyro, aa, q, &timestamp, &sensors, &more);
}
