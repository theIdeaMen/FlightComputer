#include "I2Cdev.h"
#include "wrapper.h"

I2CdevWrapper * I2Cdev_create()
{
  return reinterpret_cast<I2CdevWrapper*>( new I2Cdev );
}
void I2Cdev_destroy( I2CdevWrapper * v )
{
  delete reinterpret_cast<I2Cdev*>(v);
}
char I2Cdev_write( I2CdevWrapper * v, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data )
{
  if (reinterpret_cast<I2Cdev*>(v)->writeBytes(slave_addr, reg_addr, length, data))
    return 0;
  return -1;
}
char I2Cdev_read( I2CdevWrapper * v, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data )
{
  if (reinterpret_cast<I2Cdev*>(v)->readBytes(slave_addr, reg_addr, length, data))
    return 0;
  return -1;
}