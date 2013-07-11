

bool I2Cdev_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{ 
  return I2Cdev::writeBytes(slave_addr, reg_addr, length, data); 
}
char I2Cdev_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
  return I2Cdev::readBytes(slave_addr, reg_addr, length, data);
}