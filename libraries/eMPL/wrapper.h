#ifndef _WRAPPER_H_
#define _WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

struct I2CdevWrapper; // An opaque type that we'll use as a handle
typedef struct I2CdevWrapper I2CdevWrapper;

I2CdevWrapper * I2Cdev_create();
void I2Cdev_destroy( I2CdevWrapper * v );

char I2Cdev_write(I2CdevWrapper * v, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
char I2Cdev_read(I2CdevWrapper * v, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _WRAPPER_H_ */