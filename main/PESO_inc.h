// Includes and definitions
#include <Thread.h>
#include <ThreadController.h>

#include <Wire.h>
#include <I2Cdev.h>
#include <VirtualWire.h>

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

// Pin definitions
#define CUTDOWN_PIN   5
#define CUT_EXPRMNT1  6
#define CUT_EXPRMNT2  7
#define CUT_EXPRMNT3  8

#define HEARTBEAT     10
#define BUZZER        34
#define COMMAND_LINK  9
#define RSSI_PIN      1    // Analog pin number

#define REMOVE_BEFORE_FLIGHT 33

// Data logging rate
#define SAMPLE_RATE 20      // Must be between 4 and 20 Hz