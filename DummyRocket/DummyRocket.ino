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

// Altitude drop trigger for MAIN FLIGHT TERMINATION UNIT
Trigger cutter(CUTDOWN_PIN, 40, Trigger::ABOVE, 5000, Trigger::ABOVE);
// Altitude drop trigger
Trigger cut_exprmnt1(CUT_EXPRMNT1, 40, Trigger::ABOVE, 5000, Trigger::ABOVE);
// Ground command trigger
Trigger cut_exprmnt2(CUT_EXPRMNT2, 1, Trigger::ABOVE, 5000, Trigger::ABOVE);
// Specific altitude trigger
Trigger cut_exprmnt3(CUT_EXPRMNT3, 25000, Trigger::ABOVE, 5000, Trigger::ABOVE);

// Variables
long maxAltitude = 0;
unsigned long ms = 0;
short command = 0;
short count = 0;
byte buf[VW_MAX_MESSAGE_LEN];
byte buflen = VW_MAX_MESSAGE_LEN;

Logger logger;
Timer timer(SAMPLE_RATE);
Timer armTimer(1);
Timer hrtbtTimer(10000);

IMU imu;
void imuInterrupt() { imu.interrupt(); } // IMU interrupt

GPS gps;
SIGNAL(TIMER0_COMPA_vect) { char c = gps.data.read(); } // GPS interrupt

void cutCallBack()
{
  logger.append << millis() << ",";
  logger.append << "Main Cutdown Triggered";
  logger.echo();
  logger.recordln();
}

void exp1CallBack()
{
  logger.append << millis() << ",";
  logger.append << "Altitude Drop Cutdown Triggered";
  logger.echo();
  logger.recordln();
}
void exp2CallBack()
{
  logger.append << millis() << ",";
  logger.append << "Ground Command Cutdown Triggered";
  logger.echo();
  logger.recordln();
}
void exp3CallBack()
{
  logger.append << millis() << ",";
  logger.append << "Reached Altitude Cutdown Triggered";
  logger.echo();
  logger.recordln();
}


void setup()
{
  Serial.begin(115200);
  
  Wire.begin();
  
  // Initialize the IO and ISR
  vw_setup(2000);	              // Bits per sec
  vw_set_rx_pin(COMMAND_LINK);  // Pin for receiving data
  vw_rx_start();                // Start the receiver PLL running
  
  pinMode(HEARTBEAT, OUTPUT);
  digitalWrite(HEARTBEAT, LOW);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(REMOVE_BEFORE_FLIGHT, INPUT);
  digitalWrite(REMOVE_BEFORE_FLIGHT, HIGH);
  
  
  while(imu.initialize(imuInterrupt, SAMPLE_RATE))
    delay(100);
  
  logger.initialize(3, false); // Chip Select Pin 3, Full Speed
  
  gps.initialize();
  
  logger.append << setprecision(9) << "timestamp,temperature,q_w,q_x,q_y,q_z,aa_x,aa_y,aa_z,time,date,lat,lon,speed,alt";
  logger.echo();
  logger.recordln();
  
  cutter.onCallBack(&cutCallBack);
  cut_exprmnt1.onCallBack(&exp1CallBack);
  cut_exprmnt2.onCallBack(&exp2CallBack);
  cut_exprmnt3.onCallBack(&exp3CallBack);
  
  // Sound buzzer twice
  digitalWrite(BUZZER, HIGH);
  delay(800);
  digitalWrite(BUZZER, LOW);
  delay(500);
  digitalWrite(BUZZER, HIGH);
  delay(800);
  digitalWrite(BUZZER, LOW);
}

void loop()
{

  if (hrtbtTimer.ready()) 
    digitalWrite(HEARTBEAT, HIGH);
  else
    digitalWrite(HEARTBEAT, LOW);
  
  imu.update();
  gps.update();
  
  if (digitalRead(REMOVE_BEFORE_FLIGHT) == LOW) { armTimer.reset(); return; }
  
  // Check for radio link command
  if (vw_get_message(buf, &buflen))
  {
    logger.append << millis() << ",";
    for (short i = 0; i < buflen; i++)
    {
      logger.append << buf[i];
      if (buf[i] == '-')
      {
        if (buf[i+1] == 'C' && buf[i+2] == 'U' && buf[i+3] == 'T' && buf[i+4] == '!')
          command = 2;
      }
    }
    int RSSI = analogRead(RSSI_PIN);
    logger.append << ",RSSI: " << RSSI;
    logger.echo();
    logger.recordln();
  }

  // Wait for set delay
  if (!timer.ready()) return;
  
  // Sound buzzer once
  if (count < 50)
  {
    digitalWrite(BUZZER, HIGH);
    count++;
  }
  else
  {
    digitalWrite(BUZZER, LOW);
  }

  // Arm the cutdowns after 10 minutes
  if (armTimer.delta() > 600000)
  {
    ms = millis();
    maxAltitude = max(maxAltitude, gps.altitude);
    if (gps.altitude)
    {
      cutter.update(maxAltitude - gps.altitude, ms);
      cut_exprmnt1.update(maxAltitude - gps.altitude, ms);
    }
    cut_exprmnt2.update(command, ms);
    cut_exprmnt3.update(gps.altitude, ms);
    
    if (gps.altitude < 600)
      count = 0;
  }
  
  // Append time since last update
  logger.append << imu.timestamp << ",";
  
  // Append temperature
  logger.append << (imu.temperature/(float)65536) << ",";
  
  // Append IMU data
  logger.append << (imu.q[0]/(float)1073741824) << "," << (imu.q[1]/(float)1073741824) << "," << (imu.q[2]/(float)1073741824) << "," << (imu.q[3]/(float)1073741824) << ",";
  logger.append << (imu.aa[0]/(float)imu.aa_sens) << "," << (imu.aa[1]/(float)imu.aa_sens) << "," << (imu.aa[2]/(float)imu.aa_sens) << ",";
  //logger.append << (imu.gyro[0]/imu.gyro_sens) << "," << (imu.gyro[1]/imu.gyro_sens) << "," << (imu.gyro[2]/imu.gyro_sens) << ",";

  // Append GPS data
  //if (gps.hour < 10) logger.append << 0;
  logger.append << setfill('0') << setw(2) << int(gps.hour) << ":";
  //if (gps.minute < 10) logger.append << 0;
  logger.append << setw(2) << int(gps.minute) << ":";
  //if (gps.seconds < 10) logger.append << 0;
  logger.append << setw(2) << int(gps.seconds) << "." << setw(3) << int(gps.milliseconds) << ",";
  
  //if (gps.day < 10) logger.append << 0;
  logger.append << setw(2) << int(gps.day) << "/";
  //if (gps.month < 10) logger.append << 0;
  logger.append << setw(2) << int(gps.month) << "/20";
  //if (gps.year < 10) logger.append << 0;
  logger.append << setw(2) << int(gps.year) << ",";
  
  logger.append << gps.latitude << ",";
  logger.append << gps.longitude << ",";
  logger.append << gps.speed << ",";
  logger.append << gps.altitude;
  
  //logger.echo();
  logger.recordln();
  
}
