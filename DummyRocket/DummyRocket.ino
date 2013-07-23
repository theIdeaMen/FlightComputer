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
#define CUTDOWN_PIN 5
#define CUT_EXPRMNT1 6
#define CUT_EXPRMNT2 7
#define CUT_EXPRMNT3 8

#define HEARTBEAT 9

#define COMMAND_LINK 10

#define REMOVE_BEFORE_FLIGHT 11

#define SAMPLE_RATE 31      // Must be between 4 and 35 Hz

// Altitude drop trigger for MAIN FLIGHT TERMINATION UNIT
Trigger cutter(CUTDOWN_PIN, 40, Trigger::ABOVE, 5000, Trigger::ABOVE);
// Altitude drop trigger
Trigger cut_exprmnt1(CUT_EXPRMNT1, 40, Trigger::ABOVE, 5000, Trigger::ABOVE);
// Ground command trigger
Trigger cut_exprmnt2(CUT_EXPRMNT2, 1, Trigger::ABOVE, 5000, Trigger::ABOVE);
// Specific altitude trigger
Trigger cut_exprmnt3(CUT_EXPRMNT3, 25000, Trigger::ABOVE, 5000, Trigger::ABOVE);

long maxAltitude = 0;
unsigned long ms = 0;
short command = 0;
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
  Serial.println("Cutdown!");
}

void stopCallBack()
{
  Serial.println("Cutdown secured.");
}


void setup()
{
  Serial.begin(115200);
  
  Wire.begin();
  
  // Initialise the IO and ISR
  vw_set_ptt_inverted(true);    // Required for DR3100
  vw_setup(2000);	              // Bits per sec
  vw_set_rx_pin(COMMAND_LINK);  // Pin for receiving data
  vw_rx_start();                // Start the receiver PLL running
  
  pinMode(REMOVE_BEFORE_FLIGHT, INPUT);
  digitalWrite(REMOVE_BEFORE_FLIGHT, HIGH);
  pinMode(HEARTBEAT, OUTPUT);
  digitalWrite(HEARTBEAT, LOW);
  
  while(imu.initialize(imuInterrupt, SAMPLE_RATE))
    delay(100);
  
  logger.initialize(3, false); // Chip Select Pin 3, Full Speed
  
  gps.initialize();
  
  logger.append << setprecision(9) << "timestamp,temperature,q_w,q_x,q_y,q_z,aa_x,aa_y,aa_z,gyro_x,gyro_y,gyro_z,time,date,lat,lon,speed,alt";
  logger.echo();
  logger.recordln();
  
  cutter.onCallBack(&cutCallBack);
  cutter.offCallBack(&stopCallBack);
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
    for (short i = 0; i < buflen; i++)
    {
      Serial.print(buf[i]);
      if (buf[i] == '-')
      {
        if (buf[i+1] == 'C' && buf[i+2] == 'U' && buf[i+3] == 'T' && buf[i+4] == '!')
          command = 2;
      }
      Serial.print(" ");
    }
  }

  // Wait for set delay
  if (!timer.ready()) return;

  // Arm the cutdowns after 10 minutes
  if (armTimer.delta() > 600000)
  {
    ms = millis();
    maxAltitude = max(maxAltitude, gps.altitude);
    cutter.update(maxAltitude - gps.altitude, ms);
    cut_exprmnt1.update(maxAltitude - gps.altitude, ms);
    cut_exprmnt2.update(command, ms);
    cut_exprmnt3.update(gps.altitude, ms);
  }
  
  // Append time since last update
  logger.append << imu.timestamp << ",";
  
  // Append temperature
  logger.append << (imu.temperature/(float)65536) << ",";
  
  // Append IMU data
  logger.append << (imu.q[0]/(float)1073741824) << "," << (imu.q[1]/(float)1073741824) << "," << (imu.q[2]/(float)1073741824) << "," << (imu.q[3]/(float)1073741824) << ",";
  logger.append << (imu.aa[0]/(float)imu.aa_sens) << "," << (imu.aa[1]/(float)imu.aa_sens) << "," << (imu.aa[2]/(float)imu.aa_sens) << ",";
  logger.append << (imu.gyro[0]/imu.gyro_sens) << "," << (imu.gyro[1]/imu.gyro_sens) << "," << (imu.gyro[2]/imu.gyro_sens) << ",";

  // Append GPS data
  if (gps.hour < 10) logger.append << 0;
  logger.append << int(gps.hour) << ":";
  if (gps.minute < 10) logger.append << 0;
  logger.append << int(gps.minute) << ":";
  if (gps.seconds < 10) logger.append << 0;
  logger.append << int(gps.seconds) << "." << int(gps.milliseconds) << ",";
  
  if (gps.day < 10) logger.append << 0;
  logger.append << int(gps.day) << "/";
  if (gps.month < 10) logger.append << 0;
  logger.append << int(gps.month) << "/20";
  if (gps.year < 10) logger.append << 0;
  logger.append << int(gps.year) << ",";
  
  logger.append << gps.latitude << gps.lat << ",";
  logger.append << gps.longitude << gps.lon << ",";
  logger.append << gps.speed << ",";
  logger.append << gps.altitude;
  
  //logger.echo();
  logger.recordln();
  
}
