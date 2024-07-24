// Plug-and-play USB ROS-enabled micro AHRS and IMU software
// Author: Grzegorz Ficht <ficht@ais.uni-bonn.de>

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#define ADAFRUIT 1
#define SPARKFUN 2

#define GEOMETRY_MSGS 1
#define IMU_MSGS 2

/* Provide a default library if the user does not select one. */
#ifndef LIBRARY_TO_USE
#define LIBRARY_TO_USE SPARKFUN
#endif

#ifndef MESSAGES_TO_USE
#define MESSAGES_TO_USE GEOMETRY_MSGS
#endif

/* Include the selected library while handling errors properly. */
#if LIBRARY_TO_USE == ADAFRUIT
#include <Adafruit_BNO08x.h>
#elif LIBRARY_TO_USE == SPARKFUN
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#else
#error "Invalid choice for LIBRARY_TO_USE (select ADAFRUIT or SPARKFUN)"
#endif

/* Include the selected library while handling errors properly. */
#if MESSAGES_TO_USE == GEOMETRY_MSGS
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#elif MESSAGES_TO_USE == IMU_MSGS
#include <imuData.h>
#else
#error "Invalid choice for MESSAGES_TO_USE (select GEOMETRY_MSGS or IMU_MSGS)"
#endif
  
#include <ros.h>
#include <std_msgs/String.h>



typedef struct 
{
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float quatR;
  float quatI;
  float quatJ;
  float quatK;
} imuDataMeasured;

typedef struct 
{
    uint32_t milliseconds;
    uint16_t seconds;
    uint16_t minutes;
    uint16_t hours;
    uint16_t days;
} timeData;

#define SERIAL_DEBUG_ENABLE 0
#define NUMPIXELS 1
#define LED_POWER 11
#define PIN 12
#define BNO08X_INT A3 //A2
#define BNO08X_RESET D8 // A0
#define BNO08X_ADDR 0x4A // 0x4B 

void getRealTime(timeData &time);
void resetBNO();

// init stuff
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#if LIBRARY_TO_USE == ADAFRUIT
  Adafruit_BNO08x  bno08x(BNO08X_RESET);
#elif LIBRARY_TO_USE == SPARKFUN
  BNO08x bno08x;
#endif
volatile imuDataMeasured imu;
volatile bool haveImu=false;
volatile bool accessingImu = false;
sh2_SensorValue_t sensorValue;
timeData prevTime;

ros::NodeHandle nh;
#if MESSAGES_TO_USE == GEOMETRY_MSGS
geometry_msgs::Vector3 acc_msg;
geometry_msgs::Vector3 gyro_msg;
geometry_msgs::Quaternion quat_msg;
ros::Publisher pubAcc("imuData/acc", &acc_msg);
ros::Publisher pubGyro("imuData/gyro", &gyro_msg);
ros::Publisher pubQuat("imuData/quat", &quat_msg);
#elif MESSAGES_TO_USE == IMU_MSGS
imu_data_ros_msgs::imuData imu_data_msg;
ros::Publisher pubImu("imuState",&imu_data_msg);
#endif

char stringData[13] = "nnn"; // no data

void getRealTime(timeData &time) 
{
  time.milliseconds = millis();
  time.seconds = time.milliseconds / 1000;
  time.minutes = time.seconds / 60;
  time.hours = time.minutes / 60;
  time.days = time.hours / 24;
}

void resetBNO()
{
    digitalWrite(BNO08X_RESET, HIGH);
    delay(10);
    digitalWrite(BNO08X_RESET, LOW);
    delay(10);
    digitalWrite(BNO08X_RESET, HIGH);
    delay(10);
}

void setup() 
{
  pinMode(BNO08X_INT,INPUT_PULLUP);
  pinMode(BNO08X_RESET,OUTPUT);
  digitalWrite(BNO08X_RESET, HIGH);
  if(SERIAL_DEBUG_ENABLE)
    Serial.begin(115200);
  if(SERIAL_DEBUG_ENABLE)
    while (!Serial) delay(10);

  getRealTime(prevTime);

#if LIBRARY_TO_USE == ADAFRUIT
  while(!bno08x.begin_I2C(BNO08X_ADDR))
  { 
    if(SERIAL_DEBUG_ENABLE) Serial.println("Failed to find BNO08x chip (i2c)");
    delay(100);
    bno08x.softReset();
    resetBNO();
    delay(100);
  }
#elif LIBRARY_TO_USE == SPARKFUN
  Wire.begin();
  Wire.setClock(400000); // works with 100khz, 400kHz and 1MHz
  while(!bno08x.begin(BNO08X_ADDR, Wire, -1, -1))
  { 
    if(SERIAL_DEBUG_ENABLE) Serial.println("Failed to find BNO08x chip (i2c)");
    delay(100);
    // bno08x.softReset();
    resetBNO();
    delay(100);
  }
  haveImu = true;
#endif

  
  if(SERIAL_DEBUG_ENABLE) Serial.println("BNO08x found!");

  if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) // 100Hz, set to 5000 for 200Hz
  { 
    if(SERIAL_DEBUG_ENABLE) Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) // 100Hz, set to 5000 for 200Hz
  {
    if(SERIAL_DEBUG_ENABLE) Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) // 100Hz, set to 5000 for 200Hz
  {
    if(SERIAL_DEBUG_ENABLE) Serial.println("Could not enable rotation vector");
  }

  if(SERIAL_DEBUG_ENABLE) Serial.printf("Starting main loop ");

}


// core0 - handles reading IMU
void loop() 
{
  if (bno08x.wasReset())
  {
    if(SERIAL_DEBUG_ENABLE) Serial.print("sensor was reset ");
    if (!bno08x.enableReport(SH2_ACCELEROMETER,10000)) { // 100Hz, set to 5000 for 200Hz
      if(SERIAL_DEBUG_ENABLE) Serial.println("Could not enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR,10000)) { // 100Hz, set to 5000 for 200Hz
      if(SERIAL_DEBUG_ENABLE) Serial.println("Could not enable rotation vector");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,10000)) { // 100Hz, set to 5000 for 200Hz
      if(SERIAL_DEBUG_ENABLE) Serial.println("Could not enable rotation vector");
    }

  }

#if LIBRARY_TO_USE == ADAFRUIT
  if (!digitalRead(BNO08X_INT)) // if the int pin is low, means we got stuff, should check it;
  {
    bno08x.getSensorEvent(&sensorValue);
    switch (sensorValue.sensorId)
    {
    case SH2_ACCELEROMETER:
      imu.accX = sensorValue.un.accelerometer.x;
      imu.accY = sensorValue.un.accelerometer.y;
      imu.accZ = sensorValue.un.accelerometer.z;
      // if(SERIAL_DEBUG_ENABLE)  Serial.println("a");
      stringData[0] = 'a';
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      imu.gyroX = sensorValue.un.gyroscope.x;
      imu.gyroY = sensorValue.un.gyroscope.y;
      imu.gyroZ = sensorValue.un.gyroscope.z;
      // if(SERIAL_DEBUG_ENABLE)  Serial.println("g");
      stringData[1] = 'g';
      break;
    case SH2_GAME_ROTATION_VECTOR:
      imu.quatR = sensorValue.un.gameRotationVector.real;
      imu.quatI = sensorValue.un.gameRotationVector.i;
      imu.quatJ = sensorValue.un.gameRotationVector.j;
      imu.quatK = sensorValue.un.gameRotationVector.k;
      // if(SERIAL_DEBUG_ENABLE)  Serial.println("r");
      stringData[2] = 'r';
      break;
    }
  }
#elif LIBRARY_TO_USE == SPARKFUN
  if (!digitalRead(BNO08X_INT)) // if the int pin is low, means that BNO is ready to transmit
  {
    if (bno08x.getSensorEvent())
    {
      if (bno08x.getSensorEventID() == SH2_ACCELEROMETER)
      {
        // accessingImu = true;
        imu.accX = bno08x.getAccelX();
        imu.accY = bno08x.getAccelY();
        imu.accZ = bno08x.getAccelZ();
        // accessingImu = false;
        stringData[0] = 'a';
      }
      if (bno08x.getSensorEventID() == SH2_GYROSCOPE_CALIBRATED)
      {
        // accessingImu = true;
        imu.gyroX = bno08x.getGyroX();
        imu.gyroY = bno08x.getGyroY();
        imu.gyroZ = bno08x.getGyroZ();
        // accessingImu = false;
        stringData[1] = 'g';
      }
      if (bno08x.getSensorEventID() == SH2_GAME_ROTATION_VECTOR)
      {
        // accessingImu = true;
        imu.quatR = bno08x.getGameQuatReal();
        imu.quatI = bno08x.getGameQuatI();
        imu.quatJ = bno08x.getGameQuatJ();
        imu.quatK = bno08x.getGameQuatK();
        // accessingImu = false;
        stringData[1] = 'r';
      }
    }
  }

#endif

  if(SERIAL_DEBUG_ENABLE) Serial.println(stringData); //data

}


void setup1()
{
  pixels.begin();
  pinMode(LED_POWER,OUTPUT);
  digitalWrite(LED_POWER, HIGH);
  nh.initNode();

  #if MESSAGES_TO_USE == GEOMETRY_MSGS
  nh.advertise(pubAcc);
  nh.advertise(pubGyro);
  nh.advertise(pubQuat);
  #elif MESSAGES_TO_USE == IMU_MSGS
  nh.advertise(pubImu);
  #endif
  nh.getHardware()->setBaud(1000000);
}


// core1 - handles publishing over serial
void loop1()
{
  timeData now;
  getRealTime(now);
  if((now.milliseconds - prevTime.milliseconds)>=9)
  {
    #if MESSAGES_TO_USE == GEOMETRY_MSGS
    // while(accessingImu);
    acc_msg.x = (double)imu.accX;
    acc_msg.y = (double)imu.accY;
    acc_msg.z = (double)imu.accZ;
    pubAcc.publish( &acc_msg );
    // while(accessingImu);
    gyro_msg.x = (double)imu.gyroX;
    gyro_msg.y = (double)imu.gyroY;
    gyro_msg.z = (double)imu.gyroZ;
    pubGyro.publish( &gyro_msg );
    // while(accessingImu);
    quat_msg.w = (double)imu.quatR;
    quat_msg.x = (double)imu.quatI;
    quat_msg.y = (double)imu.quatJ;
    quat_msg.z = (double)imu.quatK;
    pubQuat.publish( &quat_msg );
    #elif MESSAGES_TO_USE == IMU_MSGS
    imu_data_msg.acc.x = (double)imu.accX;
    imu_data_msg.acc.y = (double)imu.accY;
    imu_data_msg.acc.z = (double)imu.accZ;
    imu_data_msg.gyro.x = (double)imu.gyroX;
    imu_data_msg.gyro.y = (double)imu.gyroY;
    imu_data_msg.gyro.z = (double)imu.gyroZ;
    imu_data_msg.orientation.w = (double)imu.quatR;
    imu_data_msg.orientation.x = (double)imu.quatI;
    imu_data_msg.orientation.y = (double)imu.quatJ;
    imu_data_msg.orientation.z = (double)imu.quatK;
    pubImu.publish(&imu_data_msg);
    #endif
    nh.spinOnce();
    stringData[0] = 'n';
    stringData[1] = 'n';
    stringData[2] = 'n';
    prevTime = now;
  }
  if(prevTime.milliseconds > now.milliseconds) // if there was a time overflow
    prevTime = now;
  if(!haveImu)
  { // if IMU not set-up then show red
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(50, 0, 0));
      pixels.show();
  }
  else if(nh.connected())
  { // if IMU set up, but not connected to ros master then show blue
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 50, 0));
      pixels.show();
  }
  else
  { // if all is OK then show green
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 0, 50));
      pixels.show();
  }
}
