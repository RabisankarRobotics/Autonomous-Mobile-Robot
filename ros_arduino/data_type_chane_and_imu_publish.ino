#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//******************************* Initialize ros Publisher Node ********************************
ros::NodeHandle nh;

std_msgs::Float32MultiArray imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);

float imu_data[6];


// *************************** Imu data publisher *****************************************

void imu_data_publish(){

  sensors_event_t angVelocityData, linearAccelData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  float gyroX = static_cast<float>(angVelocityData.gyro.x);
  float gyroY = static_cast<float>(angVelocityData.gyro.y);
  float gyroZ = static_cast<float>(angVelocityData.gyro.z);

  float accelX = static_cast<float>(linearAccelData.acceleration.x);
  float accelY = static_cast<float>(linearAccelData.acceleration.y);
  float accelZ = static_cast<float>(linearAccelData.acceleration.z);
//
//  float gyroX = 5.0;
//  float gyroY = 2.0;
//  float gyroZ = 3.0;
//
//  float accelX = 1.0;
//  float accelY = 6.0;
//  float accelZ = 7.0;
//  Serial.println("ROS - Angular Velocity:");
//  Serial.println(gyroX);
//  Serial.println(gyroY);
//  Serial.println(gyroZ);
//
//  Serial.println("ROS - Linear acceleration:");
//  Serial.println(accelX);
//  Serial.println(accelY);
//  Serial.println(accelZ);

  imu_data[0] = gyroX;
  imu_data[1] = gyroY;
  imu_data[2] = gyroZ;
  imu_data[3] = accelX;
  imu_data[4] = accelY;
  imu_data[5] = accelZ;

  imu_msg.data_length = 6;
  imu_msg.data = imu_data;

  imu_pub.publish(&imu_msg);

  
  }



void setup() {
 
 
  nh.initNode();

  nh.advertise(imu_pub);
  
  
}


void loop() {

   nh.spinOnce();

   imu_data_publish();
   
   delay(BNO055_SAMPLERATE_DELAY_MS);
  
}
