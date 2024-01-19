#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

ros::NodeHandle nh;

std_msgs::Float32MultiArray imu_msg;
ros::Publisher imu_publisher("imu_data", &imu_msg);
float imu_data[6];

void imu_data_publish(){

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  float gyroX = gyro.x();
  float gyroY = gyro.y();
  float gyroZ = gyro.z();
  
  float accelX = linear_accel.x();
  float accelY = linear_accel.y();
  float accelZ = linear_accel.z();

  imu_data[0] = gyroX;
  imu_data[1] = gyroY;
  imu_data[2] = gyroZ;
  imu_data[3] = accelX;
  imu_data[4] = accelY;
  imu_data[5] = accelZ;

//  imu_data[0] = 4;
//  imu_data[1] = 5;
//  imu_data[2] = 6;
//  imu_data[3] = 7;
//  imu_data[4] = 8;
//  imu_data[5] = 9;

  imu_msg.data_length = 6;
  imu_msg.data = imu_data;

  imu_publisher.publish(&imu_msg);

  
  }


void setup() {

  
  nh.initNode();
  nh.advertise(imu_publisher);
  
}

void loop() {
  
  nh.spinOnce();
  imu_data_publish();
  delay(20);  // Adjust the delay as needed
}
