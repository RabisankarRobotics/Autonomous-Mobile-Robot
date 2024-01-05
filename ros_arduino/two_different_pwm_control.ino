
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>  // Int16 for use for positive and negative both value

ros::NodeHandle nh;

const int motor1PWM_pin = 9;  // PWM pin for Motor 1
const int motor1Dir_pin = 8;  // Direction pin for Motor 1

const int motor2PWM_pin = 10; // PWM pin for Motor 2
const int motor2Dir_pin = 11; // Direction pin for Motor 2

void motor_control_cb(const std_msgs::Int32MultiArray& pwm_msg)
{
  // Extract PWM values from the message
  int speedMotor1 = pwm_msg.data[0];
  int speedMotor2 = pwm_msg.data[1]; 
  

  // Determine the direction based on the sign of the speed
  int Motor1_direction = (speedMotor1 >= 0) ? HIGH : LOW;
  int motor2_direction = (speedMotor2 >= 0) ? HIGH : LOW;
  

  // Set motor directions
  digitalWrite(motor1Dir_pin, Motor1_direction);
  digitalWrite(motor2Dir_pin, motor2_direction);

  // Set motor speeds (use the absolute value)
  analogWrite(motor1PWM_pin, abs(speedMotor1));
  analogWrite(motor2PWM_pin, abs(speedMotor2));
  
}


ros::Subscriber<std_msgs::Int32MultiArray> sub("motor_pwm_control", &motor_control_cb);


void setup() {
  // put your setup code here, to run once:
  pinMode(motor1PWM_pin, OUTPUT);
  pinMode(motor1Dir_pin, OUTPUT);

  pinMode(motor2PWM_pin, OUTPUT);
  pinMode(motor2Dir_pin, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();

}
