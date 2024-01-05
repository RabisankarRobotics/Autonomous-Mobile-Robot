#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;

const int motor1PWM_pin = 9;  // PWM pin for Motor 1
const int motor1Dir_pin = 8;  // Direction pin for Motor 1
const int motor2PWM_pin = 10; // PWM pin for Motor 2
const int motor2Dir_pin = 11; // Direction pin for Motor 2


// puls count per revolution
#define ENC_COUNT_REV 8123


// encoder pin connect for right motor  
#define ENC_IN_RIGHT_A 3
#define ENC_IN_RIGHT_B 5


// encoder pin for left motor
#define ENC_IN_LEFT_A 2
#define ENC_IN_LEFT_B 4

 
// True = Forward; False = Reverse
boolean Direction_right = true;
boolean Direction_left = true;

 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
// Keep track of the number of left wheel pulses
volatile long left_wheel_pulse_count = 0;

 
// One-second interval for measurements
int interval = 1000;

  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

 
// Variable for RPM measuerment
float rpm_right = 0;
float rpm_left = 0;
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;

// Variable for angular velocity measurement
float ang_velocity_left = 0;
float ang_velocity_left_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;


void motorMove(int speedMotor1, int speedMotor2) {
  int Motor1_direction = (speedMotor1 >= 0) ? HIGH : LOW;
  int motor2_direction = (speedMotor2 >= 0) ? HIGH : LOW;

  // Set motor directions
  digitalWrite(motor1Dir_pin, Motor1_direction);
  digitalWrite(motor2Dir_pin, motor2_direction);

  // Set motor speeds (use the absolute value)
  analogWrite(motor1PWM_pin, abs(speedMotor1));
  analogWrite(motor2PWM_pin, abs(speedMotor2));
}


void pwmCallback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data_length == 2) {
    int speedMotor1 = static_cast<int>(msg.data[0]);
    int speedMotor2 = static_cast<int>(msg.data[1]);
    motorMove(speedMotor1, speedMotor2);
  }
}



ros::Subscriber<std_msgs::Int32MultiArray> sub("motor_pwm_control", &pwmCallback);


void setup() {
  // Your existing setup code 
  
  // Open the serial port at 9600 bps
  Serial.begin(9600); 

  pinMode(motor1PWM_pin, OUTPUT);
  pinMode(motor1Dir_pin, OUTPUT);
  pinMode(motor2PWM_pin, OUTPUT);
  pinMode(motor2Dir_pin, OUTPUT);
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
    // Your existing loop code here
  
    currentMillis = millis();

    if (currentMillis - previousMillis > interval) {
   
      previousMillis = currentMillis;
   
      // Calculate revolutions per minute for right motor
      rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
     
      // Calculate revolutions per minute for left motor
      rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);
       
      Serial.print(" Pulses: ");
      Serial.print(right_wheel_pulse_count);
      Serial.print(",");
      Serial.println(left_wheel_pulse_count);
      
      Serial.print(" Speed: ");
      Serial.print(rpm_right);
      Serial.print(",");
      Serial.print(rpm_left);
      Serial.println(" RPM");
   
      right_wheel_pulse_count = 0;
  
      left_wheel_pulse_count = 0;
     
    }
}


void right_wheel_pulse() {
   
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}



void left_wheel_pulse() {
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = false; // Reverse
  } else {
    Direction_left = true; // Forward
  }

  if (Direction_left) {
    left_wheel_pulse_count++;
  } else {
    left_wheel_pulse_count--;
  }
}
