// Motor 1 pins
const int motor1PWM = 9;  // PWM pin for Motor 1
const int motor1Dir = 8;  // Direction pin for Motor 1

// Motor 2 pins
const int motor2PWM = 10; // PWM pin for Motor 2
const int motor2Dir = 11; // Direction pin for Motor 2

void setup() {
  // Set motor control pins as output
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
}

// Function to control a motor using PWM
void motorControl(int pwmPin, int dirPin, int speed) {
  digitalWrite(dirPin, (speed >= 0) ? HIGH : LOW);  // Set motor direction
  analogWrite(pwmPin, abs(speed));  // Set motor speed (0 to 255)
}

void loop() {
  // Example usage
  motorControl(motor1PWM, motor1Dir, 200);  // Motor 1, forward at speed 150
  motorControl(motor2PWM, motor2Dir, 200); // Motor 2, reverse at speed 200

  // Add a delay to observe the motor behavior
  delay(1000);

  // Stop the motors
  motorControl(motor1PWM, motor1Dir, 0);  // Motor 1, stop
  motorControl(motor2PWM, motor2Dir, 0);  // Motor 2, stop

  // Add a delay before repeating the loop
  delay(1000);

  motorControl(motor1PWM, motor1Dir, -200);  // Motor 1, forward at speed 150
  motorControl(motor2PWM, motor2Dir, -200); // Motor 2, reverse at speed 200

   // Add a delay before repeating the loop
  delay(1000);

  // Stop the motors
  motorControl(motor1PWM, motor1Dir, 0);  // Motor 1, stop
  motorControl(motor2PWM, motor2Dir, 0);  // Motor 2, stop


}
