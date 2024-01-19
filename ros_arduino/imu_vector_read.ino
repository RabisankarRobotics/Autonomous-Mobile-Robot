#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//******************************* Initialize ros Publisher Node ********************************



void setup() {
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    while (1);
  }
  bno.setExtCrystalUse(true);
}

void loop() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.print("Gyroscope: ");
  Serial.print(gyro.x());
  Serial.print(", ");
  Serial.print(gyro.y());
  Serial.print(", ");
  Serial.print(gyro.z());
  Serial.print("\tLinear Acceleration: ");
  Serial.print(linear_accel.x());
  Serial.print(", ");
  Serial.print(linear_accel.y());
  Serial.print(", ");
  Serial.print(linear_accel.z());
  Serial.println();
  delay(20);
}
