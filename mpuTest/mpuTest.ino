#include "MPU9250.h"
#include "math.h"

float prevAngle = 0;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();

  // get angle using accel
  float accY = IMU.getAccelY_mss(); // ms^-2
  float accZ = IMU.getAccelZ_mss();

  float accAngle = atan2(accY, accZ) * RAD_TO_DEG + 180;

  // left side
  if(accAngle > 180) {
    accAngle -= 360;
  }

  // get angle using gyro
  float gyroX = IMU.getGyroX_rads() * RAD_TO_DEG; //deg^-1
  // float gyroRate = map(gyroX, -32768, 32767, -250, 250);
  float gyroAngle = prevAngle + gyroX * 4 / 1000;

  // complementary filter
  float currentAngle = 0.9934 * gyroAngle + 0.0066 * accAngle;

  prevAngle = gyroAngle;

  // display the data

  Serial.print(IMU.getGyroX_rads(), 6);
  Serial.print("\t");
  Serial.print(accAngle, 6);
  Serial.print("\t");
  Serial.print(gyroAngle, 6);
  Serial.print("\t");
  Serial.println(currentAngle, 6);

  // Serial.print(IMU.getAccelX_mss(), 6);
  // Serial.print("\t");
  // Serial.print(IMU.getAccelY_mss(), 6);
  // Serial.print("\t");
  // Serial.print(IMU.getAccelZ_mss(), 6);
  // Serial.print("\t");
  // Serial.print(IMU.getGyroX_rads(), 6);
  // Serial.print("\t");
  // Serial.print(IMU.getGyroY_rads(), 6);
  // Serial.print("\t");
  // Serial.println(IMU.getGyroZ_rads(), 6);

  delay(4);
}