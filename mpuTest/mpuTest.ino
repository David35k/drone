// #include "MPU9250.h"
// #include "math.h"
#include "Wire.h"

float RateRoll, RatePitch, RateYaw;

// calibration variables
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

void gyro_signals(void) {
  // switch on low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // set sensitivity scale factor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  // access registers storing measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  // read gyro measurements
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // get in degrees per second
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

void setup() {
  Serial.begin(57600);
  
  // set clock speed of I2C to 400kHz
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // start gyro in power mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission();

  // take 2000 measurements and get calibration values
  for(RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  // calculate calibration values
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

}

void loop() {
  gyro_signals();

  // use calibration values to correct measurements
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // print values
  Serial.print("everything in deg/s ");
  Serial.print("roll rate: ");
  Serial.print(RateRoll);
  Serial.print(" pitch rate: ");
  Serial.print(RatePitch);
  Serial.print(" yaw rate: ");
  Serial.println(RateYaw);

  delay(50);
}








// -------------------- previous code --------------------

// float prevAngle = 0;

// // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
// MPU9250 IMU(Wire, 0x68);
// int status;

// void setup() {
//   // serial to display data
//   Serial.begin(115200);
//   while (!Serial) {}

//   // start communication with IMU
//   status = IMU.begin();
//   if (status < 0) {
//     Serial.println("IMU initialization unsuccessful");
//     Serial.println("Check IMU wiring or try cycling power");
//     Serial.print("Status: ");
//     Serial.println(status);
//     while (1) {}
//   }
// }

// void loop() {
//   // read the sensor
//   IMU.readSensor();

//   // get angle using accel
//   float accY = IMU.getAccelY_mss(); // ms^-2
//   float accZ = IMU.getAccelZ_mss();

//   float accAngle = atan2(accY, accZ) * RAD_TO_DEG + 180;

//   // left side
//   if(accAngle > 180) {
//     accAngle -= 360;
//   }

//   // get angle using gyro
//   float gyroX = IMU.getGyroX_rads() * RAD_TO_DEG; //deg^-1
//   // float gyroRate = map(gyroX, -32768, 32767, -250, 250);
//   float gyroAngle = prevAngle + gyroX * 4 / 1000;

//   // complementary filter
//   float currentAngle = 0.9934 * gyroAngle + 0.0066 * accAngle;

//   prevAngle = gyroAngle;

//   // display the data

//   Serial.print(IMU.getGyroX_rads(), 6);
//   Serial.print("\t");
//   Serial.print(accAngle, 6);
//   Serial.print("\t");
//   Serial.print(gyroAngle, 6);
//   Serial.print("\t");
//   Serial.println(currentAngle, 6);

//   // Serial.print(IMU.getAccelX_mss(), 6);
//   // Serial.print("\t");
//   // Serial.print(IMU.getAccelY_mss(), 6);
//   // Serial.print("\t");
//   // Serial.print(IMU.getAccelZ_mss(), 6);
//   // Serial.print("\t");
//   // Serial.print(IMU.getGyroX_rads(), 6);
//   // Serial.print("\t");
//   // Serial.print(IMU.getGyroY_rads(), 6);
//   // Serial.print("\t");
//   // Serial.println(IMU.getGyroZ_rads(), 6);

//   delay(4);
// }