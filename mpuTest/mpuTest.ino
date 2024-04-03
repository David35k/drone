#include "Wire.h"

// rate variables
float RateRoll, RatePitch, RateYaw;

// accel variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// calibration variables
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

void gyro_signals(void) {
  // switch on low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // configure accel
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  

  // configure gyro
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
  // this will take 2 seconds, don't move quad while calibrating
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