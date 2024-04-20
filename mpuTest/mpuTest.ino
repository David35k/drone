#include "Wire.h"

// dude i do not understand how the kalman filter works 💀

float LoopTimer;

// rate variables
float RateRoll, RatePitch, RateYaw;
float RollAngleGyro, PitchAngleGyro;
float PrevAngleRoll, PrevAnglePitch;

// accel variables
float AccX, AccY, AccZ;
float AngleRollAcc, AnglePitchAcc;

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
  Wire.write(0x10);
  Wire.endTransmission();

  // configure gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  // access registers storing accel values
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  // store accel measurements
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // convert to physical values
  AccX = (float)AccXLSB / 4096 - 0.01;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 + 0.01;

  // epic math shinenigans
  AngleRollAcc = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (PI / 180);
  AnglePitchAcc = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (PI / 180);

  // access registers storing gyro values
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  // store gyro measurements
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // get in degrees per second
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  LoopTimer = micros();
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
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
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

  // get angle from gyro!!
  RollAngleGyro = PrevAngleRoll + RateRoll * 0.004;
  PitchAngleGyro = PrevAnglePitch + RatePitch * 0.004;

  // epic complementary filter!!!
  AngleRoll = 0.9934 * RollAngleGyro + 0.0066 * AngleRollAcc;
  AnglePitch = 0.9934 * PitchAngleGyro + 0.0066 * AnglePitchAcc;

  // set prev variables for next iteration
  PrevAngleRoll = AngleRoll;
  PrevAnglePitch = AnglePitch;

  // print values
  Serial.print("ratepitch:");
  Serial.print(RatePitch * 0.004);
  Serial.print(" angleRollGyro:");
  Serial.print(RollAngleGyro);
  Serial.print(" anglePitchGyro:");
  Serial.print(PitchAngleGyro);
  Serial.print(AccZ);
  Serial.print(" angleRollAcc:");
  Serial.print(AngleRollAcc);
  Serial.print(" anglePitchAcc:");
  Serial.print(AnglePitchAcc);
  Serial.print(" AngleRoll:");
  Serial.print(AngleRoll);
  Serial.print(" AnglePitch:");
  Serial.println(AnglePitch);

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}