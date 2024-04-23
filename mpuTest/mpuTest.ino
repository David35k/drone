#include "Wire.h"

// dude i do not understand how the kalman filter works 💀

// DEBUGGIN SHI
float prevGyroAnglePitch;
float gyroAnglePitch;

float LoopTimer;

// rate variables
float RateRoll, RatePitch, RateYaw;

// accel variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// calibration variables
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

// kalman filter shinenigans
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;

// output of filter {angle prediction, uncertainty of the prediction}
// 1D means 1 dimensional
float Kalman1DOutput[] = { 0, 0 };

// wtf is going on bro 😭
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

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
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (PI / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (PI / 180);

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

  prevGyroAnglePitch = AnglePitch;
}

void loop() {

  LoopTimer = micros();

  gyro_signals();

  // use calibration values to correct measurements
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // get gyro angle
  gyroAnglePitch = prevGyroAnglePitch + RatePitch * 0.004;
  prevGyroAnglePitch = gyroAnglePitch;

  // kalman for roll
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  // for pitch
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // print values
  Serial.println(KalmanAnglePitch);

  while (micros() - LoopTimer < 4000) {
    asm("");
  }
}