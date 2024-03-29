// this simple balance test only uses two motors and the pitch angle
// they are mounted on a seesaw-like stand

#include "Wire.h"

float RateRoll, RatePitch, RateYaw;

// calibration variables
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

#include <ESP32Servo.h>

int ReceiverPin1 = 16;
int ReceiverPin2 = 17;
int ReceiverPin3 = 18;
int ReceiverPin4 = 19;
unsigned long ReceiverValues[] = { 0, 0, 0, 0 };

Servo Motor1, Motor2;

float MotorInput1, MotorInput2;

// define loop time
uint32_t LoopTimer;

// all variables needed for PID - ONLY PITCH
float DesiredRatePitch;
float ErrorRatePitch;
float InputThrottle, InputPitch;
float PrevErrorRatePitch;
float PrevItermRatePitch;
float PIDReturn[] = { 0, 0, 0 };

// PID parameters
float PRatePitch = 0.6;
float IRatePitch = 3.5;
float DRatePitch = 0.03;

// gyro measurements function
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

// PID function
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  // PID calculation
  float Pterm = P * Error;
  float Iterm = PrevIterm = I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  // return output from PID function
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
  PrevErrorRatePitch = 0;
  PrevItermRatePitch = 0;
}

// reads pulse widths from receiver in microseconds (1000-2000)
void read_receiver(void) {
  ReceiverValues[0] = pulseIn(ReceiverPin1, HIGH);
  ReceiverValues[1] = pulseIn(ReceiverPin2, HIGH);
  ReceiverValues[2] = pulseIn(ReceiverPin3, HIGH);
  ReceiverValues[3] = pulseIn(ReceiverPin4, HIGH);
}

void setup() {

  // should have some LED to indicate status

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

  // set receiver pins to input
  pinMode(ReceiverPin1, INPUT);
  pinMode(ReceiverPin2, INPUT);
  pinMode(ReceiverPin3, INPUT);
  pinMode(ReceiverPin4, INPUT);

  // attach ESCs
  Motor1.attach(4, 1000, 2000);
  Motor2.attach(32, 1000, 2000);

  // avoid uncontrolled motor start
  while (ReceiverValues[2] < 1020 || ReceiverValues[2] > 1050) {
    read_receiver();
    delay(4);
  }

  // finished setup hooray

  LoopTimer = micros();
}

void loop() {
  // get gyro readings
  gyro_signals();

  // use calibration values to correct measurements
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // get receiver readings
  read_receiver();

  // calculate desired rates based on input
  // rate range: -75 to 75 deg/s
  DesiredRatePitch = 0.15 * (ReceiverValues[1] - 1500);
  InputThrottle = ReceiverValues[2];

  // calculate the errors that will be corrected by PID
  ErrorRatePitch = DesiredRatePitch - RateRoll;

  // execute PID calculations
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  // limit throttle output to leave room for PID corrections
  if(InputThrottle > 1800) InputThrottle = 1800;

  // use quadcopter dynamics equation to determine motor speeds
  MotorInput1 = InputThrottle - InputPitch;
  MotorInput2 = InputThrottle + InputPitch;

  // make sure they dont exceed 2000 microseconds
  if(MotorInput1 > 2000) MotorInput1 = 1999;
  if(MotorInput2 > 2000) MotorInput2 = 1999;

  // keep motors running and minimum 18%
  int ThrottleIdle = 1180;
  if(MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if(MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;

  // make sure you can turn the motors off lol
  int ThrottleCutoff = 1000;
  if(ReceiverValues[2] < 1050) {
    MotorInput1 = ThrottleCutoff;
    MotorInput2 = ThrottleCutoff;
    reset_pid();
  }

  // send signal to motors
  Motor1.write(MotorInput1);
  Motor2.write(MotorInput2);

  // finish 250Hz control loop
  while(micros() - LoopTimer < 4000);
  LoopTimer = micros();
}