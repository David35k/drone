// this simple balance test only uses two motors and the pitch angle
// they are mounted on a seesaw-like stand

#include "Wire.h"

// motor pins
// right front = 1, left rear = 2, left front = 3, right rear = 4 <- not pin numbers, just motor number
int MotorPins[4] = { 4, 0, 0, 32 };

// calibration variables
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

int ReceiverPin1 = 16;
int ReceiverPin2 = 17;
int ReceiverPin3 = 18;
int ReceiverPin4 = 19;
float ReceiverValues[] = { 0, 0, 0, 0 };

// stuff for calculating pulse widths lol
volatile long StartTime1 = 0, StartTime2 = 0, StartTime3 = 0, StartTime4 = 0;
volatile long CurrentTime1 = 0, CurrentTime2 = 0, CurrentTime3 = 0, CurrentTime4 = 0;
volatile long Pulses1 = 0, Pulses2 = 0, Pulses3 = 0, Pulses4 = 0;

float MotorInput1, MotorInput4;

// define loop time
uint32_t LoopTimer;

// rate variables
float RateRoll, RatePitch, RateYaw;

// accel variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// all variables needed for PID - ONLY PITCH - rates!
float DesiredRatePitch;
float ErrorRatePitch;
float InputThrottle, InputPitch;
float PrevErrorRatePitch;
float PrevItermRatePitch;
float PIDReturn[] = { 0, 0, 0 };

// PID parameters
float PRatePitch = 0.6;
float IRatePitch = 3.5;
float DRatePitch = 0.01;

// kalman filter shinenigans
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;

// output of filter {angle prediction, uncertainty of the prediction}
// 1D means 1 dimensional
float Kalman1DOutput[] = { 0, 0 };

// PID varaibles only pitch ANGLE
float DesiredAnglePitch;
float ErrorAnglePitch;
float PrevErrorAnglePitch;
float PrevItermAnglePitch;

// PID parameters
float PAnglePitch = 0.1;
float IAnglePitch = 0;
float DAnglePitch = 0;

// function definitions
inline void kalman_1d(float, float, float, float) __attribute__((always_inline));
inline void update_receiver_values() __attribute__((always_inline));
inline void gyro_signals() __attribute__((always_inline));
inline void pid_equation(float, float, float, float, float, float) __attribute__((always_inline));
inline void reset_pid() __attribute__((always_inline));

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

// update the measured values
void update_receiver_values() {
  ReceiverValues[0] = Pulses1;
  ReceiverValues[1] = Pulses2;
  ReceiverValues[2] = Pulses3;
  ReceiverValues[3] = Pulses4;
}

void gyro_signals() {
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
  RateRoll = (float)GyroX / 65.5;  // i have no idea why the rates are half but if it works it works
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

// PID function
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  // PID calculation
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
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

void reset_pid() {
  PrevErrorRatePitch = 0;
  PrevItermRatePitch = 0;

  PrevErrorAnglePitch = 0;
  PrevItermAnglePitch = 0;
}

void setup() {

  // should have some LED to indicate status

  Serial.begin(9600);

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
  pinMode(ReceiverPin1, INPUT_PULLUP);
  pinMode(ReceiverPin2, INPUT_PULLUP);
  pinMode(ReceiverPin3, INPUT_PULLUP);
  pinMode(ReceiverPin4, INPUT_PULLUP);

  // attach interrupts to radio receiver pins
  attachInterrupt(digitalPinToInterrupt(ReceiverPin1), PulseTimer1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverPin2), PulseTimer2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverPin3), PulseTimer3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverPin4), PulseTimer4, CHANGE);

  // set up comminication with ESCs
  pinMode(MotorPins[0], OUTPUT);   // MOTOR NUMBER 1
  ledcSetup(0, 250, 12);           // channel 0, 250Hz frequency, 12bit resolution - 0 and 4095 which corresponds to 0us and 4000us
  ledcAttachPin(MotorPins[0], 0);  // assign channel 0 to esc pin

  pinMode(MotorPins[3], OUTPUT);   // MOTOR NUMBER 4
  ledcSetup(3, 250, 12);           // channel 3, 250Hz frequency, 12bit resolution - 0 and 4095 which corresponds to 0us and 4000us
  ledcAttachPin(MotorPins[3], 0);  // assign channel 0 to esc pin

  delay(250);  // cheeky delay lol

  // ONLY WHILE TESTING MAKE SURE THIS IS ENABLES WHEN USING PROPS

  // avoid uncontrolled motor start
  // while (ReceiverValues[2] < 1020 || ReceiverValues[2] > 1050) {
  //   update_receiver_values();
  //   delay(4);
  // }

  // finished setup hooray - should be LED to indicate this lol

  LoopTimer = micros();
}

void loop() {
  gyro_signals();

  // use calibration values to correct measurements
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // update signals from receiver
  update_receiver_values();

  //kalman for pitch
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // calculate desired angles
  // angle range: -50 to 50 deg
  DesiredAnglePitch = 0.1 * (ReceiverValues[1] - 1500);

  // get throttle input
  InputThrottle = ReceiverValues[2];

  // calculate difference between desired and actual angles
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

  // calcualte desired rotation rates based on angle error
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  // calculate the errors that will be corrected by PID for RATE
  ErrorRatePitch = DesiredRatePitch - RateRoll;

  // execute PID calculations
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  // limit throttle output to leave room for PID corrections
  if (InputThrottle > 1800) InputThrottle = 1800;

  // use quadcopter dynamics equation to determine motor speeds
  MotorInput1 = 1.024 * (InputThrottle - InputPitch);
  MotorInput4 = 1.024 * (InputThrottle + InputPitch);

  // make sure they dont exceed 2000 microseconds
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;

  // make sure you can turn the motors off lol
  int ThrottleCutoff = 1000;
  if (ReceiverValues[2] < 1050) {
    MotorInput1 = ThrottleCutoff;
    MotorInput4 = ThrottleCutoff;
    reset_pid();
  }

  // send signal to motors
  ledcWrite(0, MotorInput1);  // write to channel 0 which is MOTOR 1
  ledcWrite(3, MotorInput4);  // write to channel 3 which is MOTOR 4

  // debugging shi
  Serial.println(KalmanAnglePitch);

  // finish 250Hz control loop
  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}

// silly loop timer functions, if it works it works type shi

void PulseTimer1() {
  CurrentTime1 = micros();
  if (CurrentTime1 > StartTime1) {
    Pulses1 = CurrentTime1 - StartTime1;
    StartTime1 = CurrentTime1;
  }
}
void PulseTimer2() {
  CurrentTime2 = micros();
  if (CurrentTime2 > StartTime2) {
    Pulses2 = CurrentTime2 - StartTime2;
    StartTime2 = CurrentTime2;
  }
}
void PulseTimer3() {
  CurrentTime3 = micros();
  if (CurrentTime3 > StartTime3) {
    Pulses3 = CurrentTime3 - StartTime3;
    StartTime3 = CurrentTime3;
  }
}
void PulseTimer4() {
  CurrentTime4 = micros();
  if (CurrentTime4 > StartTime4) {
    Pulses4 = CurrentTime4 - StartTime4;
    StartTime4 = CurrentTime4;
  }
}