#include "Wire.h"

// motor pins
// right front = 1, left rear = 2, left front = 3, right rear = 4 <- not pin numbers, just motor number
int MotorPins[4] = { 26, 27, 32, 33 };

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

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// define loop time
uint32_t LoopTimer;

// rate variables
float RateRoll, RatePitch, RateYaw;

// accel variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// PID variables - RATES
float DesiredRatePitch, DesiredRateRoll, DesiredRateYaw;
float ErrorRatePitch, ErrorRateRoll, ErrorRateYaw;
float InputThrottle, InputPitch, InputRoll, InputYaw;
float PrevErrorRatePitch, PrevErrorRateRoll, PrevErrorRateYaw;
float PrevItermRatePitch, PrevItermRateRoll, PrevItermRateYaw;
float PIDReturn[] = { 0, 0, 0 };

// PID parameters - RATES
float PRatePitch, PRateRoll = 1;
float PRateYaw = 2;
float IRatePitch, IRateRoll = 3.5;
float IRateYaw = 12;
float DRatePitch, DRateRoll = 0.01;
float DRateYaw = 0;

// kalman filter shinenigans
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;

// output of filter {angle prediction, uncertainty of the prediction}
// 1D means 1 dimensional
float Kalman1DOutput[] = { 0, 0 };

// PID varaibles - ANGLES
float DesiredAnglePitch, DesiredAngleRoll;
float ErrorAnglePitch, ErrorAngleRoll;
float PrevErrorAnglePitch, PrevErrorAngleRoll;
float PrevItermAnglePitch, PrevItermAngleRoll;

// PID - ANGLES
float PAnglePitch, PAngleRoll = 1;
float IAnglePitch, IAngleRoll = 0.2;
float DAnglePitch, DAngleRoll = 0.3;

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
  // for some reason constrain didnt work
  if (Pulses1 <= 2000 && Pulses1 >= 1000) {
    ReceiverValues[0] = Pulses1;
  }
  if (Pulses2 <= 2000 && Pulses2 >= 1000) {
    ReceiverValues[1] = Pulses2;
  }
  if (Pulses3 <= 2000 && Pulses3 >= 1000) {
    ReceiverValues[2] = Pulses3;
  }
  if (Pulses4 <= 2000 && Pulses4 >= 1000) {
    ReceiverValues[3] = Pulses4;
  }
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

  Serial.begin(115200);

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
  ledcSetup(0, 250, 12);           // channel 0, 250Hz frequency, 12bit resolution - 0 and 4095 which corresponds to 0us and 4000us
  ledcAttachPin(MotorPins[0], 0);  // assign channel 0 to esc pin

  ledcSetup(1, 250, 12);           // channel 1, 250Hz frequency, 12bit resolution - 0 and 4095 which corresponds to 0us and 4000us
  ledcAttachPin(MotorPins[1], 1);  // assign channel 1 to esc pin

  ledcSetup(2, 250, 12);           // channel 2, 250Hz frequency, 12bit resolution - 0 and 4095 which corresponds to 0us and 4000us
  ledcAttachPin(MotorPins[2], 2);  // assign channel 2 to esc pin

  ledcSetup(3, 250, 12);           // channel 3, 250Hz frequency, 12bit resolution - 0 and 4095 which corresponds to 0us and 4000us
  ledcAttachPin(MotorPins[3], 3);  // assign channel 3 to esc pin
  
  delay(250);  // cheeky delay lol

  // avoid uncontrolled motor start - make sure this is not commented when testing with props
  while (ReceiverValues[2] < 1020 || ReceiverValues[2] > 1050) {
    update_receiver_values();
    delay(4);
  }

  Serial.println("done setup yay");

  // finished setup hooray - should be LED to indicate this lol
}

void loop() {
  LoopTimer = micros();

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

  //kalman for roll
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  // calculate desired angles
  // angle range: -50 to 50 deg
  DesiredAnglePitch = -0.1 * (ReceiverValues[1] - 1500) + 7; // some offset lawl
  DesiredAngleRoll = -0.1 * (ReceiverValues[0] - 1500); // <<< CHECK THIS
  
  // yaw will still use rates!
  DesiredRateYaw = -0.15 * (ReceiverValues[3] - 1500);
  
  // get throttle input
  InputThrottle = ReceiverValues[2];

  // calculate difference between desired and actual angles
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;

  // calcualte desired rotation rates based on angle error
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  // calculate the errors that will be corrected by PID for RATE
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // execute PID calculations
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  // limit throttle output to leave room for PID corrections
  if (InputThrottle > 1800) InputThrottle = 1800;

  // use quadcopter dynamics equation to determine motor speeds - inverted lawl
  MotorInput1 = 1.024 * (InputThrottle + InputPitch + InputRoll - InputYaw); // front right
  MotorInput2 = 1.024 * (InputThrottle - InputPitch - InputRoll - InputYaw); // back left
  MotorInput3 = 1.024 * (InputThrottle + InputPitch - InputRoll + InputYaw); // front left
  MotorInput4 = 1.024 * (InputThrottle - InputPitch + InputRoll + InputYaw); // back right

  // make sure they dont exceed 2000 microseconds
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;

  // make sure you can turn the motors off lol
  int ThrottleCutoff = 1000;
  if (ReceiverValues[2] < 1050) {
    MotorInput1 = ThrottleCutoff;
    MotorInput2 = ThrottleCutoff;
    MotorInput3 = ThrottleCutoff;
    MotorInput4 = ThrottleCutoff;
    reset_pid();
  }

  // send signal to motors
  ledcWrite(0, MotorInput1);  // write to channel 0 which is MOTOR 1
  ledcWrite(1, MotorInput2);  // write to channel 1 which is MOTOR 2
  ledcWrite(2, MotorInput3);  // write to channel 2 which is MOTOR 3
  ledcWrite(3, MotorInput4);  // write to channel 3 which is MOTOR 4
  
  // debugging shi
  // Serial.printf("angle pitch: %f, desired angle pitch: %f", KalmanAnglePitch , DesiredAnglePitch); // for testing pitch
  // Serial.printf("angle roll: %f, desired angle roll: %f", KalmanAngleRoll, DesiredAngleRoll); // for testing roll
  // Serial.printf("rate yaw: %f, desired rate yaw : %f", RateYaw, DesiredRateYaw); // for testing yaw
  // Serial.printf("input throttle: %f, motor1: %f, motor2: %f, motor3: %f, motor4: %f", InputThrottle, MotorInput1, MotorInput2, MotorInput3, MotorInput4); // for testing yaw
  // Serial.println("");

  // finish 250Hz control loop
  while (micros() - LoopTimer < 4000) {
    asm("");
  }
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