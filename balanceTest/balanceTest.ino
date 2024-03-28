#include <ESP32Servo.h>
#include "MPU9250.h"
#include "math.h"

//accel stuff
float accY, accZ;
float accAngle;

//gyro stuff
float gyroX;
float gyroAngle = 0;
unsigned long currTime, prevTime = 0, loopTime;

float prevAngle = 0, currentAngle;

// PID shinenigans
float targetAngle = 7;
float errorSum = 0, prevError, prevIntegral;
float kP = 2;  // proportional
float kI = 0;   // integral
float kD = 0;   // derivative

MPU9250 IMU(Wire, 0x68);
int status;

// initiate the two motors
Servo leftMotor;
Servo rightMotor;

// receiver stuff
int receivePin1 = 16;
int receivePin2 = 17;
int receivePin3 = 18;
int receivePin4 = 19;
unsigned long pulseDuration1;
unsigned long pulseDuration2;
unsigned long pulseDuration3;
unsigned long pulseDuration4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  pinMode(receivePin1, INPUT);
  pinMode(receivePin2, INPUT);
  pinMode(receivePin3, INPUT);
  pinMode(receivePin4, INPUT);

  // attach left motor to pin 32
  leftMotor.attach(32, 1000, 2000);
  // attach right motor to pin 4
  rightMotor.attach(4, 1000, 2000);
}

void loop() {
  // read the IMU
  IMU.readSensor();

  // get angle using accel
   accY = IMU.getAccelY_mss();  // ms^-2
   accZ = IMU.getAccelZ_mss();

   accAngle = atan2(accY, accZ) * RAD_TO_DEG + 180;

  // left side
  if (accAngle > 180) {
    accAngle -= 360;
  }

  // get angle using gyro
   gyroX = IMU.getGyroX_rads() * RAD_TO_DEG;  //deg^-1
  // float gyroRate = map(gyroX, -32768, 32767, -250, 250);
   gyroAngle = prevAngle + gyroX * 4 / 1000;

  // complementary filter
  float currentAngle = 0.9934 * gyroAngle + 0.0066 * accAngle;

  // PID
  float error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  float i = errorSum * 4;
  float d = (currentAngle - prevAngle) / 4;
  float output = kP * error + kI * i + kD * d;

  prevError = error;
  prevIntegral = i;
  prevAngle = currentAngle;

  // get pulse durations from reciever
  pulseDuration1 = pulseIn(receivePin1, HIGH);  // returns in microseconds
  pulseDuration2 = pulseIn(receivePin2, HIGH);
  pulseDuration3 = pulseIn(receivePin3, HIGH);
  pulseDuration4 = pulseIn(receivePin4, HIGH);

  // write combined pulse duration to ESCs
  leftMotor.write(pulseDuration3);
  rightMotor.write(pulseDuration3);

  Serial.print(currentAngle);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  // Serial.print(pulseDuration1);
  // Serial.print(" ");
  // Serial.print(pulseDuration2);
  // Serial.print(" ");
  Serial.println(pulseDuration3);
  // Serial.print(" ");
  // Serial.println(pulseDuration4);

  delay(4);
}
