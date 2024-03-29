#include <ESP32Servo.h>

Servo leftMotor;
Servo rightMotor;

int ReceiverPin1 = 16;
int ReceiverPin2 = 17;
int ReceiverPin3 = 18;
int ReceiverPin4 = 19;
unsigned long ReceiverValues[] = { 0, 0, 0, 0 };

float InputThrottle;

void read_receiver(void) {
  // get pulse durations
  // returns in microseconds
  ReceiverValues[0] = pulseIn(ReceiverPin1, HIGH);
  ReceiverValues[1] = pulseIn(ReceiverPin2, HIGH);
  ReceiverValues[2] = pulseIn(ReceiverPin3, HIGH);
  ReceiverValues[3] = pulseIn(ReceiverPin4, HIGH);
}

void setup() {
  Serial.begin(57600);
  
  // set receiver pins to input
  pinMode(ReceiverPin1, INPUT);
  pinMode(ReceiverPin2, INPUT);
  pinMode(ReceiverPin3, INPUT);
  pinMode(ReceiverPin4, INPUT);

  // attach ESCs
  leftMotor.attach(32, 1000, 2000);
  rightMotor.attach(4, 1000, 2000);

  // avoid uncontrolled motor start
  while (ReceiverValues[2] < 1020 || ReceiverValues[2] > 1050) {
    read_receiver();
    delay(4);
  }
}

void loop() {
  // get signals from receiver
  read_receiver();

  InputThrottle = ReceiverValues[2];

  // write pulse duration to ESC
  leftMotor.write(InputThrottle);
  rightMotor.write(InputThrottle);

  Serial.print(ReceiverValues[0]);
  Serial.print(" ");
  Serial.print(ReceiverValues[1]);
  Serial.print(" ");
  Serial.print(ReceiverValues[2]);
  Serial.print(" ");
  Serial.print(ReceiverValues[3]);
  Serial.print(" ");
  Serial.println(InputThrottle);
}
