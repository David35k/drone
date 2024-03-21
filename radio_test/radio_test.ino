#include <ESP32Servo.h>

Servo ESC;

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
  Serial.begin(9600);
  pinMode(receivePin1, INPUT);
  pinMode(receivePin2, INPUT);
  pinMode(receivePin3, INPUT);
  pinMode(receivePin4, INPUT);

  // attach ESC to pin 21
  ESC.attach(21, 1000, 2000);
}

void loop() {
  // get pulse durations
  pulseDuration1 = pulseIn(receivePin1, HIGH);  // returns in microseconds
  pulseDuration2 = pulseIn(receivePin2, HIGH);
  pulseDuration3 = pulseIn(receivePin3, HIGH);
  pulseDuration4 = pulseIn(receivePin4, HIGH);

  // write pulse duration to ESC
  ESC.write(pulseDuration3);

  Serial.print(pulseDuration1);
  Serial.print(" ");
  Serial.print(pulseDuration2);
  Serial.print(" ");
  Serial.print(pulseDuration3);
  Serial.print(" ");
  Serial.println(pulseDuration4);
}
