
// trying to do it without using servo library

int escPin = 32;

int ReceiverPin1 = 16;
int ReceiverPin2 = 17;
int ReceiverPin3 = 18;
int ReceiverPin4 = 19;
unsigned int ReceiverValues[] = { 0, 0, 0, 0 };

volatile long StartTime1 = 0, StartTime2 = 0, StartTime3 = 0, StartTime4 = 0;
volatile long CurrentTime1 = 0, CurrentTime2 = 0, CurrentTime3 = 0, CurrentTime4 = 0;
volatile long Pulses1 = 0, Pulses2 = 0, Pulses3 = 0, Pulses4 = 0;

float InputThrottle;

void update_receiver_values() {
  ReceiverValues[0] = Pulses1;
  ReceiverValues[1] = Pulses2;
  ReceiverValues[2] = Pulses3;
  ReceiverValues[3] = Pulses4;
}

void setup() {
  Serial.begin(57600);

  // set receiver pins to input
  pinMode(ReceiverPin1, INPUT_PULLUP);
  pinMode(ReceiverPin2, INPUT_PULLUP);
  pinMode(ReceiverPin3, INPUT_PULLUP);
  pinMode(ReceiverPin4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ReceiverPin1), PulseTimer1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverPin2), PulseTimer2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverPin3), PulseTimer3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ReceiverPin4), PulseTimer4, CHANGE);

  // set up comminication with esc
  pinMode(escPin, OUTPUT);
  ledcSetup(0, 250, 12);     // channel 0, 250Hz frequency, 12bit resolution - 0 and 4095 which corresponds to 0us and 4000us
  ledcAttachPin(escPin, 0);  // assign channel 0 to esc pin
  delay(250);

  // avoid uncontrolled motor start
  while (ReceiverValues[2] < 1020 || ReceiverValues[2] > 1050) {
    update_receiver_values();
    delay(4);
  }
}

void loop() {
  // get signals from receiver
  update_receiver_values();

  if(ReceiverValues[2] > 2000) {
    ReceiverValues[2] = 1999;
  }

  InputThrottle = ReceiverValues[2];

  ledcWrite(0, InputThrottle * 1.024);  // write to channel 0

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