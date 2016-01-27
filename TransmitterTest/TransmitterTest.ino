#define CH1_PIN 3   // Roll
#define CH2_PIN 2   // Throttle
#define CH3_PIN 19  // Pitch
#define CH4_PIN 18  // Yaw

volatile int roll_input, throttle_input, pitch_input, yaw_input;
volatile long t1, t2, t3, t4;

void setup() {
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), roll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4_PIN), yaw, CHANGE);

  Serial.begin(9600);
}

void loop() {
  delay(250);
  print_signals();
}

void roll() {
  if(PINE & B00100000) {
    t1 = micros();
  } else {
    roll_input = micros() - t1;
  }
}

void throttle() {
  if(PINE & B00010000) {
    t2 = micros();
  } else {
    throttle_input = micros() - t2;
  }
}

void pitch() {
  if(PIND & B00000100) {
    t3 = micros();
  } else {
    pitch_input = micros() - t3;
  }
}

void yaw() {
  if(PIND & B00001000) {
    t4 = micros();
  } else {
    yaw_input = micros() - t4;
  }
}

void print_signals() {
  Serial.print("Roll:");
  if(roll_input - 1480 < 0) Serial.print("<<<");
  else if(roll_input - 1520 > 0) Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(roll_input);

  Serial.print(" Throttle: ");
  if(throttle_input - 1480 < 0) Serial.print("vvv");
  else if(throttle_input - 1520 > 0) Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(throttle_input);

  Serial.print(" Pitch: ");
  if(pitch_input - 1480 < 0) Serial.print("^^^");
  else if(pitch_input - 1520 > 0) Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(pitch_input);

  Serial.print(" Yaw: ");
  if(yaw_input - 1480 < 0) Serial.print("<<<");
  else if(yaw_input - 1520 > 0) Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(yaw_input);
}

