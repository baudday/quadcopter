#define MOTOR_1 10 // FR
#define MOTOR_2 11 // BR
#define MOTOR_3 12 // BL
#define MOTOR_4 50 // FL
#define CH1_PIN 3  // Roll
#define CH2_PIN 2  // Throttle
#define CH3_PIN 19 // Pitch
#define CH4_PIN 18 // Yaw

int throttle_input;
int throttle_counter, start;
unsigned long throttle_timer, esc_loop_timer;
unsigned long zero_timer, t2, current_time;

void setup() {
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);
  pinMode(CH2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), throttle, CHANGE);
  while (throttle_input < 990 || throttle_input > 1020) {
    PORTB |= B11111000;
    delayMicroseconds(1000);
    PORTB &= B00000111;
    delay(3);
  }
  zero_timer = micros();
}

void loop() {
  while(zero_timer + 4000 > micros());                       //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.
  PORTB |= B11111000;
  throttle_timer = throttle_input + zero_timer;
  
  while (PORTB >= 16) {
    esc_loop_timer = micros();                               //Check the current time.
    if (throttle_timer <= esc_loop_timer) PORTB &= B11101111; //When the delay time is expired, digital port 8 is set low.
    if (throttle_timer <= esc_loop_timer) PORTB &= B11011111; //When the delay time is expired, digital port 8 is set low.
    if (throttle_timer <= esc_loop_timer) PORTB &= B10111111; //When the delay time is expired, digital port 8 is set low.
    if (throttle_timer <= esc_loop_timer) PORTB &= B01110111; //When the delay time is expired, digital port 8 is set low.
  }
}

void throttle() {
  current_time = micros();
  if(PINE & B00010000) {
    t2 = current_time;
  } else {
    throttle_input = current_time - t2;
  }
}

