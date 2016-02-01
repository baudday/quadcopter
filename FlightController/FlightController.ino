#include <Wire.h>

#define MOTOR_1 10 // FR
#define MOTOR_2 11 // BR
#define MOTOR_3 12 // BL
#define MOTOR_4 50 // FL

#define CH1_PIN 3  // Roll
#define CH2_PIN 2  // Throttle
#define CH3_PIN 19 // Pitch
#define CH4_PIN 18 // Yaw

#define LED     9 // Status LED

#define ITG_ADDR  (0xD0 >> 1)
#define ITG_SAMPLE_RATE_DIV_REG 0x15
#define ITG_SAMPLE_RATE_DIV_VAL B00001001
#define ITG_DLPF_REG  0X16
#define ITG_DLPF_VAL  B00011011

float pid_p_gain_roll = 0;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 35;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = .02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

int roll_input, throttle_input, pitch_input, yaw_input;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long current_time, t1, t2, t3, t4, esc_timer, esc_loop_timer, loop_timer;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

int cal_int;
double gyro_roll,     gyro_pitch,     gyro_yaw,
       gyro_roll_deg, gyro_pitch_deg, gyro_yaw_deg,
       gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte low, high;

int esc_1, esc_2, esc_3, esc_4;
int battery_voltage;
int throttle_setpoint;

int start = 0;

void setup() {
  Wire.begin();
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);
  pinMode(LED, OUTPUT);

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), roll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4_PIN), yaw, CHANGE);

  digitalWrite(LED, HIGH);
  setup_gyro();
  delay(250);
  calibrate_gyro();

  while (throttle_input < 990 || throttle_input > 1020 || yaw_input < 1400) {
    PORTB |= B11111000;
    delayMicroseconds(1000);
    PORTB &= B00000111;
    delay(3);
  }

  battery_voltage = (analogRead(4) + 80) * .8211143;
  digitalWrite(LED, LOW);
}

void loop() {
  read_gyro();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  if(throttle_input < 1050 && yaw_input < 1050) start = 1;
  if(start == 1 && throttle_input < 1050 && yaw_input > 1450){
    start = 2;
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  if(start == 2 && throttle_input < 1050 && yaw_input > 1950) start = 0;

  pid_roll_setpoint = 0;
  if(roll_input > 1508) pid_roll_setpoint = (roll_input - 1508)/3.0;
  else if(roll_input < 1492) pid_roll_setpoint = (roll_input - 1492)/3.0;

  pid_pitch_setpoint = 0;
  if(pitch_input > 1508) pid_pitch_setpoint = (pitch_input - 1508)/3.0;
  else if(pitch_input < 1492) pid_pitch_setpoint = (pitch_input - 1492)/3.0;
  
  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(yaw_input > 1050) { //Do not yaw when turning off the motors.
    if(yaw_input > 1508) pid_yaw_setpoint = (yaw_input - 1508)/3.0;
    else if(yaw_input < 1492) pid_yaw_setpoint = (yaw_input - 1492)/3.0;
  }

  calculate_pid();

  battery_voltage = battery_voltage * 0.92 + (analogRead(4) + 80) * .08 * .8211143;
  if(battery_voltage < 700 && battery_voltage > 550) digitalWrite(LED, HIGH);

  throttle_setpoint = throttle_input;

  if (start == 2){                                                          //The motors are started.
    if (throttle_setpoint > 1800) throttle_setpoint = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle_setpoint - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle_setpoint + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle_setpoint + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle_setpoint - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 550){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
    
    if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1200;                                         //Keep the motors running.
    
    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }
  
  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTB |= B11111000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTB >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTB &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTB &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTB &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTB &= B01110111;                //Set digital output 7 to low if the time is expired.
  }
}

/* =====GYRO STUFF===== */
void setup_gyro() {
  Wire.beginTransmission(ITG_ADDR);
  Wire.write(ITG_SAMPLE_RATE_DIV_REG);
  Wire.write(ITG_SAMPLE_RATE_DIV_VAL);
  Wire.endTransmission();

  Wire.beginTransmission(ITG_ADDR);
  Wire.write(ITG_DLPF_REG);
  Wire.write(ITG_DLPF_VAL);
  Wire.endTransmission();
}

void calibrate_gyro() {
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    read_gyro();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;
    delay(4);
  }
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
}

void read_gyro() {
  Wire.beginTransmission(ITG_ADDR);
  Wire.write(0x1D);
  Wire.endTransmission();
  Wire.beginTransmission(ITG_ADDR);
  Wire.requestFrom(ITG_ADDR, 6);
  while(Wire.available() < 6);

  high = Wire.read();
  low = Wire.read();
  gyro_pitch = ((high<<8)|low);
  gyro_pitch *= -1;
  if (cal_int == 2000) gyro_pitch -= gyro_pitch_cal;
  gyro_pitch_deg = gyro_pitch/57.14286;

  high = Wire.read();
  low = Wire.read();
  gyro_roll = ((high<<8)|low);
  gyro_roll *= -1;
  if (cal_int == 2000) gyro_roll -= gyro_roll_cal;
  gyro_roll_deg = gyro_roll/57.14286;

  high = Wire.read();
  low = Wire.read();
  gyro_yaw = ((high<<8)|low);
  gyro_yaw *= -1;
  if (cal_int == 2000) gyro_yaw -= gyro_yaw_cal;
  gyro_yaw_deg = gyro_yaw/57.14286;

  Wire.endTransmission();
}

/* ===== PID STUFF =====*/
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}


/* =====TRANSMITTER STUFF===== */
void roll() {
  current_time = micros();
  if(PINE & B00100000) {
    t1 = current_time;
  } else {
    roll_input = current_time - t1;
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

void pitch() {
  current_time = micros();
  if(PIND & B00000100) {
    t3 = current_time;
  } else {
    pitch_input = current_time - t3;
  }
}

void yaw() {
  current_time = micros();
  if(PIND & B00001000) {
    t4 = current_time;
  } else {
    yaw_input = current_time - t4;
  }
}
