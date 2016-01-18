#include <Wire.h>

#define ITG_ADDR  (0xD0 >> 1)

#define ITG_SAMPLE_RATE_DIV_REG 0x15
#define ITG_SAMPLE_RATE_DIV_VAL B00001001

#define ITG_DLPF_REG  0X16
#define ITG_DLPF_VAL  B00011011

int cal_int;
double gyro_roll,     gyro_pitch,     gyro_yaw,
       gyro_roll_deg, gyro_pitch_deg, gyro_yaw_deg,
       gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte low, high;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  setup_gyro();
  delay(250);
  calibrate_gyro();
}

void loop() {
  delay(250);
  read_gyro();
  print_output();
}

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
  Serial.print("Calibrating...");
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    read_gyro();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;
    if(cal_int % 100 == 0) Serial.print(".");
    delay(4);
  }
  Serial.println(" done!");
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

void print_output() {
  Serial.print("Roll:");
  if (gyro_roll >= 0) Serial.print("+");
  Serial.print(gyro_roll_deg, 0);
  if (gyro_roll_deg - 2 > 0) Serial.print(" RwD");
  else if (gyro_roll_deg + 2 < 0) Serial.print(" RwU");
  else Serial.print(" ---");
  
  Serial.print(" Pitch:");
  if (gyro_pitch >= 0) Serial.print("+");
  Serial.print(gyro_pitch_deg, 0);
  if (gyro_pitch_deg - 2 > 0) Serial.print(" NoU");
  else if (gyro_pitch_deg + 2 < 0) Serial.print(" NoD");
  else Serial.print(" ---");

  Serial.print(" Yaw:");
  if (gyro_yaw >= 0) Serial.print("+");
  Serial.print(gyro_yaw_deg, 0);
  if (gyro_yaw_deg - 2 > 0) Serial.println(" NoR");
  else if (gyro_yaw_deg + 2 < 0) Serial.println(" NoL");
  else Serial.println(" ---");
}

