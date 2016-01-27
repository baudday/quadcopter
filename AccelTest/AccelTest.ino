#include <Wire.h>

#define ADXL_ADDR     0x53
#define POWER_CTL_REG 0x2D
#define ADXL_DATA_REG 0x32

bool calibrated = false;
double raw_x, raw_y, raw_z, roll_deg, pitch_deg, yaw_deg, zg, x_cal, y_cal, z_cal;
byte low, high;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  setup_accel();
  delay(250);
  calibrate_accel();
}

void loop() {
  delay(250);
  read_accel();
  print_output();
}

void setup_accel() {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(POWER_CTL_REG);
  Wire.write(B00001000);
  Wire.endTransmission();

  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(ADXL_DATA_REG);
  Wire.write(B00001011);
  Wire.endTransmission();
}

void calibrate_accel() {
  Serial.print("Calibrating...");
  for (int cal_int = 0; cal_int < 2000; cal_int++) {
    read_accel();
    x_cal += raw_x;
    y_cal += raw_y;
    z_cal += raw_z;
    if(cal_int % 100 == 0) Serial.print(".");
    delay(4);
  }
  Serial.println(" done!");
  x_cal /= 2000;
  y_cal /= 2000;
  z_cal /= 2000;
  calibrated = true;
}

void read_accel() {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(0x32);
  Wire.endTransmission();
  Wire.beginTransmission(ADXL_ADDR);
  Wire.requestFrom(ADXL_ADDR, 6);
  while(Wire.available() < 6);

  low = Wire.read();
  high = Wire.read();
  raw_x = ((high<<8)|low);
  if (calibrated) raw_x -= x_cal;

  low = Wire.read();
  high = Wire.read();
  raw_y = ((high<<8)|low);
  raw_y *= -1;
  if (calibrated) raw_y -= y_cal;

  low = Wire.read();
  high = Wire.read();
  raw_z = ((high<<8)|low);

  Wire.endTransmission();

  pitch_deg = (atan2(raw_y,(sqrt(raw_x*raw_x+raw_z*raw_z))) * 180.0) / PI;
  roll_deg = (atan2(raw_x,sqrt(raw_y*raw_y+raw_z*raw_z)) * 180.0) / PI;
  zg = raw_z / z_cal;
}

void print_output() {
  Serial.print("Roll: ");
  Serial.print(roll_deg);
  Serial.print(" Pitch: ");
  Serial.print(pitch_deg);
  Serial.print(" Z: ");
  Serial.println(zg);
}

