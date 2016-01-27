#include <Wire.h>

#define HMC_ADDR      0x1E
#define CTL_REG_A     0
#define CTL_REG_B     1
#define MODE_REG      2
#define HMC_DATA_REG  3

byte low, high;
double raw_x, raw_y, raw_z, angle;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  setup_magneto();
}

void loop() {
  delay(250);
  read_magneto();
  print_output();
}

void setup_magneto() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(CTL_REG_A);
  Wire.write(B00110000);
  Wire.endTransmission();

  Wire.beginTransmission(HMC_ADDR);
  Wire.write(MODE_REG);
  Wire.write(0);
  Wire.endTransmission();
}

void read_magneto() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_DATA_REG);
  Wire.endTransmission();
  Wire.beginTransmission(HMC_ADDR);
  Wire.requestFrom(HMC_ADDR, 6);
  while (Wire.available() < 6);

  high = Wire.read();
  low = Wire.read();
  raw_x = ((high<<8)|low);

  high = Wire.read();
  low = Wire.read();
  raw_z = ((high<<8)|low);

  high = Wire.read();
  low = Wire.read();
  raw_y = ((high<<8)|low);
}

void print_output() {
  Serial.print("Heading: ");
  Serial.print(heading());

  Serial.print(" You are heading ");
  if (angle < 22.5 || angle > 337.5) Serial.println("South");
  if (angle > 22.5 && angle < 67.5) Serial.println("South West");
  if (angle > 67.5 && angle < 112.5) Serial.println("West");
  if (angle > 112.5 && angle < 157.5) Serial.println("North West");
  if (angle > 157.5 && angle < 202.5) Serial.println("North");
  if (angle > 202.5 && angle < 247.5) Serial.println("North East");
  if (angle > 247.5 && angle < 292.5) Serial.println("East");
  if (angle > 292.5 && angle < 337.5) Serial.println("South East");
}

double heading() {
  angle = ((atan2(raw_y, raw_x) * 180.0) / PI) + 180;
  return angle;
}

