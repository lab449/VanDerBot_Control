#define initAng1 229.36
#define initAng2 30.86
#define mPrecision 0.2
#define mSpeed 6000

#include "Wire.h"
#include "SoftwareWire.h"
SoftwareWire sWire(14, 17);
static const uint8_t _addr_raw_angle = 0x0c;
static const uint8_t _ams5600_Address = 0x36;
float ang1, ang2;
int i;

union Received {
  uint8_t in[4];
  double fl;
} rec1, rec2, rec3;

float readAngle1() {
  sWire.beginTransmission(_ams5600_Address);
  sWire.write(_addr_raw_angle);
  sWire.endTransmission();
  sWire.requestFrom(_ams5600_Address, (uint8_t) 2);
  while (sWire.available() < 2);
  int highByte = sWire.read();
  int lowByte  = sWire.read();
  return (( highByte << 8 ) | lowByte) * 0.0879120879120879121;
}

float readAngle2() {
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(_addr_raw_angle);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, (uint8_t) 2);
  while (Wire.available() < 2);
  int highByte = Wire.read();
  int lowByte  = Wire.read();
  return (( highByte << 8 ) | lowByte) * 0.0879120879120879121;
}

void motorStep(int mDir, int mNum, int mSpd) {
  if (mDir == 1) digitalWrite(mNum + 4, HIGH);
  else digitalWrite(mNum + 4, LOW);
  digitalWrite(mNum + 1, HIGH);
  delayMicroseconds(mSpd);
  digitalWrite(mNum + 1, LOW);
  delayMicroseconds(mSpd);
}

bool ctrl(int motor, float ang, float angDes) {
  if (abs(ang - angDes) > 0.5)
    motorStep(ang > angDes, motor, 2500);
  return abs(ang - angDes) > 0.5;
}

void track(float angD1, float angD2, float angD3) {
  if (angD3 > 1.5) {
    for (int i = 0; i < 5000; i++) motorStep(0, 3, 50);
  } else if (angD3 > 0.5) {
    for (int i = 0; i < 5000; i++) motorStep(1, 3, 50);
  } else {
    ang1 = initAng1 - readAngle1();
    ang2 = readAngle2() - initAng2;
    while (abs(ang1 - angD1) > mPrecision or
           abs(ang2 - angD2) > mPrecision) {
      if (ang1 > angD1) digitalWrite(5, HIGH);
      else digitalWrite(5, LOW);
      if (ang2 > angD2) digitalWrite(6, HIGH);
      else digitalWrite(6, LOW);
      if (abs(ang1 - angD1) > mPrecision) digitalWrite(2, HIGH);
      if (abs(ang2 - angD2) > mPrecision) digitalWrite(3, HIGH);
      delayMicroseconds(mSpeed);
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      delayMicroseconds(mSpeed);
      ang1 = initAng1 - readAngle1();
      ang2 = readAngle2() - initAng2;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sWire.begin();
  for (int i = 2; i < 9; i++) pinMode(i, OUTPUT);
  digitalWrite(8, LOW);
  Serial.print("START");
//  for (int i=0; i<5000; i++) motorStep(0, 3, 50);
}

void loop() {
  if (Serial.available() >= 3 * sizeof(float)) {
    for (i = 0; i < 4; i++) rec1.in[i] = Serial.read();
    for (i = 0; i < 4; i++) rec2.in[i] = Serial.read();
    for (i = 0; i < 4; i++) rec3.in[i] = Serial.read();
    track(rec1.fl, rec2.fl, rec3.fl);
    Serial.print("OK");
  }
}
