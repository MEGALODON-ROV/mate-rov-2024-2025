#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"
#define TCAADDR 0x70

int BR_PWM;
int FL_PWM;
int BL_PWM;
int FR_PWM;
int VTR_PWM;
int VTL_PWM;
int VBR_PWM;
int VBL_PWM;

Servo FL_T; //left front
Servo BL_T; //left back
Servo FR_T; //right front
Servo BR_T; //right back
Servo VTR_T;
Servo VTL_T;
Servo VBR_T;
Servo VBL_T;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //add corresponding pin numbers like this: FL_T.attach(1)
  FL_T.attach(4); //
  BR_T.attach(8); //
  BL_T.attach(10); //
  FR_T.attach(6); //

  VTR_T.attach(3);
  VTL_T.attach(7);
  VBR_T.attach(9); //
  VBL_T.attach(5); //

  initMPU(1);
  calibrateMPU();

  // collecting top and bottom pic prob won't use
  /*
  VTR_T.writeMicroseconds(1900);
  VTL_T.writeMicroseconds(1900);
  VBR_T.writeMicroseconds(1100);
  VBL_T.writeMicroseconds(1100);
  delay(2000);
  VTR_T.writeMicroseconds(1500);
  VTL_T.writeMicroseconds(1500);
  VBR_T.writeMicroseconds(1500);
  VBL_T.writeMicroseconds(1500);
  delay(1000);
  VTR_T.writeMicroseconds(1100);
  VTL_T.writeMicroseconds(1100);
  VBR_T.writeMicroseconds(1900);
  VBL_T.writeMicroseconds(1900);
  delay(4000);
  */

  // having the ROV move around to take pics

  getAngle(1);
  int stopYaw = yaw
}

void loop() {
  // put your main code here, to run repeatedly:
    getAngle(1);
  
  while (yaw != stopYaw) {
    FR_T.writeMicroseconds(1900);
    BR_T.writeMicroseconds(1900);
    FL_T.writeMicroseconds(1100);
    BL_T.writeMicroseconds(1100);
  }

  // stoping thruster movmeents
  FR_T.writeMicroseconds(1500);
  BR_T.writeMicroseconds(1500);
  FL_T.writeMicroseconds(1500);
  BL_T.writeMicroseconds(1500);

  delay(50)
}

// change SDA/SCL on mux
void selectChannel(int channel) {
  if (channel > 7) return;

  Wire.beginTransmission(0x70); // TCA9548A address
  Wire.write(1 << channel);     // send byte to select bus
  Wire.endTransmission();
}

// intialize MPU6050 with necessary delays
void initMPU(int channel) {
  delay(500);
  Serial.println("Intializing MPU6050...");

  selectChannel(channel);
  Wire.beginTransmission(MPU_addr1);                 //begin, send the slave adress (in this case 68)
  Wire.write(0x6B);                                  //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);                        //end the transmission

  Serial.println("Success!\n");
  delay(500);
}

void getAngle(int channel) {
  selectChannel(channel);

  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);  //send starting register address, accelerometer high byte
  Wire.endTransmission(false); //restart for read

  Wire.requestFrom(MPU_addr1, 6, true); //get six bytes accelerometer data
  int t = Wire.read();
  xAccel = (t << 8) | Wire.read();
  t = Wire.read();
  yAccel = (t << 8) | Wire.read();
  t = Wire.read();
  zAccel = (t << 8) | Wire.read();

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  // IN RADIANS
  roll = atan2(yAccel , zAccel);
  pitch = atan2(-xAccel , sqrt(yAccel * yAccel + zAccel * zAccel)); //account for roll already applied
  yaw = yaw + zAccel*elapsedTime
  
  
  // convert to degrees
  // roll *= 180.0 / PI;
  // pitch *= 180.0 / PI;

  roll -= baselineRoll;
  pitch -= baselinePitch;

}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050");
  
  int tick2 = 1;
  double rollSum = 0.0;
  double pitchSum = 0.0;
  
  while (tick2 <= 100) {
    getAngle(1);
    rollSum += roll;
    pitchSum += pitch;
    tick2++;
  }

  baselineRoll = rollSum/tick2;
  baselinePitch = pitchSum/tick2;

  Serial.print("Roll Deviation: ");
  Serial.println(baselineRoll);
  Serial.print("Pitch Deviation: ");
  Serial.println(baselinePitch);
  Serial.println();

  delay(300);
}
