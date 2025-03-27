#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"
#define TCAADDR 0x70

MS5837 depthSensor;
const int MPU_addr1 = 0x68;
float xAccel, yAccel, zAccel, roll, pitch;
double proportionalGain, integralGain, derivativeGain;
double error, derivativeError, previousError, integralSum, pidOutput;
double proportion_value, integral_value, derivative_value;
double depth, goal, depth_diff; // in meters
double bot_width = 17; //in inches
double baselineDepth, baselineRoll, baselinePitch;

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

//Pid calculations :)
double PID(double depth, double goal) {
  int pidOutput;
  error = goal - depth; //depth from sensor
  integralSum += error;
  //derivativeError = error - previousError;

  proportion_value = proportionalGain * error;
  //integral_value = integralGain * integralSum;
  //derivative_value = derivativeGain * derivativeError;

  pidOutput = proportion_value + integral_value + derivative_value;
  pidOutput = pidOutput;

  if (pidOutput > 0) {
    pidOutput += 36;
  }
  else {
    pidOutput -= 36;
  }

  pidOutput = max(-400, pidOutput);
  pidOutput = min(400, pidOutput);

  //previousError = error; //for derivative

  return pidOutput;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  FL_T.attach(4); //
  BR_T.attach(8); //
  BL_T.attach(10); //
  FR_T.attach(6); //

  VTR_T.attach(3);
  VTL_T.attach(7);
  VBR_T.attach(9); //
  VBL_T.attach(5); // 3.1 back, 2.8 forward

  initDepthSensor(7);
  calibrateDepth();
  
  initMPU(1);
  calibrateMPU();

  tunePID(500, 10, 0);

  delay(1000);
}

int tick = 0;
int line = 0;

void loop() {
  if (Serial.available()) {
    line++;
    Serial.print("line ");
    Serial.print(line);
    FR_PWM = Serial.readStringUntil('-').toInt();
    FL_PWM = Serial.readStringUntil('=').toInt();
    BR_PWM = ((Serial.readStringUntil('+').toInt() - 1500) * (-1)) + 1500;
    BL_PWM = Serial.readStringUntil('*').toInt();
    VTR_PWM = Serial.readStringUntil(',').toInt();
    VTL_PWM = Serial.readStringUntil(']').toInt();
    VBR_PWM = Serial.readStringUntil('/').toInt();
    VBL_PWM = Serial.readStringUntil('.').toInt();
    
    

    if ((VTR_PWM > 1464) && (VTR_PWM < 1536)) {
      getDepth(7);
      // getAngle(1);

      Serial.print(" PID ON ");
      Serial.print("DEPTH: ");
      Serial.print(depth);
      Serial.print(" ");
      
//      Serial.print("Depth: ");
//      Serial.print(depth);
//      
//      Serial.print(" | Roll: ");
//      Serial.print(roll);
//      
//      Serial.print(" | Pitch: ");
//      Serial.println(pitch);
      
      // depth_diff = bot_width / (tan(roll - PI / 2)); // in inches
      // depth_diff /= 39.37; // conversion to meters
      
      // gets depth every 50 readings to ensure it stays in place
      if (tick == 0) {
        goal = depth;
        tick++;
      }
      else if (tick == 100) {
        tick = 0;
      }
      
      

      int pidPWM = PID(depth, goal);
      Serial.print("PiDPWM: ");
      Serial.print(pidPWM);
    
      VTR_PWM += PID(depth, goal);
      VTL_PWM += PID(depth, goal);
      VBR_PWM += PID(depth, goal);
      VBL_PWM += PID(depth, goal);
      Serial.print("tick: ");
      Serial.print(tick);
    }
    else {
      tick = 0;
    }    

    Serial.println(
               "FR_PWM: " + String(FR_PWM) + ", " + 
               "FL_PWM: " + String(FL_PWM) + ", " + 
               "BR_PWM: " + String((BR_PWM - 1500) * (-1) + 1500) +
               "BL_PWM: " + String(BL_PWM) + ", " + 
               "VTR_VERT: " + String(VTR_PWM) + ", " + 
               "VTL_VERT: " + String(VTL_PWM) + ", " + 
               "VBR_VERT: " + String(VBR_PWM) + ", " +
               "VBL_VERT: " + String(VBL_PWM) + ", ");

    FL_T.writeMicroseconds(FL_PWM);
    BL_T.writeMicroseconds(BL_PWM);
    FR_T.writeMicroseconds(FR_PWM);
    BR_T.writeMicroseconds(BR_PWM);    
    VTR_T.writeMicroseconds(VTR_PWM);
    VTL_T.writeMicroseconds(VTL_PWM);
    VBR_T.writeMicroseconds(VBR_PWM);
    VBL_T.writeMicroseconds(VBL_PWM);

//    Serial.println("RB_PWM: " + String((RB_PWM - 1500) * (-1) + 1500) + ", " +
//                   "LF_PWM: " + String(LF_PWM) + ", " + 
//                   "LB_PWM: " + String(LB_PWM) + ", " + 
//                   "RF_PWM: " + String(RF_PWM) + ", " + 
//                   "FRONT_VERT: " + String(FRONT_PWM) + ", " + 
//                   "BACK_VERT: " + String(BACK_PWM));
    
    // delay(10);
  }
}





// change SDA/SCL on mux
void selectChannel(int channel) {
  if (channel > 7) return;

  Wire.beginTransmission(0x70); // TCA9548A address
  Wire.write(1 << channel);     // send byte to select bus
  Wire.endTransmission();
}


// intialize pressure sensor with necessary delays
void initDepthSensor(int channel) {
  delay(500);

  Serial.println("Intializing Depth Sensor...");
  selectChannel(channel);

  while (!depthSensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  depthSensor.setModel(MS5837::MS5837_02BA);
  depthSensor.setFluidDensity(997);
  depthSensor.init();

  Serial.println("Success!\n");

  delay(500);
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


// reads depth from pressure sensor
void getDepth(int channel) {
  selectChannel(channel);
  depthSensor.read();
  depth = (double) depthSensor.depth();  // float -> double
  depth -= baselineDepth;
}


// MPU6050 calculations to obtain roll and pitch
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

  // IN RADIANS
  roll = atan2(yAccel , zAccel);
  pitch = atan2(-xAccel , sqrt(yAccel * yAccel + zAccel * zAccel)); //account for roll already applied
  
  
  // convert to degrees
  // roll *= 180.0 / PI;
  // pitch *= 180.0 / PI;

  roll -= baselineRoll;
  pitch -= baselinePitch;

}





//change gain values
void tunePID(double proportional, double integral, double derivative) {
  proportionalGain = proportional;
  integralGain = integral;
  derivativeGain = derivative;
}


void calibrateDepth() {
  Serial.println("Calibrating Depth Sensor");
  
  int tick1 = 0;
  double sum = 0.0;
  while (tick1 < 100) {
    getDepth(7);
    sum += depth;
    
    tick1++;
  }

  baselineDepth = sum/tick1;

  Serial.print("Depth Deviation: ");
  Serial.println(baselineDepth);
  Serial.println();

  delay(300);
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