#include <Servo.h>

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
  Serial.begin(9600);

  //add corresponding pin numbers like this: FL_T.attach(1)
  FL_T.attach(4); //
  BR_T.attach(11); //
  BL_T.attach(8); //
  FR_T.attach(6); //

  VTR_T.attach(7);
  VTL_T.attach(9);
  VBR_T.attach(5); //
  VBL_T.attach(3); // 3.1 back, 2.8 forward

  delay(2000);
}



void loop() {
  if (Serial.available()) {
    
    //ONE OF THE THRUSTERS IS REVERESED. 
    //CHECK ALL EACH AND EVERY THRUSTER'S DIRECTION.
    //CUT AND PASTE EQ IN BR_PWM
    FR_PWM = Serial.readStringUntil('-').toInt();
    FL_PWM = Serial.readStringUntil('=').toInt();
    BR_PWM = Serial.readStringUntil('+').toInt();
    BL_PWM = ((Serial.readStringUntil('*').toInt() - 1500) * (-1)) + 1500;
    VTR_PWM = Serial.readStringUntil(',').toInt();
    VTL_PWM = Serial.readStringUntil(']').toInt();
    VBR_PWM = Serial.readStringUntil('/').toInt();
    VBL_PWM = Serial.readStringUntil('.').toInt();

    Serial.println(
               "FR_PWM: " + String(FR_PWM) + ", " + 
               "FL_PWM: " + String(FL_PWM) + ", " + 
               "BR_PWM: " + String(BR_PWM) + "," + 
               "BL_PWM: " + String((BL_PWM - 1500) * (-1) + 1500) + ", " + 
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
    
    delay(10);
  }
}