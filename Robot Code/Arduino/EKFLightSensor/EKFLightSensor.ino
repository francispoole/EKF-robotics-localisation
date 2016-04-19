#include <math.h>

//EKFLightSensor
//Francis Poole

//--------Initialise Variables---------------------------------------------------------------------------------
//Calibration data for linear interpolation of light readings
const float calBackData[7][2]= {{0,869}, {50,783}, {100,600}, {150,445}, {200,349}, {250,278}, {300,235}};
const float calRightData[7][2]= {{0,888}, {50,833}, {100,694}, {150,557}, {200,468}, {250,399}, {300,347}};
const float calFrontData[7][2]= {{0,876}, {50,791}, {100,604}, {150,445}, {200,338}, {250,270}, {300,236}};
const float calLeftData[7][2]= {{0,878}, {50,855}, {100,770}, {150,578}, {200,415}, {250,332}, {300,272}};

//Motion Control
float transVel;
float rotVel;

//Sensor Measurements
float range;
float bearing;

//Sensor
float back;
float right;
float front;
float left;

//Serial Input Data
char serialInput;
String data[2];
int dataNum = 0;

//Motor Speed
int motorSpeedL;
int motorSpeedR;

//Motor Input Pins
int ML1 = 11; //Left Motor input 1
int ML2 = 10; //Left Motor input 2
int MR1 = 6; //Right Motor input 1
int MR2 = 5; //Right Motor input 2

//Encoder Direction Pins
int AL = 1; //Clock Left Wheel Interupt No
int BL = 9; //Directon Left Wheel Pin
int AR = 0; //Clock Right Wheel Interupt No
int BR = 8; //Directon Right Wheel Pin

//Encoder Interup Pins
volatile int counterL = 0;
volatile int counterR = 0;

//Time
unsigned long time;
float elapsed;

//--------Setup------------------------------------------------------------------------------------------------
//What does it do?
void setup() {
  //Led Pin
  pinMode(13, OUTPUT);
  
  //Motor Input Pins Setup
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  
  //Encoder Direction Pins Setup
  pinMode(BL, INPUT);
  pinMode(BR, INPUT);
  
  //Encoder Interupt Pins Setup
  attachInterrupt(AL, countL, CHANGE);
  attachInterrupt(AR, countR, CHANGE);
  
  motor(0,0);                                    //#######MOTOR COMMAND!#########\\
  
  //Start Serial
  Serial.begin(115200);
  //Wait for connection
  while (!Serial.available()) {} 
  //Clear Serial
  while (Serial.available()) {Serial.read();} 
  Serial.println("---Connected!---");
  
  
  time = millis(); //Start encoder timer
  delay(1);
}



//--------Main Loop--------------------------------------------------------------------------------------------
//What does it do?
void loop() {
  //Wait for data request
  while (!Serial.available()) {}
  
  
  //CHANGE TO MOTION COMMAND
  //Calc Motion Control
  getControl();
  
  //Calc Sensor Measurement
  getMeasurement();
    
  //Print readings to Serial
  Serial.print(transVel,8);
  Serial.print(",");
  Serial.print(rotVel,8);
  Serial.print(";");
  Serial.print(range,8);
  Serial.print(",");
  Serial.print(bearing,8);
  Serial.print(";");
  Serial.println(elapsed,8);
  
  //delay(1);
  //Clear Serial
  while (Serial.available()) {Serial.read();}
}





//--------Get Measurement---------------------------------------------------------------------------------------
void getMeasurement() {
  //Get Readings
  back = analogRead(A0);
  right = analogRead(A1);
  front = analogRead(A2);
  left = analogRead(A3);

  //Calibrate readings
  float backRange = linInterp(calBackData, back);
  float rightRange = linInterp(calRightData, right);
  float frontRange = linInterp(calFrontData, front);
  float leftRange = linInterp(calLeftData, left);
  
  //Get minimum range and its bearing
  range = backRange;
  bearing = M_PI;
  if (rightRange < range) {
    range = rightRange;
    bearing = -M_PI/2;
  }
  if (frontRange < range) {
    range = frontRange;
    bearing = 0;
  }
  if (leftRange < range) {
    range = leftRange;
    bearing = M_PI/2;
  }
  
//  Serial.print("Range: ");
//  Serial.println(range,10);
//  Serial.print("Bearing:  ");
//  Serial.println(bearing,10);
//  Serial.println("---------");  
} 

//--------Get Control---------------------------------------------------------------------------------------
void getControl() {
  elapsed = millis() - time;
  elapsed = elapsed / 1000;
  time = millis(); //Reset encoder timer
  
  counterR = 200;
  counterL = 150;
  
  //Claculate angular velocities
  float velR = (counterR/128.0 * M_PI) / elapsed;
  float velL = (counterL/128.0 * M_PI) / elapsed;

  //Convert to translational and rotational velocities
  transVel = (velR + velL) / 2.0 * (0.069/2.0);
  rotVel = (velR - velL) / 0.14 * (0.069/2.0);

//  Serial.println(transVel,10);
//  Serial.println(rotVel,10);
//  Serial.println(elapsed,10);
  
  counterR = 0;
  counterL = 0;
}



//--------Linear Interpret--------------------------------------------------------------------------------------
float linInterp(const float calData[7][2], float y) {
  float x;
  for (int i=0; i < 6; i++) {    
    if (y <= calData[i][1] && y >= calData[i+1][1]) {
      float x0 = calData[i][0];
      float x1 = calData[i+1][0];
      float y0 = calData[i][1];
      float y1 = calData[i+1][1];
      
      x= x0 + ((y - y0) / ((y1 - y0) / (x1 - x0))); //Linearly Interpolate
      return(x);
    }
  }
  if (y < calData[6][1]) {
    return(300);
  } else {
    return(0);
  }
}

//--------Left Encoder Interupt-------------------------------------------------------------------------------
void countL(){
  if(digitalRead(BL) == 0) {
    counterL++;
  } else {
    counterL--;
  }
}

//--------Right Encoder Interupt-----------------------------------------------------------------------------
void countR(){
  if(digitalRead(BR) == 1) {
    counterR++;
  } else {
    counterR--;
  }
}

//--------Motor-------------------------------------------------------------------------------
void motor(int speedL, int speedR) {
  if (speedL >= 0) {
    analogWrite(ML2, 0);
    analogWrite(ML1, speedL);
  } else {
    analogWrite(ML1, 0);
    analogWrite(ML2, -speedL);
  }

  if (speedR >= 0) {
    analogWrite(MR1, 0);
    analogWrite(MR2, speedR);
  } else {
    analogWrite(MR2, 0);
    analogWrite(MR1, -speedR);
  }
}
