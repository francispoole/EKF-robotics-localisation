#include <math.h>
//Calibration data for linear interpolation of light readings
const float calBackData[7][2]= {{0,869}, {50,783}, {100,600}, {150,445}, {200,349}, {250,278}, {300,235}};
const float calRightData[7][2]= {{0,888}, {50,833}, {100,694}, {150,557}, {200,468}, {250,399}, {300,347}};
const float calFrontData[7][2]= {{0,876}, {50,791}, {100,604}, {150,445}, {200,338}, {250,270}, {300,236}};
const float calLeftData[7][2]= {{0,878}, {50,855}, {100,770}, {150,578}, {200,415}, {250,332}, {300,272}};

const int maxReadings = 5;
int reading = 0;
float back[maxReadings];
float right[maxReadings];
float front[maxReadings];
float left[maxReadings];

void setup() {
  //Take initial light readings
  for (int i = 0; i < maxReadings; i++) {
    back[i] = analogRead(A0);
    right[i] = analogRead(A1);
    front[i] = analogRead(A2);
    left[i] = analogRead(A3);
  }
  
  //Start serial
  Serial.begin(115200);
}

void loop() {
  back[reading] = analogRead(A0);
  right[reading] = analogRead(A1);
  front[reading] = analogRead(A2);
  left[reading] = analogRead(A3);
  reading = (reading + 1) % maxReadings;

  //Do calibrations for each?
  float backRange = linInterp(calBackData, readingsMean(back));
  float rightRange = linInterp(calRightData, readingsMean(right));
  float frontRange = linInterp(calFrontData, readingsMean(front));
  float leftRange = linInterp(calLeftData, readingsMean(left));

  Serial.print("Back:  ");
  Serial.println(backRange);
  Serial.print("Right: ");
  Serial.println(rightRange);
  Serial.print("Front: ");
  Serial.println(frontRange);
  Serial.print("Left:  ");
  Serial.println(leftRange);
  Serial.println("---------");
  
  //Get min range with its bearing
  float range = backRange;
  float bearing = M_PI;
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
  
  Serial.print("Range: ");
  Serial.println(range);
  Serial.print("Bearing:  ");
  Serial.println(bearing);
  Serial.println("---------");  
  
  
  delay(500);
}



float readingsMean(float vals[]) {
  //Calc total
  float total = 0;
  for (int i = 0; i < maxReadings; i++) {
    total = total + vals[i];
  }
  return(total/maxReadings);
}
  

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




////DEPRICATED - Trilaterate position
//  //Use minimum ranges
//  float r1 = frontRange;
//  float r2 = backRange;
//  float r3 = rightRange;
//  
//  Serial.print("R1: ");
//  Serial.println(r1);
//  Serial.print("R2: ");
//  Serial.println(r2);
//  Serial.print("R3:  ");
//  Serial.println(r3);
//  Serial.println("---------");
//  
//  //Trilaterate x and y of light
//  int d = 20;
//  float i = 10;
//  float j = 10;
//  float x = (pow(r1,2) - pow(r2,2) + pow(d,2)) / (d*2);
//  float y = ((pow(r1,2) - pow(r3,2) + pow(i,2) + pow(j,2)) / (j*2)) - ((i/j) *x);
//  
//  Serial.print("X: ");
//  Serial.println(x);
//  Serial.print("Y:  ");
//  Serial.println(y);
//  Serial.println("---------"); 
//  
//  //Calc range and bearing
//  float range = sqrt(pow(x-4.5,2)+pow(y,2));
//  float bearing = atan((x-4.5)/y);
//  
//  Serial.print("Range: ");
//  Serial.println(range);
//  Serial.print("Bearing:  ");
//  Serial.println(bearing);
//  Serial.println("---------");
