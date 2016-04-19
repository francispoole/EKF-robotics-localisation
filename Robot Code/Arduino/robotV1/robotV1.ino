//Robot V1

//Motors - Done
//Encoders - Done
//Light Sensors - 
//Connection to RasPi -

//Input data
char serialInput;
String data[2];
int dataNum = 0;


int motorSpeedL;
int motorSpeedR;


int ML1 = 11; //Left Motor input 1
int ML2 = 10; //Left Motor input 2
int MR1 = 6; //Right Motor input 1
int MR2 = 5; //Right Motor input 2

int AL = 1; //Clock Left Wheel Interupt No
int BL = 9; //Directon Left Wheel Pin
int AR = 0; //Clock Right Wheel Interupt No
int BR = 8; //Directon Right Wheel Pin


volatile int counterL = 0;
volatile int counterR = 0;


void setup() {  
  pinMode(13, OUTPUT); //Led pin
  Serial.begin(9600);  //Start serial for output
  
  //Motor Input Pins  
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  
  //Encoder Pins
  pinMode(BL, INPUT);
  pinMode(BR, INPUT);
  
  attachInterrupt(AL, countL, CHANGE);
  attachInterrupt(AR, countR, CHANGE);

  motor(0,0);
}

void loop() {
  while (Serial.available()) {
    serialInput = Serial.read();
    //Check end of line
    if (serialInput == '\n') {
      //Print encoders
      Serial.print(counterL);
      Serial.print(",");
      Serial.println(counterR);
      Serial.flush();
      
      //Set motors
      motorSpeedL = data[0].toInt();
      motorSpeedR = data[1].toInt();
      motor(motorSpeedL,motorSpeedR);
      
      //Reset
      data[0] = "";
      data[1] = "";
      dataNum = 0;
    } else if (serialInput == ',') {
      dataNum = 1;
    } else {
      data[dataNum] += serialInput;
    }
  }

     
}

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


void countL(){
  if(digitalRead(BL) == 0) {
    counterL++;
  } else {
    counterL--;
  }
  //Serial.println(counterL);
  
}

void countR(){
  if(digitalRead(BR) == 1) {
    counterR++;
  } else {
    counterR--;
  }
  //Serial.println(counterR);
  
}
