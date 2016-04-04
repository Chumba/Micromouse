// Stevens IEEE Micromouse 2016
// IEEE@stevens.edu
// Bryan Charalambous

//include encoder library
#include "PololuWheelEncoders.h"

//Sensor pins
#define IRpinR 3
#define IRpinF 2
#define IRpinL 1

//Motor Pins
#define speedPinR 9
#define directionPinR 8
#define speedPinL 10
#define directionPinL 11

// Encoder Pins
#define encRA 2
#define encRB 3
#define encLA 5
#define encLB 4

//Button pin
#define buttonPin 12

// Define constants
#define size 16
#define DEBUG 1

//Button variables
bool buttonState; 
bool lastButtonState = HIGH;
long lastDebounceTime = 0;
long debounceDelay = 50;

//Maze variables
unsigned short maze [size] [size];
unsigned short visits [size] [size];

// location variables
int posx = 0;
int posy = 15;

//direction: 1: North 2: East 3: South 4: West
char direction = 1;

// Motor Speed variable
int speed = 127;

PololuWheelEncoders enc;

void setup() {
  if (DEBUG) Serial.begin(9600);  
  enc.init(encRA, encRB, encLA, encLB);
  pinMode(buttonPin, INPUT_PULLUP);

  // generate the maze values
  generateMaze();
  
  // wait for start signal
  waitBtn();

  //solve the maze and return home
  solve();
}

void loop() {
 
  // wait for start signal
  waitBtn();
  
  //increase speed
  speed = speed*.10;
  
  //Speed run
  speedRun();
}

// This is the solving part of the maze, it discovers a path and returns home
void solve(){
  //array of 4 cells, each cell has {x, y, gravity, visits}
  //the four cells are North, East, South, West respectively
  int cells [4][4] = {
      {posx,   posy-1, maze[posx]   [posy-1], visits[posx]  [posy-1]},
      {posx+1, posy,   maze[posx+1] [posy],   visits[posx+1][posy]},
      {posx,   posy+1, maze[posx]   [posy+1], visits[posx]  [posy+1]},
      {posx-1, posy,   maze[posx-1] [posy],   visits[posx-1][posy]}
    };

  
  char possible = possibles();

  //testing Control Systems, temp code
while(1){
  forward(1);
  turnRight();
  forward(1);
  turnRight();
  forward(1);
  turnLeft();
  forward(1);
  turnLeft();
  forward(2);
  turnLeft();
  forward(1);
  turnRight();
  forward(1);
  turnRight();
  forward(1);
  turnLeft();
  forward(1);
  turnLeft();
  forward(1);
  turnAround();
  forward(1);
  turnLeft();
  forward(1);
  turnLeft();
  forward(2);
  turnLeft();
  forward(3);
  turnAround();
  waitBtn();
}

  // //end of test code
//while(1){
//  turnLeft();
//  delay(200);
//  turnRight();
//  waitBtn();
//}
  
  
  
}

char possibles(){
  // check the walls to determine possible moves
  int vR = analogRead(IRpinR);
  int vF = analogRead(IRpinF);
  int vL = analogRead(IRpinL);
  
  bool left = vL > 200;
  bool front = vF > 200;
  bool right = vL > 200;

  char results = 0100;

  if(!left){
    // no left wall
    results = results + B1000;
  }
  if(!front){
    // no front wall
    results = results + B0001;
  }
  if(!right){
    // no right wall
    results = results + B0010;
  }
  return results;
}

// This is the speed run code, it knows a path and attempts to run the path as
// fast as possible. This should be done as many times as time allows.
void speedRun(){
  
}

void move(int dR, int dL, int sR, int sL){
  while((abs(enc.getCountsM1()) < dR) or (abs(enc.getCountsM2()) < dL)){
    setMotors(sR,sL);
  }
  offReset();
}

void turnAround(){
  move(915, -915, -150, 100);
}

void turnRight(){

 move(400, -400, -150, 100); 
}

void turnLeft(){
  
  move(-365, 365, 100, -115);
}
void forward(int cells){
  
  int dist = 0;
  
  while(dist<120*cells){
    
    dist = ((enc.getCountsM1() / 12) + (enc.getCountsM2() / 12))/2;
    int vR = analogRead(IRpinR);
    int vF = analogRead(IRpinF);
    int vL = analogRead(IRpinL);
  
    int err;
    int errR;
    int errL;

    if(vF > 400){
      offReset();
      break;
    }
    
    if((vL < 220) && (vR < 220)){
      if (DEBUG) Serial.println("No walls");
      errL = enc.getCountsM2() - enc.getCountsM1();
      errR = errL;
    }  
    else if (vR < 200){
      if (DEBUG) Serial.println("No right wall");
      errR = -(430 - vL);
      errL = 0;
    }
    else if(vL < 200){
      if (DEBUG) Serial.println("No left wall");
      errL = -(430 - vR);
      errR = 0;
    }
    else{
      if (DEBUG) Serial.println("2 walls");

      errR = -(430 - vL);
      errL = -(430 - vR);
    }

       
    int fixedR = map(errR, -128, 128, -30, 30);
    int fixedL = map(errL, -128, 128, -30, 30);
   
    setMotors(speed-fixedR, speed-fixedL);
    
  }

  offReset();
}

void setMotors(int speedR, int speedL){
  bool rRev = speedR>0;
  bool lRev = speedL>0;
  
  digitalWrite(directionPinR, rRev);
  digitalWrite(directionPinL, lRev);
  delay(5);
  analogWrite(speedPinR, abs(speedR));
  analogWrite(speedPinL, abs(speedL-15)); //-20 here
}

void offReset(){
  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);
  delay(200);
  enc.getCountsAndResetM1();
  enc.getCountsAndResetM2();

}

void waitBtn(){
  while(1){
    bool reading = digitalRead(buttonPin);
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading;
        if (buttonState == LOW) {
          digitalWrite(13, HIGH);
          delay(150);
          digitalWrite(13,LOW);
          delay(500);
          return;
        }
      }
    }
    lastButtonState = reading;
  }
}

void generateMaze(){
  for(int i=0; i<size ; i++){
    for(int j=0; j<size ; j++){
      maze[i][j] = gen(i,j);
      if (DEBUG)  Serial.print(maze[i][j]);
      if (DEBUG)  Serial.print(", ");
    }
    if (DEBUG) Serial.println();
    
  }
  if (DEBUG) Serial.println("-------------------");
  for(int i=0; i<size ; i++){
    for(int j=0; j<size ; j++){
      visits[i][j] = 0;
      if (DEBUG) Serial.print(visits[i][j]);
      if (DEBUG) Serial.print(", ");
    }
    if (DEBUG) Serial.println();
  }
}

int gen(unsigned short row1, unsigned short col1){
  unsigned short nr;
  unsigned short nc;
  
  short half = size/2;
  short halfless1 = (size/2)-1;
  if(row1>halfless1){
    nr = row1-half;
    row1 = halfless1-nr;
  }
  if(col1>halfless1){
    nc=col1-half;
    col1=halfless1-nc;
  }
  return(size-(col1)-(row1));
}
