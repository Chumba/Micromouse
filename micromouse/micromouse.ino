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

  while(!isSolved()){
    //array of 4 cells, each cell has {x, y, gravity, visits}
    //the four cells are North, East, South, West respectively
    int cells [4][4] = {
        {posx,   posy-1, maze[posx]   [posy-1], visits[posx]  [posy-1]},
        {posx+1, posy,   maze[posx+1] [posy],   visits[posx+1][posy]},
        {posx,   posy+1, maze[posx]   [posy+1], visits[posx]  [posy+1]},
        {posx-1, posy,   maze[posx-1] [posy],   visits[posx-1][posy]}
      };
   
    boolean possibles[4] = {0,0,0,0};
    getPossibles(possibles);

    //cant go anywhere but turn around
    if(possibles[0]==0&&possibles[1]==0&&possibles[2]==0){
      turnAround();
      forward(1);
    }
    else{
      //now we need to pick the possible cell with the lowest visit value
      
      //if there is a tie we pick the one with the lowest gravity

      //if there is still a tie we favor forward, then right, then left. 
      int weights [4]={0,0,0,0}; 
     
      // weights gets populated with visit values of each direction
      // -1 means we cannot visit that location
      for(int i=0; i<4; i++){
        if (possibles[i]){
          weights[i] = cells[i][3]; 
        }
        else weights[i] = -1;
      }
      if(getWeights(weights)){
        //we figured out where we want to go
        if(weights[0]!=-1){
          goNorth();
        }
        if(weights[1]!=-1){
          goEast();
        }
        if(weights[2]!=-1){
          goSouth();
        }
        if(weights[3]!=-1){
          goWest();
        }        
      }
      else{
        // still need to rule out destinations, this time based on the gravity values
        for(int i=0; i<4; i++){
          if (possibles[i]!=-1){
            weights[i] = cells[i][4]; 
          }
          else weights[i] = -1;
        }
        
        if(getWeights(weights)){
          //the gravity check was enough to pick a best destination, we've figured out where to go
          if(weights[0]!=-1){
            goNorth();
          }
          if(weights[1]!=-1){
            goEast();
          }
          if(weights[2]!=-1){
            goSouth();
          }
          if(weights[3]!=-1){
            goWest();
          }                  
        }
        else{
          //still need to narrow it down, we favor forward, then right, then left. 
          //weights is still in NESW, we need to change it to LFR
          if(weights[direction]!=-1){ //favor forward first
            switch(direction){
              case 0:
                goNorth();
                break;
              case 1:
                goEast();
                break;
              case 2:
                goSouth();
                break;
              case 3:
                goWest();
                break;
            }
          //TODO pick another direction, right or left, i dont think this could ever happen tbh
          }
        }
      }    
    }
    //we're at our destination, update visits
    visits[posx][posy]++;
  }

  /*testing Control Systems, temp code
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
  
  */

}

void goNorth(){
  switch(direction){
    case 0:
      forward(1);
      break;
    case 1:
      turnLeft();
      forward(1);
      break;
    case 2:
      turnAround();
      forward(1);
      break;
    case 3:
      turnRight();
      forward(1);
      break;
  }
}

void goEast(){
  switch(direction){
    case 0:
      turnRight();
      forward(1);
      break;
    case 1:
      forward(1);
      break;
    case 2:
      turnLeft();
      forward(1);
      break;
    case 3:
      turnAround();
      forward(1);
      break;
  }
}

void goSouth(){
  switch(direction){
    case 0:
      turnAround();
      forward(1);
      break;
    case 1:
      turnRight();
      forward(1);
      break;
    case 2:
      forward(1);
      break;
    case 3:
      turnLeft();
      forward(1);
      break;
  }
}

void goWest(){
  switch(direction){
    case 0:
      turnLeft();
      forward(1);
      break;
    case 1:
      turnAround();
      forward(1);
      break;
    case 2:
      turnRight();
      forward(1);
      break;
    case 3:
      forward(1);
      break;
  }
}

//modifies weights to determine which has lowest values
//examples (0,0,-1,3) would create (0,0,-1,-1)
// 4,4,1,-1 would create -1,-1,1,-1
// we also return true if there is only one value left that is not -1
boolean getWeights(int weights []){
  int min;
  for(int i=0;i<4;i++){
    if (weights[i]==-1){
      //do nothing we already ruled this out
    }
    if (weights[i]<min) min = weights[i];
  }
  int count=0;
  for(int i=0; i<4; i++){
    if(weights[i]>min){
      count++;
      weights[i]=-1;
    }
  if (count=3) return true;
  else return false;
}
}

//determines if we are at a solved position (in the center of the maze
boolean isSolved(){
  if((posx == 7 || posx == 8)&&(posy == 7 || posy == 8)){
    return true;
  }
  return false;
}

//modifies the array possibles to determine which cell i can move to next
// north, east, south, west
void getPossibles(boolean possibles[]){
  // check the walls to determine possible moves
  int vR = analogRead(IRpinR);
  int vF = analogRead(IRpinF);
  int vL = analogRead(IRpinL);

  boolean poss[] = {0,0,0,0};
  
  bool left = vL > 200;
  bool front = vF > 200;
  bool right = vR > 200;

  if(!left){
    // no left wall
    poss[0] = 1;
  }
  if(!front){
    // no front wall
    poss[1] = 1;
  }
  if(!right){
    // no right wall
    poss[2] = 1;
  }

  switch(direction){
    // facing north  
    case 0:
      possibles[0] = poss[1];
      possibles[1] = poss[2];
      possibles[2] = 1;
      possibles[3] = poss[0];
      break;
    // facing east
    case 1:
      possibles[0] = poss[0];
      possibles[1] = poss[1];
      possibles[2] = poss[2];
      possibles[3] = 1;
      break;
    // facing south
    case 2:
      possibles[0] = 1;
      possibles[1] = poss[0];
      possibles[2] = poss[1];
      possibles[3] = poss[2];
      break;
    //facing west
    case 3:
      possibles[0] = poss[2];
      possibles[1] = 1;
      possibles[2] = poss[0];
      possibles[3] = poss[1];
      break;
    
  }
 
}

// This is the speed run code, it knows a path and attempts to run the path as
// fast as possible. This should be done as many times as time allows.
void speedRun(){
  
}

//move the motors using encoders for distance measurement
// distance right motor, distance left, speed right, speed left
void move(int dR, int dL, int sR, int sL){
  while((abs(enc.getCountsM1()) < dR) or (abs(enc.getCountsM2()) < dL)){
    setMotors(sR,sL);
  }
  offReset();
}

// turn the mouse around
// also sets direction
void turnAround(){
  move(915, -915, -150, 100);
  if(direction==1){
    direction=3;
  }
  else if(direction==2){
    direction=4;
  }
  else if(direction==3){
    direction=1;
  }
  else{
    direction = 2;
  }
}

// turn the mouse right
// also sets direction
void turnRight(){

 move(400, -400, -150, 100); 
 if(direction==4){
  direction=1;
 }
 else{
  direction++;
 }
}

// turn the mouse left
// also sets direction
void turnLeft(){
  
  move(-365, 365, 100, -115);
  if(direction==1){
  direction = 4;
 }
 else{
  direction--;
 }
}

//moves the mouse forward x cells
//also sets the new position
//TODO set visits for movements longer than 1 cell
void forward(int cells){
  //update positions
   switch(direction){
    case 1:
      posy=posy-cells;
      break;
    case 2:
      posx=posx+cells;
      break;
    case 3:
      posy=posy+cells;
      break;
    case 4:
      posx=posx-cells;
      break;
  }
  
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

//sets motors to speed right, speed left
void setMotors(int speedR, int speedL){
  bool rRev = speedR>0;
  bool lRev = speedL>0;
  
  digitalWrite(directionPinR, rRev);
  digitalWrite(directionPinL, lRev);
  delay(5);
  analogWrite(speedPinR, abs(speedR));
  analogWrite(speedPinL, abs(speedL-15)); //-20 here
}

//shuts off motors, resets encoder counts and waits for stop
void offReset(){
  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);
  delay(200);
  enc.getCountsAndResetM1();
  enc.getCountsAndResetM2();

}

//waits for a button press
//also accounts for debouncing
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

//generate maze data needed for maze algorithms
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

//generates gravity maze values
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
