// Stevens IEEE Micromouse 2016
// IEEE@stevens.edu
// Bryan Charalambous

//include encoder library
#include "PololuWheelEncoders.h"
#include <LinkedList.h>

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
#define mazeSize 16
#define DEBUG   0  //debugging for control systems
#define DEBUG1  0  //debugging for algorithms

//Button variables
bool buttonState;
bool lastButtonState = HIGH;
long lastDebounceTime = 0;
long debounceDelay = 50;

//Maze variables
int maze [mazeSize] [mazeSize];
int visits [mazeSize] [mazeSize];

// location variables
int posx = 0;
int posy = 15;

//direction: 0: North 1: East 2: South 3: West
char direction = 0;

// Motor Speed variable
int speed = 127;

// variables for cellSolutions
struct cell {
  int x;
  int y;
};
struct action {
  int act; //0 is forward, 1 is turnLeft, 2 is turnRight, 3 is turnAround
  int cells; //only matters for forward, how many cells should I move
};
LinkedList<cell> cellSolution;
LinkedList<action> actionSolutionToSolve;
LinkedList<action> actionSolutionToHome;

PololuWheelEncoders enc;

void setup() {
  if (DEBUG || DEBUG1) Serial.begin(9600);
  enc.init(encRA, encRB, encLA, encLB);
  pinMode(buttonPin, INPUT_PULLUP);
  // generate the maze values
  generateMaze();

  //add home to cellSolution
  cellSolution.add({posx, posy});

  // wait for start signal
  waitBtn();

  //solve the maze
  solve();

  // create the action solution for solving the maze
  translateSolve();

  //create the action solution for returning to start
  translateHome();
}

void loop() {

  //start by going home
  act(actionSolutionToHome);

  // wait for start signal
  waitBtn();

  //solve the maze with the optimized solution
  act(actionSolutionToSolve);
  delay(1000);
}

// This is the solving part of the maze, it discovers a path to the center of the maze
void solve() {
  cell next = {0, 0};
  while (!isSolved()) {
    //array of 4 cells, each cell has {x, y, gravity, visits}
    //the four cells are North, East, South, West respectively
    int cells [4][4] = {
      {posx,   posy - 1, maze[posx]   [posy - 1], visits[posx]  [posy - 1]},
      {posx + 1, posy,   maze[posx + 1] [posy],   visits[posx + 1][posy]},
      {posx,   posy + 1, maze[posx]   [posy + 1], visits[posx]  [posy + 1]},
      {posx - 1, posy,   maze[posx - 1] [posy],   visits[posx - 1][posy]}
    };

    if (posx == 0) {
      cells[3][0] = -1;
      cells[3][2] = 17;
      cells[3][3] = 999;
    }
    if (posx == 15) {
      cells[1][0] = 16;
      cells[1][2] = 17;
      cells[1][3] = 999;
    }
    if (posy == 0) {
      cells[0][1] = -1;
      cells[0][2] = 17;
      cells[0][3] = 999;
    }
    if (posy == 15) {
      cells[2][1] = 16;
      cells[2][2] = 17;
      cells[2][3] = 999;
    }

    boolean possibles[4] = {0, 0, 0, 0};
    getPossibles(possibles);

    if (DEBUG1) {
      Serial.print("possibles: ");
      Serial.print(possibles[0]);
      Serial.print(possibles[1]);
      Serial.print(possibles[2]);
      Serial.println(possibles[3]);
    }




    //now we need to pick the possible cell with the lowest visit value

    //if there is a tie we pick the one with the lowest gravity

    //if there is still a tie we favor forward, then right, then left.
    int weights [4] = {0, 0, 0, 0};

    // weights gets populated with visit values of each direction
    // -1 means we cannot visit that location
    for (int i = 0; i < 4; i++) {
      if (possibles[i]) {
        weights[i] = cells[i][3];
      }
      else weights[i] = -1;
    }


    bool temp = getWeights(weights);
    //printMazes();
    //waitBtn();
    if (temp) {
      //we figured out where we want to go
      if (weights[0] != -1) {
        goNorth();
      }
      if (weights[1] != -1) {
        goEast();
      }
      if (weights[2] != -1) {
        goSouth();
      }
      if (weights[3] != -1) {
        goWest();
      }
    }
    else {
      delay(5);
      // still need to rule out destinations, this time based on the gravity values
      for (int i = 0; i < 4; i++) {
        if (weights[i] != -1) {
          weights[i] = cells[i][2];
        }
        else weights[i] = -1;
      }

      if (DEBUG1) {
        Serial.print("weights before getweight gravity: ");
        Serial.print(weights[0]);
        Serial.print(weights[1]);
        Serial.print(weights[2]);
        Serial.println(weights[3]);
      }
      if (getWeights(weights)) {
        //the gravity check was enough to pick a best destination, we've figured out where to go
        if (weights[0] != -1) {
          goNorth();
        }
        if (weights[1] != -1) {
          goEast();
        }
        if (weights[2] != -1) {
          goSouth();
        }
        if (weights[3] != -1) {
          goWest();
        }
      }
      else {
        //still need to narrow it down, we favor forward, then right, then left.
        //weights is still in NESW, we need to change it to LFR
        if (weights[direction] != -1) { //favor forward first
          switch (direction) {
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
      next.x = posx;
      next.y = posy;
      push(next);
    }

    //we're at our destination, update visits
    visits[posx][posy]++;
  }
}

// acts based on the solution linked list, wether the homebound of solvedbound solutions
void act(LinkedList<action> solution) {
  //for each action in the solution, act based on the solution step
  for (int i = 0; i < solution.size(); i++) {
    //solution is a linked list of actions
    //an action has act, and cells.
    
    //if act is 0, we need a cells value (number of cells to move forward)
    //act values: 0=forward, 1 is turnleft, 2 is turnright, 3 is turnaround

    switch (solution.get(i).act) {
      case 0:
        forward(solution.get(i).cells);
        break;
      case 1:
        turnLeft();
        break;
      case 2:
        turnRight();
        break;
      case 3:
        turnAround();
        break;
    }
  }
}

// adds the next element of the cellSolution to the linked list
// if that cell is already in the cellSolution, we can remove every
// cell which follows the first instance, since we did a loop
void push(cell next) {
  bool found = false;
  cell current;
  for (int i = 0; i < cellSolution.size(); i++) {
    current = cellSolution.get(i);
    if (current.x == next.x && current.y == next.y) {
      // we found a matching node, remove everything which follows
      found = true;
    }
    if (found) {
      cellSolution.remove(i);
    }
  }
  cellSolution.add(next);
}

void goNorth() {

  if (DEBUG1) Serial.println("Going north");

  switch (direction) {
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

void goEast() {
  if (DEBUG1) Serial.println("Going east");
  switch (direction) {
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

void goSouth() {
  if (DEBUG1) Serial.println("Going south");
  switch (direction) {
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

void goWest() {
  if (DEBUG1) Serial.println("Going west");
  switch (direction) {
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

// this translates the cellSolution into a actionSolution
// cellSolution is the direction cell by cell
// action solution is a solution in terms of movements
// this version translates into a solution from home to solved
void translateSolve() {
  int dir = 0;
  //loops through every node of cell solution
  for (int i = 0; i < cellSolution.size(); i++) {
    cell current = cellSolution.get(i);
    cell next = cellSolution.get(i + 1);

    switch (dir) {

      case 0:
        if (current.y - next.y == 1) {
          //move forward
          actionSolutionToSolve.add({0, 1});
          dir = 0;
        }
        else if (current.y - next.y == -1) {
          //turn around, then move forward
          actionSolutionToSolve.add({3, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 2;
        }
        else if (current.x - next.x == 1) {
          //turn right, then move forward
          actionSolutionToSolve.add({2, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 1;
        }
        else if (current.x - next.x == -1) {
          //turn left, then move forward
          actionSolutionToSolve.add({1, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 3;
        }
        break;

      case 1:
        //go north
        if (current.y - next.y == 1) {
          actionSolutionToSolve.add({1, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 0;
        }
        //go south
        else if (current.y - next.y == -1) {
          actionSolutionToSolve.add({2, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 2;
        }
        //go east
        else if (current.x - next.x == 1) {
          actionSolutionToSolve.add({0, 1});
          dir = 1;
        }
        //go west
        else if (current.x - next.x == -1) {
          actionSolutionToSolve.add({3, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 3;
        }
        break;

      case 2:
        //go north
        if (current.y - next.y == 1) {
          actionSolutionToSolve.add({3, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 0;
        }
        //go south
        else if (current.y - next.y == -1) {

          actionSolutionToSolve.add({0, 1});
          dir = 2;
        }
        //go east
        else if (current.x - next.x == 1) {
          actionSolutionToSolve.add({1, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 1;
        }
        //go west
        else if (current.x - next.x == -1) {
          actionSolutionToSolve.add({2, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 3;
        }
        break;

      case 3:
        //go north
        if (current.y - next.y == 1) {
          actionSolutionToSolve.add({2, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 0;
        }
        //go south
        else if (current.y - next.y == -1) {
          actionSolutionToSolve.add({1, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 2;
        }
        //go east
        else if (current.x - next.x == 1) {
          actionSolutionToSolve.add({3, 0});
          actionSolutionToSolve.add({0, 1});
          dir = 1;
        }
        //go west
        else if (current.x - next.x == -1) {
          actionSolutionToSolve.add({0, 1});
          dir = 3;
        }
        break;
    }
  }
}

void translateHome() {
  int dir = 0;
  //loops through every node of cell solution
  // this version translates into a solution from solved to hom
  for (int i = cellSolution.size(); i > 0; i--) {
    cell current = cellSolution.get(i);
    cell next = cellSolution.get(i - 1);

    switch (dir) {

      case 0:
        if (current.y - next.y == 1) {
          //move forward
          actionSolutionToHome.add({0, 1});
          dir = 0;
        }
        else if (current.y - next.y == -1) {
          //turn around, then move forward
          actionSolutionToHome.add({3, 0});
          actionSolutionToHome.add({0, 1});
          dir = 2;
        }
        else if (current.x - next.x == 1) {
          //turn right, then move forward
          actionSolutionToHome.add({2, 0});
          actionSolutionToHome.add({0, 1});
          dir = 1;
        }
        else if (current.x - next.x == -1) {
          //turn left, then move forward
          actionSolutionToHome.add({1, 0});
          actionSolutionToHome.add({0, 1});
          dir = 3;
        }
        break;

      case 1:
        //go north
        if (current.y - next.y == 1) {
          actionSolutionToHome.add({1, 0});
          actionSolutionToHome.add({0, 1});
          dir = 0;
        }
        //go south
        else if (current.y - next.y == -1) {
          actionSolutionToHome.add({2, 0});
          actionSolutionToHome.add({0, 1});
          dir = 2;
        }
        //go east
        else if (current.x - next.x == 1) {
          actionSolutionToHome.add({0, 1});
          dir = 1;
        }
        //go west
        else if (current.x - next.x == -1) {
          actionSolutionToHome.add({3, 0});
          actionSolutionToHome.add({0, 1});
          dir = 3;
        }
        break;

      case 2:
        //go north
        if (current.y - next.y == 1) {
          actionSolutionToHome.add({3, 0});
          actionSolutionToHome.add({0, 1});
          dir = 0;
        }
        //go south
        else if (current.y - next.y == -1) {

          actionSolutionToHome.add({0, 1});
          dir = 2;
        }
        //go east
        else if (current.x - next.x == 1) {
          actionSolutionToHome.add({1, 0});
          actionSolutionToHome.add({0, 1});
          dir = 1;
        }
        //go west
        else if (current.x - next.x == -1) {
          actionSolutionToHome.add({2, 0});
          actionSolutionToHome.add({0, 1});
          dir = 3;
        }
        break;

      case 3:
        //go north
        if (current.y - next.y == 1) {
          actionSolutionToHome.add({2, 0});
          actionSolutionToHome.add({0, 1});
          dir = 0;
        }
        //go south
        else if (current.y - next.y == -1) {
          actionSolutionToHome.add({1, 0});
          actionSolutionToHome.add({0, 1});
          dir = 2;
        }
        //go east
        else if (current.x - next.x == 1) {
          actionSolutionToHome.add({3, 0});
          actionSolutionToHome.add({0, 1});
          dir = 1;
        }
        //go west
        else if (current.x - next.x == -1) {
          actionSolutionToHome.add({0, 1});
          dir = 3;
        }
        break;
    }
  }
}

//modifies weights to determine which has lowest values
//examples (0,0,-1,3) would create (0,0,-1,-1)
// 4,4,1,-1 would create -1,-1,1,-1
// we also return true if there is only one value left that is not -1
boolean getWeights(int weights []) {
  int min = 999;
  for (int i = 0; i < 4; i++) {
    if (weights[i] == -1) {
      //do nothing we already ruled this out
    }
    else if (weights[i] < min) min = weights[i];
  }
  //count counts the -1s
  //set non minimums to -1
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (weights[i] > min) {
      weights[i] = -1;
    }
    if (weights[i] == -1) count++;
  }

  if (DEBUG1) {
    Serial.print("weights: ");
    Serial.print(weights[0]);
    Serial.print(weights[1]);
    Serial.print(weights[2]);
    Serial.println(weights[3]);
  }


  if (count == 3) return true;
  else return false;
}

//determines if we are at a solved position (in the center of the maze
boolean isSolved() {
  if ((posx == 7 || posx == 8) && (posy == 7 || posy == 8)) {
    return true;
  }
  return false;
}

//modifies the array possibles to determine which cell i can move to next
// north, east, south, west
void getPossibles(boolean possibles[]) {
  // check the walls to determine possible moves
  int vR = analogRead(IRpinR);
  int vF = analogRead(IRpinF);
  int vL = analogRead(IRpinL);

  boolean poss[] = {0, 0, 0, 0};

  bool left = vL > 240;
  bool front = vF > 240;
  bool right = vR > 240;

  if (!left) {
    // no left wall
    poss[0] = 1;
  }
  if (!front) {
    // no front wall
    poss[1] = 1;
  }
  if (!right) {
    // no right wall
    poss[2] = 1;
  }

  if (poss[0] == 0 && poss[1] == 0 && poss[2] == 0) {
    visits[posx][posy] = 999;
  }

  switch (direction) {
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


//move the motors using encoders for distance measurement
// distance right motor, distance left, speed right, speed left
void move(int dR, int dL, int sR, int sL) {
  while ((abs(enc.getCountsM1()) < dR) or (abs(enc.getCountsM2()) < dL)) {
    setMotors(sR, sL);
  }
  offReset();
}

// turn the mouse around
// also sets direction
void turnAround() {
  move(945, -945, -150, 100);
  offReset();
  move(-30, -30, -100, -100);
  switch (direction) {
    case 0:
      direction = 2;
      break;
    case 1:
      direction = 3;
      break;
    case 2:
      direction = 0;
      break;
    case 3:
      direction = 1;
      break;
  }
}

// turn the mouse right
// also sets direction
void turnRight() {

  move(400, -400, -150, 100);
  if (direction == 3) {
    direction = 0;
  }
  else {
    direction++;
  }
}

// turn the mouse left
// also sets direction
void turnLeft() {

  move(-365, 365, 100, -115);
  if (direction == 0) {
    direction = 3;
  }
  else {
    direction--;
  }
}

//moves the mouse forward x cells
//also sets the new position
//TODO set visits for movements longer than 1 cell
void forward(int cells) {
  //update positions
  switch (direction) {
    case 0:
      posy = posy - cells;
      break;
    case 1:
      posx = posx + cells;
      break;
    case 2:
      posy = posy + cells;
      break;
    case 3:
      posx = posx - cells;
      break;
  }

  int dist = 0;

  while (dist < 120 * cells) {

    dist = ((enc.getCountsM1() / 12) + (enc.getCountsM2() / 12)) / 2;
    int vR = analogRead(IRpinR);
    int vF = analogRead(IRpinF);
    int vL = analogRead(IRpinL);

    int err;
    int errR;
    int errL;

    if (vF > 400) {
      offReset();
      break;
    }

    if ((vL < 220) && (vR < 220)) {
      if (DEBUG) Serial.println("No walls");
      errL = enc.getCountsM2() - enc.getCountsM1();
      errR = errL;
    }
    else if (vR < 200) {
      if (DEBUG) Serial.println("No right wall");
      errR = -(430 - vL);
      errL = 0;
    }
    else if (vL < 200) {
      if (DEBUG) Serial.println("No left wall");
      errL = -(430 - vR);
      errR = 0;
    }
    else {
      if (DEBUG) Serial.println("2 walls");

      errR = -(430 - vL);
      errL = -(430 - vR);
    }


    int fixedR = map(errR, -128, 128, -30, 30);
    int fixedL = map(errL, -128, 128, -30, 30);

    setMotors(speed - fixedR, speed - fixedL);

  }

  offReset();
}

//sets motors to speed right, speed left
void setMotors(int speedR, int speedL) {
  bool rRev = speedR > 0;
  bool lRev = speedL > 0;

  digitalWrite(directionPinR, rRev);
  digitalWrite(directionPinL, lRev);
  delay(5);
  analogWrite(speedPinR, abs(speedR));
  analogWrite(speedPinL, abs(speedL - 15)); //-20 here
}

//shuts off motors, resets encoder counts and waits for stop
void offReset() {
  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);
  delay(200);
  enc.getCountsAndResetM1();
  enc.getCountsAndResetM2();

}

//waits for a button press
//also accounts for debouncing
void waitBtn() {
  while (1) {
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
          digitalWrite(13, LOW);
          delay(500);
          return;
        }
      }
    }
    lastButtonState = reading;
  }
}

//generate maze data needed for maze algorithms
void generateMaze() {
  for (int i = 0; i < mazeSize ; i++) {
    for (int j = 0; j < mazeSize ; j++) {
      maze[i][j] = gen(i, j);
      if (DEBUG1)  Serial.print(maze[i][j]);
      if (DEBUG1)  Serial.print(", ");
    }
    if (DEBUG1) Serial.println();

  }
  if (DEBUG1) Serial.println("-------------------");
  for (int i = 0; i < mazeSize ; i++) {
    for (int j = 0; j < mazeSize ; j++) {
      if (i == 0 && j == 15) {
        visits[i][j] = 1;
      }
      else visits[i][j] = 0;
      if (DEBUG1) Serial.print(visits[i][j]);
      if (DEBUG1) Serial.print(", ");
    }
    if (DEBUG1) Serial.println();
  }
}

//generates gravity maze values
int gen(unsigned short row1, unsigned short col1) {
  unsigned short nr;
  unsigned short nc;

  short half = mazeSize / 2;
  short halfless1 = (mazeSize / 2) - 1;
  if (row1 > halfless1) {
    nr = row1 - half;
    row1 = halfless1 - nr;
  }
  if (col1 > halfless1) {
    nc = col1 - half;
    col1 = halfless1 - nc;
  }
  return (mazeSize - (col1) - (row1));
}

void printMazes() {
  Serial.print("You are here: ");
  Serial.print(posx);
  Serial.print(" ");
  Serial.println(posy);
  Serial.println("Visits:");
  for (int i = 0; i < mazeSize ; i++) {
    for (int j = 0; j < mazeSize ; j++) {
      Serial.print(maze[i][j]);
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println("-------------------");
  Serial.println("Gravity");
  for (int i = 0; i < mazeSize ; i++) {
    for (int j = 0; j < mazeSize ; j++) {
      Serial.print(visits[j][i]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

