// ============================================================
//  Robot Solver 8x8 Maze using Flood Fill - No Delay
//  - Using millis() timers instead of delay
// ============================================================

#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ===================== Maze Dimensions =====================
#define MAZE_SIZE 8
#define CELL_SIZE 7.1  // cm
#define WALL_THRESHOLD 12.0  // Distance considered a wall (cm)

// ===================== Direction Definitions =====================
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
const char* dirNames[] = {"North", "East", "South", "West"};

// ===================== MOTOR PINS =====================
#define ENA  27
#define ENB  14
#define IN1  26
#define IN2  25
#define IN3  13
#define IN4  12

// ===================== ENCODER PINS =====================
#define ENCODER_A_PIN_A 34
#define ENCODER_A_PIN_B 35
#define ENCODER_B_PIN_A 32
#define ENCODER_B_PIN_B 33

// ===================== LIDAR XSHUT PINS =====================
#define XSHUT_FRONT 4
#define XSHUT_RIGHT 16
#define XSHUT_LEFT  17

#define ADDR_FRONT 0x30
#define ADDR_RIGHT 0x31
#define ADDR_LEFT  0x32

// ===================== MPU6050 =====================
#define MPU_ADDR 0x68
float rawYawAngle  = 0.0f;
float yawOffset    = 0.0f;
unsigned long prev_time = 0;

// ===================== ENCODERS =====================
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// ===================== Movement Settings =====================
int   MOTOR_SPEED         = 60;
float Kp_straight         = 2.5;
int   TURN_SPEED          = 55;

const float WHEEL_DIAMETER = 4.5;
const float TICKS_PER_REV  = 412.1;
const float CM_PER_TICK    = 0.0343053;

// ===================== Maze Structure =====================
struct MazeCell {
  bool walls[4];  // North, East, South, West
  int distance;
  bool visited;
};

MazeCell maze[MAZE_SIZE][MAZE_SIZE];
int robotX = 0, robotY = 0;  // Robot position in the maze
Direction robotDir = EAST;    // Current robot direction
int targetX = MAZE_SIZE - 1, targetY = MAZE_SIZE - 1;  // Target (last cell)

// ===================== LIDAR OBJECTS =====================
VL53L0X sensorFront;
VL53L0X sensorRight;
VL53L0X sensorLeft;

// ===================== Path Variables =====================
long baseTicksA = 0;
long baseTicksB = 0;
float startYaw = 0.0f;
int frontHitCount = 0;
const int FRONT_HIT_NEED = 2;

// ===================== Timer Variables =====================
unsigned long lastPrintTime = 0;
unsigned long lastReadTime = 0;
unsigned long turnStartTime = 0;
unsigned long moveStartTime = 0;
unsigned long stopStartTime = 0;
unsigned long pauseStartTime = 0;
unsigned long obstacleDelayStart = 0;
bool isTurning = false;
bool isMoving = false;
bool isStopped = false;
bool isPaused = false;
bool obstacleDelayActive = false;
float turnTargetYaw = 0;
Direction turnTargetDir;
int turnStep = 0;  // for turn180

// ===================== ENCODER ISR =====================
void IRAM_ATTR encoderA_ISR() {
  bool pinA = digitalRead(ENCODER_A_PIN_A);
  bool pinB = digitalRead(ENCODER_A_PIN_B);
  if (pinA == pinB) encoderCountA++;
  else encoderCountA--;
}

void IRAM_ATTR encoderB_ISR() {
  bool pinA = digitalRead(ENCODER_B_PIN_A);
  bool pinB = digitalRead(ENCODER_B_PIN_B);
  if (pinA == pinB) encoderCountB++;
  else encoderCountB--;
}

// ===================== MPU6050 =====================
int16_t readMPUReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

float getYawAngle() {
  int16_t gyro_z = readMPUReg(0x47);
  float gz_dps = gyro_z / 131.0f;
  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0f;
  prev_time = now;
  rawYawAngle += gz_dps * dt;
  return rawYawAngle - yawOffset;
}

// ===================== MOTOR CONTROL =====================
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void driveForward(int speedL, int speedR) {
  analogWrite(ENA, constrain(speedL, 0, 255));
  analogWrite(ENB, constrain(speedR, 0, 255));
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRightRaw(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void turnLeftRaw(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

// ===================== LIDAR READ =====================
float getDistCM(VL53L0X &sensor) {
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return 9999.0f;
  return mm / 10.0f;
}

// ===================== HELPERS =====================
float segmentDistanceCm() {
  float distA = (encoderCountA - baseTicksA) * CM_PER_TICK;
  float distB = (encoderCountB - baseTicksB) * CM_PER_TICK;
  return fabs((distA + distB) / 2.0f);
}

void startNewSegment() {
  baseTicksA = encoderCountA;
  baseTicksB = encoderCountB;
  startYaw = getYawAngle();
}

// ===================== Directional Functions =====================
Direction getLeftDir(Direction dir) {
  return (Direction)((dir + 3) % 4);
}

Direction getRightDir(Direction dir) {
  return (Direction)((dir + 1) % 4);
}

Direction getOppositeDir(Direction dir) {
  return (Direction)((dir + 2) % 4);
}

// ===================== Wall Reading =====================
void readWalls(bool &front, bool &right, bool &left) {
  float frontDist = getDistCM(sensorFront);
  float rightDist = getDistCM(sensorRight);
  float leftDist = getDistCM(sensorLeft);
  
  front = (frontDist < WALL_THRESHOLD);
  right = (rightDist < WALL_THRESHOLD);
  left = (leftDist < WALL_THRESHOLD);
  
  if (millis() - lastPrintTime > 300) {
    Serial.print("Sensors - Front: "); Serial.print(frontDist);
    Serial.print("cm ("); Serial.print(front ? "WALL" : "OPEN");
    Serial.print(") | Right: "); Serial.print(rightDist);
    Serial.print("cm ("); Serial.print(right ? "WALL" : "OPEN");
    Serial.print(") | Left: "); Serial.print(leftDist);
    Serial.print("cm ("); Serial.print(left ? "WALL" : "OPEN");
    Serial.println(")");
    lastPrintTime = millis();
  }
}

// ===================== Update Maze =====================
void updateMazeWalls() {
  bool frontWall, rightWall, leftWall;
  readWalls(frontWall, rightWall, leftWall);
  
  // Update walls based on current direction
  maze[robotX][robotY].walls[robotDir] = frontWall;
  maze[robotX][robotY].walls[getRightDir(robotDir)] = rightWall;
  maze[robotX][robotY].walls[getLeftDir(robotDir)] = leftWall;
  
  // Update the opposite wall in the adjacent cell if it exists
  if (!frontWall) {
    int nx = robotX, ny = robotY;
    switch(robotDir) {
      case NORTH: ny++; break;
      case EAST:  nx++; break;
      case SOUTH: ny--; break;
      case WEST:  nx--; break;
    }
    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
      maze[nx][ny].walls[getOppositeDir(robotDir)] = frontWall;
    }
  }
  
  maze[robotX][robotY].visited = true;
  
  if (millis() - lastPrintTime > 300) {
    Serial.print("Cell ["); Serial.print(robotX);
    Serial.print("]["); Serial.print(robotY);
    Serial.print("] updated. Walls: N=");
    Serial.print(maze[robotX][robotY].walls[NORTH] ? "1" : "0");
    Serial.print(" E=");
    Serial.print(maze[robotX][robotY].walls[EAST] ? "1" : "0");
    Serial.print(" S=");
    Serial.print(maze[robotX][robotY].walls[SOUTH] ? "1" : "0");
    Serial.print(" W=");
    Serial.print(maze[robotX][robotY].walls[WEST] ? "1" : "0");
    Serial.println();
  }
}

// ===================== Flood Fill Algorithm =====================
void floodFill() {
  Serial.println("=== Running Flood Fill ===");
  
  // Initialize all cells with a large distance
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      maze[x][y].distance = 999;
    }
  }
  
  // Set target cell distance = 0
  maze[targetX][targetY].distance = 0;
  
  bool changed;
  do {
    changed = false;
    for (int x = 0; x < MAZE_SIZE; x++) {
      for (int y = 0; y < MAZE_SIZE; y++) {
        int currentDist = maze[x][y].distance;
        int minNeighbor = 999;
        
        // Check the four neighbors
        if (y < MAZE_SIZE-1 && !maze[x][y].walls[NORTH]) // North
          minNeighbor = min(minNeighbor, maze[x][y+1].distance);
        if (x < MAZE_SIZE-1 && !maze[x][y].walls[EAST])  // East
          minNeighbor = min(minNeighbor, maze[x+1][y].distance);
        if (y > 0 && !maze[x][y].walls[SOUTH])           // South
          minNeighbor = min(minNeighbor, maze[x][y-1].distance);
        if (x > 0 && !maze[x][y].walls[WEST])            // West
          minNeighbor = min(minNeighbor, maze[x-1][y].distance);
        
        int newDist = minNeighbor + 1;
        if (newDist < currentDist) {
          maze[x][y].distance = newDist;
          changed = true;
        }
      }
    }
  } while (changed);
  
  // Print Maze
  printMaze();
}

// ===================== Maze Printing =====================
void printMaze() {
  Serial.println("\n=== Maze State ===");
  for (int y = MAZE_SIZE-1; y >= 0; y--) {
    String line = "";
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (maze[x][y].distance < 10)
        line += " ";
      line += String(maze[x][y].distance);
      if (x == robotX && y == robotY)
        line += "* ";
      else
        line += "  ";
    }
    Serial.println(line);
  }
  Serial.println("=================");
}

// ===================== Determine Next Step =====================
Direction getNextDirection() {
  int currentDist = maze[robotX][robotY].distance;
  Direction bestDir = robotDir;
  int minDist = 999;
  
  // Check all possible directions
  if (!maze[robotX][robotY].walls[NORTH] && robotY < MAZE_SIZE-1) {
    if (maze[robotX][robotY+1].distance < minDist) {
      minDist = maze[robotX][robotY+1].distance;
      bestDir = NORTH;
    }
  }
  if (!maze[robotX][robotY].walls[EAST] && robotX < MAZE_SIZE-1) {
    if (maze[robotX+1][robotY].distance < minDist) {
      minDist = maze[robotX+1][robotY].distance;
      bestDir = EAST;
    }
  }
  if (!maze[robotX][robotY].walls[SOUTH] && robotY > 0) {
    if (maze[robotX][robotY-1].distance < minDist) {
      minDist = maze[robotX][robotY-1].distance;
      bestDir = SOUTH;
    }
  }
  if (!maze[robotX][robotY].walls[WEST] && robotX > 0) {
    if (maze[robotX-1][robotY].distance < minDist) {
      minDist = maze[robotX-1][robotY].distance;
      bestDir = WEST;
    }
  }
  
  return bestDir;
}

// ===================== Turn functions with Timer =====================
void startTurnRight() {
  Serial.println(">>> Starting RIGHT turn...");
  stopMotors();
  isStopped = true;
  stopStartTime = millis();
  turnTargetDir = getRightDir(robotDir);
  turnStep = 1;
}

void startTurnLeft() {
  Serial.println(">>> Starting LEFT turn...");
  stopMotors();
  isStopped = true;
  stopStartTime = millis();
  turnTargetDir = getLeftDir(robotDir);
  turnStep = 1;
}

void startTurn180() {
  Serial.println(">>> Starting 180 turn...");
  stopMotors();
  isStopped = true;
  stopStartTime = millis();
  turnTargetDir = getOppositeDir(robotDir);
  turnStep = 1;
}

void updateTurn() {
  if (isStopped) {
    if (millis() - stopStartTime >= 150) {
      isStopped = false;
      isTurning = true;
      turnStartTime = millis();
      prev_time = millis();
      
      float yawBefore = getYawAngle();
      if (turnStep == 1) {
        if (turnTargetDir == getRightDir(robotDir)) {
          Serial.println(">>> Executing RIGHT turn...");
          turnTargetYaw = yawBefore - 75.0f;
        } else if (turnTargetDir == getLeftDir(robotDir)) {
          Serial.println(">>> Executing LEFT turn...");
          turnTargetYaw = yawBefore + 75.0f;
        }
      }
    }
    return;
  }
  
  if (isTurning) {
    float currentYaw = getYawAngle();
    float remaining;
    
    if (turnTargetDir == getRightDir(robotDir)) {
      remaining = currentYaw - turnTargetYaw;
    } else {
      remaining = turnTargetYaw - currentYaw;
    }
    
    // Check if required angle reached
    if (fabs(remaining) <= 2.0f || millis() - turnStartTime > 3000) {
      if (fabs(remaining) <= 2.0f) {
        Serial.println("Turn complete");
      } else {
        Serial.println("Turn timeout!");
      }
      
      stopMotors();
      isTurning = false;
      isStopped = true;
      stopStartTime = millis();
      
      if (turnStep == 1 && turnTargetDir == getOppositeDir(robotDir)) {
        // For 180 degrees, wait then start the second turn
        turnStep = 2;
      } else {
        // Turning complete
        robotDir = turnTargetDir;
        isStopped = false;
        obstacleDelayActive = false;
        startNewSegment();
      }
      return;
    }
    
    int dynSpeed = TURN_SPEED;
    if (fabs(remaining) < 20.0f) {
      dynSpeed = max(40, TURN_SPEED / 2);
    }
    
    if (turnTargetDir == getRightDir(robotDir)) {
      turnRightRaw(dynSpeed);
    } else {
      turnLeftRaw(dynSpeed);
    }
  }
}

// ===================== Guidance based on Sensors =====================
void handleObstacle() {
  bool front, right, left;
  readWalls(front, right, left);
  
  Serial.println("=== Obstacle Detection ===");
  Serial.print("Front: "); Serial.print(front ? "WALL" : "OPEN");
  Serial.print(" | Right: "); Serial.print(right ? "WALL" : "OPEN");
  Serial.print(" | Left: "); Serial.println(left ? "WALL" : "OPEN");
  
  if (front) {
    if (!right) {
      // Front blocked + Right open -> Turn Right
      Serial.println("Decision: Front blocked, Right open -> TURN RIGHT");
      startTurnRight();
    }
    else if (!left) {
      // Front blocked + Left open -> Turn Left
      Serial.println("Decision: Front blocked, Left open -> TURN LEFT");
      startTurnLeft();
    }
    else {
      // Front + Right + Left all blocked -> Turn 180 (Go back)
      Serial.println("Decision: All directions blocked -> TURN 180");
      startTurn180();
    }
    obstacleDelayActive = true;
  }
}

// ===================== Move forward with Timer =====================
void startMoveForward() {
  Serial.println("Starting forward movement...");
  startNewSegment();
  isMoving = true;
  moveStartTime = millis();
}

void updateMoveForward() {
  if (!isMoving) return;
  
  // Read sensors periodically
  if (millis() - lastReadTime > 50) {
    bool front, right, left;
    readWalls(front, right, left);
    lastReadTime = millis();
    
    if (front) {
      Serial.println("Unexpected obstacle while moving!");
      stopMotors();
      isMoving = false;
      isStopped = true;
      stopStartTime = millis();
      handleObstacle();
      return;
    }
  }
  
  if (segmentDistanceCm() >= CELL_SIZE) {
    stopMotors();
    isMoving = false;
    isStopped = true;
    stopStartTime = millis();
    
    // Update position
    switch(robotDir) {
      case NORTH: robotY++; break;
      case EAST:  robotX++; break;
      case SOUTH: robotY--; break;
      case WEST:  robotX--; break;
    }
    
    Serial.print("Now at cell ["); Serial.print(robotX);
    Serial.print("]["); Serial.print(robotY);
    Serial.println("]");
    return;
  }
  
  float currentYaw = getYawAngle();
  float yawError = currentYaw - startYaw;
  int correction = (int)(Kp_straight * yawError);
  
  int speedL = MOTOR_SPEED + correction;
  int speedR = MOTOR_SPEED - correction;
  
  driveForward(speedL, speedR);
}

// ===================== SETUP =====================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  
  Serial.println("=== Maze Solver Robot - Timer Version ===");
  Serial.print("Maze size: "); Serial.print(MAZE_SIZE);
  Serial.print("x"); Serial.print(MAZE_SIZE);
  Serial.print(", Cell size: "); Serial.print(CELL_SIZE);
  Serial.println(" cm");

  // Initialize Maze
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int d = 0; d < 4; d++) {
        maze[x][y].walls[d] = false;
      }
      maze[x][y].distance = 999;
      maze[x][y].visited = false;
    }
  }

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  // Encoder pins
  pinMode(ENCODER_A_PIN_A, INPUT);
  pinMode(ENCODER_A_PIN_B, INPUT);
  pinMode(ENCODER_B_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_A), encoderB_ISR, CHANGE);

  // XSHUT pins
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT,  OUTPUT);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT,  LOW);
  delay(10);

  Wire.begin(21, 22);

  // LiDAR init
  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  sensorFront.init();
  sensorFront.setAddress(ADDR_FRONT);
  sensorFront.setTimeout(300);
  sensorFront.startContinuous(30);
  Serial.println("LiDAR Front OK");

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  sensorRight.init();
  sensorRight.setAddress(ADDR_RIGHT);
  sensorRight.setTimeout(300);
  sensorRight.startContinuous(30);
  Serial.println("LiDAR Right OK");

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  sensorLeft.init();
  sensorLeft.setAddress(ADDR_LEFT);
  sensorLeft.setTimeout(300);
  sensorLeft.startContinuous(30);
  Serial.println("LiDAR Left OK");

  // MPU init
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  prev_time = millis();
  Serial.print("Calibrating gyro");
  for (int i = 0; i < 100; i++) {
    getYawAngle();
    if (i % 20 == 0) Serial.print(".");
    delay(10);
  }
  yawOffset = rawYawAngle;
  Serial.println();
  
  startNewSegment();
  Serial.println("=== START MAZE SOLVING ===");
}

// ===================== Main Loop =====================
void loop() {
  static enum { EXPLORE, SOLVE } mode = EXPLORE;
  static int stuckCounter = 0;
  static bool waitingForNextAction = false;
  static unsigned long nextActionTime = 0;
  
  // Update active movements first
  if (isTurning) {
    updateTurn();
    return;
  }
  
  if (isMoving) {
    updateMoveForward();
    return;
  }
  
  if (isStopped) {
    if (millis() - stopStartTime >= 200) {
      isStopped = false;
    }
    return;
  }
  
  if (obstacleDelayActive) {
    return;
  }
  
  if (waitingForNextAction) {
    if (millis() - nextActionTime >= 500) {
      waitingForNextAction = false;
    } else {
      return;
    }
  }
  
  // Main Logic
  if (mode == EXPLORE) {
    Serial.println("\n=== EXPLORE MODE ===");
    
    // Read and update walls
    updateMazeWalls();
    
    // Check for obstacle
    bool front, right, left;
    readWalls(front, right, left);
    
    if (front) {
      // Obstacle detected - handle according to rules
      frontHitCount++;
      if (frontHitCount >= FRONT_HIT_NEED) {
        frontHitCount = 0;
        handleObstacle();
        return;
      }
    } else {
      frontHitCount = 0;
    }
    
    // Run Flood Fill
    floodFill();
    
    // Check if target reached
    if (robotX == targetX && robotY == targetY) {
      Serial.println("*** REACHED TARGET! ***");
      mode = SOLVE;
      waitingForNextAction = true;
      nextActionTime = millis();
      return;
    }
    
    // Determine next direction
    Direction nextDir = getNextDirection();
    Serial.print("Current Dir: "); Serial.print(dirNames[robotDir]);
    Serial.print(" | Next Dir: "); Serial.println(dirNames[nextDir]);
    
    // Move according to required direction
    if (nextDir != robotDir) {
      // Change direction
      int diff = (nextDir - robotDir + 4) % 4;
      if (diff == 1) { // Right
        startTurnLeft();
      } else if (diff == 2) { // Back
        startTurn180();
      } else if (diff == 3) { // Left
        startTurnRight();
      }
      return;
    }
    
    // Move forward if path is open
    readWalls(front, right, left);
    if (!front) {
      startMoveForward();
    } else {
      stuckCounter++;
      if (stuckCounter > 3) {
        handleObstacle();
        stuckCounter = 0;
      }
    }
    
    waitingForNextAction = true;
    nextActionTime = millis();
  }
  
  else if (mode == SOLVE) {
    Serial.println("\n=== SOLVE MODE (Returning to start) ===");
    
    // Restart Flood Fill to return to start
    targetX = 0;
    targetY = 0;
    floodFill();
    
    // Move according to shortest path
    Direction nextDir = getNextDirection();
    
    if (nextDir != robotDir) {
      int diff = (nextDir - robotDir + 4) % 4;
      if (diff == 1) { // Right
        startTurnLeft();
      } else if (diff == 2) { // Back
        startTurn180();
      } else if (diff == 3) { // Left
        startTurnRight();
      }
      return;
    }
    
    bool front, right, left;
    readWalls(front, right, left);
    if (!front) {
      startMoveForward();
    }
    
    // Check if back to start
    if (robotX == 0 && robotY == 0) {
      Serial.println("*** BACK TO START! MAZE COMPLETE! ***");
      stopMotors();
      while(1) {
        delay(1000);
        Serial.println("Maze solved successfully!");
      }
    }
    
    waitingForNextAction = true;
    nextActionTime = millis();
  }
}