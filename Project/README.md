# Micromouse Maze Solver - BZU 2022

This project is an autonomous maze-solving robot designed for the Micromouse Competition at Birzeit University (BZU). The robot utilizes the **Flood Fill algorithm** to explore an 8x8 maze and find the shortest path to the target cell.

## Features

- **Algorithm**: Implements the Flood Fill algorithm for efficient maze exploration and shortest-path calculation.
- **Hardware Platform**: Built on the ESP32 microcontroller using the Arduino framework.
- **Sensors**:
  - **3x VL53L0X LiDAR Sensors**: Used for high-precision wall detection (Front, Right, and Left).
  - **MPU6050 IMU**: Provides yaw angle tracking for precise 90-degree and 180-degree turns.
  - **Quadrature Encoders**: Used for distance measurement and straight-line correction.
- **Movement**: Dual DC motor setup with PWM speed control and PID-based straight-line stabilization.
- **Two Modes of Operation**:
  - **Explore Mode**: The robot navigates the maze, mapping walls and updating its internal grid.
  - **Solve Mode**: Once the target is reached, the robot calculates the optimal path back to the start or to the goal for a fast run.

## Hardware Configuration

### Motor Pins
| Component | ESP32 Pin |
|-----------|-----------|
| ENA (Left PWM) | 27 |
| ENB (Right PWM) | 14 |
| IN1 / IN2 | 26 / 25 |
| IN3 / IN4 | 13 / 12 |

### Sensor Pins
| Component | ESP32 Pin |
|-----------|-----------|
| XSHUT Front | 4 |
| XSHUT Right | 16 |
| XSHUT Left | 17 |
| I2C SDA | 21 |
| I2C SCL | 22 |

### Encoder Pins
| Component | Pin A | Pin B |
|-----------|-------|-------|
| Encoder A | 34 | 35 |
| Encoder B | 32 | 33 |

## Specifications

- **Maze Size**: 8x8 cells.
- **Cell Dimension**: 7.1 cm.
- **Wall Threshold**: 12.0 cm (distance to consider a wall present).
- **Communication**: Serial debugging at 115200 baud.

## How it Works

1.  **Initialization**: Calibrates the MPU6050 gyro and initializes the three LiDAR sensors with unique I2C addresses.
2.  **Wall Sensing**: At each cell, the robot reads the front, left, and right distances. If a distance is below the `WALL_THRESHOLD`, a wall is recorded in the internal map.
3.  **Flood Fill**: The robot updates the "distance-to-target" values for all cells in the maze based on known walls.
4.  **Navigation**: The robot chooses the neighbor with the lowest distance value and turns/moves toward it.
5.  **Completion**: Upon reaching the target cell (7,7), the robot switches to Solve Mode to return to the start (0,0) using the discovered shortest path.

## Dependencies

- [VL53L0X Library](https://github.com/pololu/vl53l0x-arduino)
- Arduino Wire Library
- ESP32 Core for Arduino

---
*Developed for the Micromouse Competition BZU 22.*
